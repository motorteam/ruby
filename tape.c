/**********************************************************************

  tape.c - deterministic record/replay of observable behavior

  See tape.h for the model, and RUBY.md for the design and wire format.

  The tape records every crossing of the program->host boundary and nothing
  else. Replay re-runs the program, but each chokepoint serves its result from
  the tape instead of calling libc -- so no effect is ever re-executed and the
  run is reproduced exactly.

  Wire format (Watt's, byte for byte):

      byte 0   : format version (2)
      bytes 1..: zstd( postcard( SpooledTape ) )

  postcard is non-self-describing: fields emit in declaration order, integers
  are LEB128 varints (zigzag for signed), Vec/String are varint-length-prefixed,
  Option is 0x00 or 0x01 ++ payload.

**********************************************************************/

#include "ruby/internal/config.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <unistd.h>

#include <zstd.h>

#include "internal.h"
#include "ruby/ruby.h"
#include "ruby/thread_native.h"
#include "ruby/version.h"
#include "tape.h"
#include "vm_core.h"

/* Matches Watt's CURRENT_SPOOL_FORMAT. */
#define TAPE_FORMAT_VERSION 2
#define TAPE_ZSTD_LEVEL 3

/* Bytes a tape may reach before it is dropped whole. A tape over this can't be
 * partially kept -- a truncated observation diverges on replay -- so we discard
 * it rather than write something that would mislead. Matches Watt's ceiling. */
#define TAPE_CEILING_BYTES (64 * 1024 * 1024)

/* The fqn of each effect, in `enum rb_tape_effect` order. Written to the tape's
 * catalog so a reader can name a func_index without this binary. */
static const char *const tape_effect_fqn[] = {
    "clock.realtime",
    "clock.monotonic",
    "random.bytes",
    "io.read",
    "io.write",
    "kernel.halt",
    "kernel.abort",
    "fs.open",
    "fs.close",
    "fs.stat",
    "fs.lstat",
    "fs.fstat",
    "fs.isatty",
    "fs.lseek",
    "fs.opendir",
    "fs.readdir",
    "fs.closedir",
    "env.get",
    "proc.getpid",
    "fs.mutate",
};

/* Effect signatures, for the Signature column of `--tape-inspect`. Ruby is
 * dynamically typed, but these effects are not: each has a fixed C shape, so we
 * can name it the way Watt names an external's type. `[[byte]]` is Watt's
 * spelling of an iov (a gather or scatter buffer). */
static const char *const tape_effect_sig[] = {
    "() -> (int, int)",         /* clock.realtime  -> (tv_sec, tv_nsec) */
    "() -> (int, int)",         /* clock.monotonic */
    "(size, [[byte]]) -> int",  /* random.bytes    -- scatter: the filled buffer */
    "(int, [[byte]]) -> int",   /* io.read         -- scatter: the bytes read */
    "(int, [[byte]]) -> int",   /* io.write        -- gather: the bytes written */
    "() -> never",              /* kernel.halt */
    "() -> never",              /* kernel.abort */
    "([[byte]], int) -> int",   /* fs.open   -- gather: the path;  -> fd */
    "(int) -> int",             /* fs.close */
    "([[byte]], [[byte]]) -> int", /* fs.stat   -- gather: path; scatter: struct stat */
    "([[byte]], [[byte]]) -> int", /* fs.lstat */
    "(int, [[byte]]) -> int",   /* fs.fstat  -- scatter: struct stat */
    "(int) -> bool",            /* fs.isatty */
    "(int, int, int) -> int",   /* fs.lseek -- (fd, whence, offset) -> position */
    "([[byte]]) -> int",        /* fs.opendir -- gather: the path */
    "() -> [[byte]]",           /* fs.readdir -- scatter: the entry name */
    "() -> int",                /* fs.closedir */
    "([[byte]]) -> [[byte]]",   /* env.get -- gather: name; scatter: value */
    "() -> int",                /* proc.getpid */
    "(op, [[byte]]) -> int",    /* fs.mutate -- gather: the path(s); op names which */
};

/* ── Allocation ───────────────────────────────────────────────────────────────
 *
 * The tape allocates with plain libc, not with CRuby's xmalloc, and that is a
 * correctness requirement rather than a preference.
 *
 * The recorder runs *with the GVL released*. `internal_read_func` and
 * `internal_write_func` (io.c) are the functions handed to rb_nogvl -- that is
 * the whole point of them -- and they call in here from inside that region.
 * `xmalloc` may trigger a garbage collection, and a garbage collection needs the
 * GVL. So every xmalloc on the recording path was a latent crash.
 *
 * Nor does the tape belong on the GC's books: it is raw bytes with a lifetime
 * that ends at rb_tape_finish, holding no VALUEs and tracing no references.
 */
static void *
tape_realloc(void *p, size_t n)
{
    void *q = realloc(p, n);
    if (!q && n) {
        fputs("[tape] out of memory\n", stderr);
        _exit(EXIT_FAILURE);
    }
    return q;
}

static void *
tape_malloc(size_t n) { return tape_realloc(NULL, n); }

static void *
tape_calloc(size_t count, size_t size)
{
    void *p = calloc(count ? count : 1, size);
    if (!p) {
        fputs("[tape] out of memory\n", stderr);
        _exit(EXIT_FAILURE);
    }
    return p;
}

/* ── Growable byte buffer ─────────────────────────────────────────────────── */

typedef struct {
    uint8_t *ptr;
    size_t len;
    size_t cap;
} tape_buf;

static void
buf_reserve(tape_buf *b, size_t extra)
{
    if (b->len + extra <= b->cap) return;
    size_t cap = b->cap ? b->cap : 64;
    while (cap < b->len + extra) cap *= 2;
    b->ptr = tape_realloc(b->ptr, cap);
    b->cap = cap;
}

static void
buf_push(tape_buf *b, const void *p, size_t n)
{
    if (n == 0) return;
    buf_reserve(b, n);
    memcpy(b->ptr + b->len, p, n);
    b->len += n;
}

static void
buf_byte(tape_buf *b, uint8_t v)
{
    buf_reserve(b, 1);
    b->ptr[b->len++] = v;
}

static void
buf_free(tape_buf *b)
{
    free(b->ptr);
    b->ptr = NULL;
    b->len = b->cap = 0;
}

static void
sb_printf(tape_buf *b, const char *fmt, ...)
{
    va_list args;
    char chunk[512];
    va_start(args, fmt);
    int n = vsnprintf(chunk, sizeof(chunk), fmt, args);
    va_end(args);
    if (n > 0) buf_push(b, chunk, (size_t)n < sizeof(chunk) ? (size_t)n : sizeof(chunk) - 1);
}

/** A UTF-8 continuation byte -- never a character boundary. */
static int
utf8_cont_p(uint8_t b)
{
    return (b & 0xc0) == 0x80;
}

/* ── postcard primitives ──────────────────────────────────────────────────── */

/** LEB128 unsigned varint. */
static void
pc_uvarint(tape_buf *b, uint64_t v)
{
    do {
        uint8_t byte = v & 0x7f;
        v >>= 7;
        if (v) byte |= 0x80;
        buf_byte(b, byte);
    } while (v);
}

/** Zigzag, then LEB128 -- postcard's encoding for signed integers. */
static void
pc_svarint(tape_buf *b, int64_t v)
{
    pc_uvarint(b, ((uint64_t)v << 1) ^ (uint64_t)(v >> 63));
}

/** Vec<u8> / &[u8]: varint(len) ++ raw bytes. */
static void
pc_bytes(tape_buf *b, const void *p, size_t n)
{
    pc_uvarint(b, n);
    buf_push(b, p, n);
}

/** String: varint(len) ++ utf8. */
static void
pc_str(tape_buf *b, const char *s)
{
    pc_bytes(b, s, strlen(s));
}

/* ── Recorder ─────────────────────────────────────────────────────────────── */

typedef struct {
    uint8_t arg_index;
    tape_buf bytes;
} tape_iov;

typedef struct {
    int32_t func_index;
    uint8_t action;
    tape_buf args;
    tape_buf ret;
    tape_iov *iovs;
    size_t n_iovs;
    size_t bytes;   /* iov bytes this entry holds; charged to the ceiling at commit */
} tape_entry;

static struct {
    int recording;
    int replaying;
    char *path;

    /* record */
    tape_entry *entries;
    size_t n_entries;
    size_t cap_entries;
    size_t bytes;   /* against TAPE_CEILING_BYTES */
    int dropped;

    /* replay */
    size_t cursor;

    /* Loading the program is not running the program. Nonzero while the loader
     * is reaching for program text; every chokepoint goes transparent. */
    int paused;
} tape;

/*
 * Chokepoints reach this file from GVL-released regions (see tape_realloc), so
 * two Ruby threads doing IO at once arrive here genuinely concurrently. The GVL
 * is not holding anything still for us; this lock is.
 *
 * It guards the shared tape only -- the entries array, the byte count, the
 * replay cursor. An entry is *built* outside it (that is the expensive part: the
 * memcpy of a read buffer) and only the append is serialized.
 */
static rb_nativethread_lock_t tape_lock;

/*
 * The timer thread is not the program.
 *
 * It wakes on a wall-clock schedule -- every few milliseconds, whether or not the
 * program did anything -- and calls rb_hrtime_now() to decide which sleeping
 * thread is due (timer_thread_check_timeout, thread_pthread.c:3164). The moment
 * the monotonic clock went on the tape, those reads went on it too, injecting
 * entries at points with no relationship to the program's own effect stream: a
 * tape would record `... fs.close, clock.monotonic, fs.open ...` and the next run
 * would produce `... fs.close, fs.open ...` because the timer happened to fire a
 * microsecond later. 130 test files diverged exactly this way, in both directions.
 *
 * An effect belongs on the tape when the program asked for it. VM infrastructure
 * running on a timer did not ask for anything, and its clock reads never flow back
 * into the program -- they only decide *when* a sleeping thread is woken, which
 * the tape already pins by other means.
 */
static rb_nativethread_id_t tape_vm_thread;
static int tape_vm_thread_known;

void
rb_tape_thread_off(void)
{
    tape_vm_thread = rb_nativethread_self();
    tape_vm_thread_known = 1;
}

static int
tape_on_vm_thread(void)
{
    return tape_vm_thread_known &&
           rb_nativethread_self() == tape_vm_thread;
}

int rb_tape_recording(void) { return tape.recording && !tape.dropped && !tape.paused && !tape_on_vm_thread(); }
int rb_tape_replaying(void) { return tape.replaying && !tape.paused && !tape_on_vm_thread(); }

/*
 * A `require` reaches the filesystem twice, and only one of the two halves is
 * visible from this file. `rb_file_load_ok` (file.c) probes each candidate path
 * in $LOAD_PATH with rb_cloexec_open -- which is a chokepoint, so it lands on the
 * tape -- and then Prism reads the source it settled on with a raw open(2) +
 * mmap(2) of its own (prism/source.c:173,213), which no chokepoint can see. The
 * result was an open on the tape whose read and close never arrived, so replay
 * handed the loader a fd that was never opened and the effect stream desynced on
 * the very next call.
 *
 * The fix is not to chase Prism's mmap onto the tape. It is to say what Watt and
 * the Python port already say: **program text is not an effect.** Watt stores the
 * assembly alongside the tape and recompiles from it; CPython keeps imports off
 * the tape entirely. So the loader runs paused, and a replayed program reads its
 * own source from disk exactly as the recorded one did.
 *
 * What this does *not* cover, deliberately: `DATA` (the handle `__END__` leaves
 * behind, ruby.c) is a file the *program* reads, not text the loader consumed, so
 * it stays on the tape.
 */
void rb_tape_pause(void)   { tape.paused++; }
void rb_tape_unpause(void) { if (tape.paused > 0) tape.paused--; }

/*
 * Whether this process was started with a tape flag -- answered *before*
 * ruby_init(), which is earlier than anything else here can be answered.
 *
 * The hash salt is the reason. It is drawn in Init_RandomSeedCore (random.c),
 * which the comment there describes as running "at very early stage of Ruby
 * startup" -- inside ruby_init(), long before the command line is parsed and the
 * tape is armed. So the salt never reached the tape, and `"x".hash` came out
 * different on every run: record and replay disagreed on the hash of every String
 * and every Symbol.
 *
 * It cannot be fixed by recording the salt and restoring it later. Every st_table
 * built during startup was hashed with the salt that was live at the time, and
 * re-seeding afterwards would leave all of them unsearchable. The salt has to be
 * *pinned before startup* rather than recorded during it -- and the only thing that
 * requires is knowing, that early, that this is a taped run. Hence a peek at argv,
 * which is the whole of what this does.
 */
static int tape_flag_seen;

void
rb_tape_scan_argv(int argc, char **argv)
{
    static const char *const flags[] = {
        "--tape-record", "--tape-replay", "--tape-view", "--tape-inspect",
    };
    for (int i = 1; i < argc; i++) {
        for (size_t f = 0; f < sizeof(flags) / sizeof(flags[0]); f++) {
            if (strncmp(argv[i], flags[f], strlen(flags[f])) == 0) {
                tape_flag_seen = 1;
                return;
            }
        }
    }
}

int rb_tape_pinned_seed_p(void) { return tape_flag_seen; }

/* Caller holds tape_lock. rb_warn is not an option here: it allocates a Ruby
 * String, and we may have no GVL. */
static void
tape_discard(void)
{
    tape.dropped = 1;
    tape.n_entries = 0;
    fprintf(stderr, "[tape] exceeded %d bytes; recording dropped\n", TAPE_CEILING_BYTES);
}

/**
 * Start an entry, or NULL if the tape has been dropped.
 *
 * The entry is built *off to the side*, on its own allocation, and only joins the
 * tape at entry_commit. It used to be built in place, at `&tape.entries[n_entries]`
 * -- with n_entries not yet incremented, so the slot was not reserved. Any second
 * chokepoint entering between begin and commit (another thread, since we run with
 * the GVL released) would hand out the *same* slot and clobber it, or grow the
 * array and leave the first caller writing through a pointer that realloc had
 * already freed. The corruption surfaced much later, as a SEGV inside the
 * serializer, on a run that had otherwise passed all its tests.
 */
static tape_entry *
entry_begin(int func_index)
{
    if (!rb_tape_recording()) return NULL;
    tape_entry *e = tape_calloc(1, sizeof(tape_entry));
    e->func_index = func_index;
    e->action = RB_TAPE_ACTION_RESUME;
    return e;
}

/** Capture one gather/scatter buffer. Charged against the ceiling at commit. */
static void
entry_iov(tape_entry *e, uint8_t arg_index, const void *p, size_t n)
{
    if (!e) return;
    e->iovs = tape_realloc(e->iovs, (e->n_iovs + 1) * sizeof(tape_iov));
    tape_iov *iov = &e->iovs[e->n_iovs++];
    iov->arg_index = arg_index;
    memset(&iov->bytes, 0, sizeof(iov->bytes));
    buf_push(&iov->bytes, p, n);
    e->bytes += n;
}

/**
 * Append the entry to the tape. This is the only place the shared tape is
 * mutated while recording, so it is the only place that has to be serialized --
 * the buffer copies in entry_iov, which are the expensive part, happen outside
 * the lock on the caller's own allocation.
 *
 * Takes ownership of `e`: its buffers move into the array, and the shell is
 * freed. `e` is dead on return.
 */
static void
entry_commit(tape_entry *e)
{
    if (!e) return;
    rb_nativethread_lock_lock(&tape_lock);

    /* Re-check under the lock: another thread may have overrun the ceiling while
     * this entry was being built. */
    if (!tape.dropped) {
        tape.bytes += e->bytes + e->args.len + e->ret.len;
        if (tape.bytes > TAPE_CEILING_BYTES) {
            tape_discard();
        }
        else {
            if (tape.n_entries == tape.cap_entries) {
                size_t cap = tape.cap_entries ? tape.cap_entries * 2 : 256;
                tape.entries = tape_realloc(tape.entries, cap * sizeof(tape_entry));
                tape.cap_entries = cap;
            }
            tape.entries[tape.n_entries++] = *e;  /* buffers move; no deep copy */
            e->args.ptr = e->ret.ptr = NULL;
            e->iovs = NULL;
            e->n_iovs = 0;                        /* the array owns them now */
        }
    }

    rb_nativethread_lock_unlock(&tape_lock);

    /* Whatever the outcome, the shell is ours to free -- and if the tape was
     * dropped, so are the buffers it still owns. */
    buf_free(&e->args);
    buf_free(&e->ret);
    for (size_t i = 0; i < e->n_iovs; i++) buf_free(&e->iovs[i].bytes);
    free(e->iovs);
    free(e);
}

/* Little-endian scalar helpers for args/return payloads. These mirror the raw
 * memcpy Watt does out of its kernel call buffer. */
static void
put_u32(tape_buf *b, uint32_t v)
{
    uint8_t le[4] = { v & 0xff, (v >> 8) & 0xff, (v >> 16) & 0xff, (v >> 24) & 0xff };
    buf_push(b, le, 4);
}

static void
put_i64(tape_buf *b, int64_t v)
{
    uint8_t le[8];
    for (int i = 0; i < 8; i++) le[i] = ((uint64_t)v >> (i * 8)) & 0xff;
    buf_push(b, le, 8);
}

static uint32_t
get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int64_t
get_i64(const uint8_t *p)
{
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint64_t)p[i] << (i * 8);
    return (int64_t)v;
}

/* ── Replayer ─────────────────────────────────────────────────────────────── */

/**
 * Report a divergence and stop.
 *
 * Deliberately not rb_fatal/rb_raise: an effect can be replayed during VM
 * teardown (a buffered stdout flush drains there), and teardown explicitly
 * ignores exceptions -- so a raise is swallowed and the run reports success. A
 * divergence is not a Ruby-level error anyway; it means the replay is invalid,
 * and there is nothing left to run. So say why, and leave.
 */
NORETURN(static void tape_diverged(const char *fmt, ...));
static void
tape_diverged(const char *fmt, ...)
{
    va_list args;
    fputs("[tape] divergence: ", stderr);
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fputc('\n', stderr);
    fflush(stderr);
    _exit(EXIT_FAILURE);
}

/**
 * The next recorded entry. A `func_index` mismatch means the program diverged
 * from what was taped -- there is nothing sound to do but stop, so we abort
 * loudly rather than serve the wrong bytes. This assertion *is* the divergence
 * check, exactly as in Watt.
 */
static const tape_entry *
tape_next(int func_index)
{
    /* Replay is reached from GVL-released regions too, so the cursor is shared
     * mutable state under exactly the same conditions as the recorder's array.
     * The entries themselves are immutable once decoded, so only the bump needs
     * guarding -- and tape_diverged never returns, so the lock dies with us. */
    rb_nativethread_lock_lock(&tape_lock);

    if (tape.cursor >= tape.n_entries) {
        tape_diverged("ran off the end of the tape at entry %zu; expected no more effects, got %s",
                      tape.cursor, tape_effect_fqn[func_index]);
    }
    const tape_entry *e = &tape.entries[tape.cursor];
    if (e->func_index != func_index) {
        tape_diverged("entry %zu: recorded %s, but the program called %s",
                      tape.cursor, tape_effect_fqn[e->func_index], tape_effect_fqn[func_index]);
    }
    tape.cursor++;

    rb_nativethread_lock_unlock(&tape_lock);
    return e;
}

/** The bytes captured for `arg_index`, or NULL if that arg had no iov. */
static const tape_iov *
entry_find_iov(const tape_entry *e, uint8_t arg_index)
{
    for (size_t i = 0; i < e->n_iovs; i++) {
        if (e->iovs[i].arg_index == arg_index) return &e->iovs[i];
    }
    return NULL;
}

/* ── Effect: clocks ───────────────────────────────────────────────────────── */

void
rb_tape_record_clock(int effect, int clock_id, const struct timespec *ts)
{
    tape_entry *e = entry_begin(effect);
    if (!e) return;
    put_u32(&e->args, (uint32_t)clock_id);
    put_i64(&e->ret, (int64_t)ts->tv_sec);
    put_i64(&e->ret, (int64_t)ts->tv_nsec);
    entry_commit(e);
}

void
rb_tape_replay_clock(int effect, int clock_id, struct timespec *ts)
{
    const tape_entry *e = tape_next(effect);

    /* Every clock but CLOCK_REALTIME shares one func_index, so the id is what
     * separates a monotonic read from a CPU-time read. A program that asks for a
     * different clock than the one recorded has diverged, and serving it the
     * wrong clock's value would be exactly the silent wrongness we exist to
     * prevent. */
    if (e->args.len >= 4) {
        int recorded = (int)get_u32(e->args.ptr);
        if (recorded != clock_id) {
            tape_diverged("entry %zu: recorded a read of clock %d, but the program read clock %d",
                          tape.cursor - 1, recorded, clock_id);
        }
    }

    ts->tv_sec = (time_t)get_i64(e->ret.ptr);
    ts->tv_nsec = (long)get_i64(e->ret.ptr + 8);
}

/* ── Effect: entropy ──────────────────────────────────────────────────────── */

void
rb_tape_record_random(const void *buf, size_t len, int ret)
{
    tape_entry *e = entry_begin(RB_TAPE_RANDOM_BYTES);
    if (!e) return;
    put_u32(&e->args, (uint32_t)len);
    /* The filled buffer is a scatter arg: captured post-call, like Watt's. */
    if (ret == 0) entry_iov(e, 0, buf, len);
    put_i64(&e->ret, ret);
    entry_commit(e);
}

int
rb_tape_replay_random(void *buf, size_t len)
{
    const tape_entry *e = tape_next(RB_TAPE_RANDOM_BYTES);
    const tape_iov *iov = entry_find_iov(e, 0);
    if (iov) {
        size_t n = iov->bytes.len < len ? iov->bytes.len : len;
        memcpy(buf, iov->bytes.ptr, n);
    }
    return (int)get_i64(e->ret.ptr);
}

/* ── Effect: IO ───────────────────────────────────────────────────────────── */

/*
 * A read or a write is only half-described by what it returned. The other half
 * is `errno`, and on a failure it is the *whole* story: -1/EAGAIN and -1/EPIPE
 * send the caller down entirely different paths.
 *
 * These two effects used to record the return value alone. So a recorded
 * -1/EAGAIN -- an ordinary nonblocking read that found nothing -- came back on
 * replay as -1 with whatever errno happened to be lying around, which is usually
 * zero, and CRuby's own sanity check fired:
 *
 *     [BUG] rb_sys_fail_path_in(io_read_nonblock, ) - errno == 0
 *
 * That single omission aborted a quarter of the test suite. fs.open and friends
 * already did this properly (put_result); io just never used it.
 *
 * Only meaningful on failure: errno after a *successful* syscall is stale, and
 * restoring stale garbage to a replayed process helps nobody. So record it when
 * ret < 0 and leave errno alone otherwise -- which is exactly the contract every
 * caller of read(2) already codes against.
 */
static void
put_io_result(tape_entry *e, ssize_t ret, int err)
{
    put_i64(&e->ret, (int64_t)ret);
    put_i64(&e->ret, (int64_t)(ret < 0 ? err : 0));
}

static ssize_t
take_io_result(const tape_entry *e)
{
    ssize_t ret = (ssize_t)get_i64(e->ret.ptr);
    if (ret < 0 && e->ret.len >= 16) errno = (int)get_i64(e->ret.ptr + 8);
    return ret;
}

/*
 * The fd and the buffer size the program presents have to match what was
 * recorded. They are on the tape as args precisely so they can be checked, and
 * they were not being checked.
 *
 * The consequence was not a missed divergence -- it was a *crash instead of* a
 * divergence. A program that had already gone off the rails would ask for a
 * 1024-byte read, be handed back the recorded return of a 16598-byte one, and
 * CRuby would die inside rb_str_set_len with
 *
 *     [BUG] probable buffer overflow: 16598 for 1024
 *
 * which says nothing about tapes at all. Crashing is not diverging. Say what
 * actually happened, at the entry where it happened.
 */
static void
check_io_args_diverged(const tape_entry *e, const char *what, int fd, size_t capa)
{
    if (e->args.len < 8) return;
    int rec_fd = (int)get_u32(e->args.ptr);
    size_t rec_capa = (size_t)get_u32(e->args.ptr + 4);

    if (rec_fd != fd) {
        tape_diverged("entry %zu: recorded a %s on fd %d, but the program used fd %d",
                      tape.cursor - 1, what, rec_fd, fd);
    }
    if (rec_capa != capa) {
        tape_diverged("entry %zu: recorded a %s of %zu bytes on fd %d, "
                      "but the program asked for %zu",
                      tape.cursor - 1, what, rec_capa, fd, capa);
    }
}

void
rb_tape_record_read(int fd, const void *buf, size_t capa, ssize_t ret, int err)
{
    tape_entry *e = entry_begin(RB_TAPE_IO_READ);
    if (!e) return;
    put_u32(&e->args, (uint32_t)fd);
    put_u32(&e->args, (uint32_t)capa);
    /* Scatter: capture only the filled prefix, as Watt does. */
    if (ret > 0) entry_iov(e, 1, buf, (size_t)ret);
    put_io_result(e, ret, err);
    entry_commit(e);
}

ssize_t
rb_tape_replay_read(int fd, void *buf, size_t capa)
{
    const tape_entry *e = tape_next(RB_TAPE_IO_READ);
    check_io_args_diverged(e, "read", fd, capa);
    ssize_t ret = take_io_result(e);
    const tape_iov *iov = entry_find_iov(e, 1);
    if (iov && ret > 0) {
        size_t n = iov->bytes.len < capa ? iov->bytes.len : capa;
        memcpy(buf, iov->bytes.ptr, n);
    }
    return ret;
}

/*
 * Capture the bytes the kernel *accepted* (`ret`), not the whole buffer we
 * presented. A short write retries with the remainder, so capturing the full
 * buffer each time would double-count the overlap; capturing the accepted
 * prefix means concatenating every io.write iov reconstructs the byte stream
 * that actually reached the fd, exactly.
 */
void
rb_tape_record_write(int fd, const void *buf, size_t capa, ssize_t ret, int err)
{
    tape_entry *e = entry_begin(RB_TAPE_IO_WRITE);
    if (!e) return;
    put_u32(&e->args, (uint32_t)fd);
    put_u32(&e->args, (uint32_t)capa);
    if (ret > 0) entry_iov(e, 1, buf, (size_t)ret);
    put_io_result(e, ret, err);
    entry_commit(e);
}

/*
 * Replay never re-performs the effect -- the bytes go nowhere -- but a write is
 * the one effect where the program *hands us* its output, so we can check it.
 * Comparing the presented bytes against what was recorded turns replay into a
 * determinism assertion: it catches a program that reaches the same effects in
 * the same order but computes different output. Watt only checks func_index; we
 * get this for free, and it is the check that makes "record before a refactor,
 * replay after" actually mean something.
 */
/**
 * A window of `bytes` around `at`, for the divergence message. Showing the head
 * of the buffer is useless -- the two agree up to `at` by definition, so the
 * head is always identical and tells the reader nothing. Show where they part.
 */
static void
excerpt(tape_buf *out, const uint8_t *bytes, size_t len, size_t at)
{
    const size_t before = 24, after = 36;
    size_t from = at > before ? at - before : 0;
    while (from > 0 && utf8_cont_p(bytes[from])) from--;  /* keep UTF-8 intact */

    size_t to = at + after < len ? at + after : len;
    while (to < len && utf8_cont_p(bytes[to])) to++;

    if (from > 0) sb_printf(out, "…");
    for (size_t i = from; i < to; i++) {
        /* Mark the byte the two disagree on, so the eye lands on it. */
        if (i == at) sb_printf(out, "»");
        switch (bytes[i]) {
          case '\n': sb_printf(out, "\\n"); break;
          case '\t': sb_printf(out, "\\t"); break;
          case '\r': sb_printf(out, "\\r"); break;
          default:
            if (bytes[i] >= 0x80 || (bytes[i] >= 0x20 && bytes[i] < 0x7f)) buf_byte(out, bytes[i]);
            else sb_printf(out, "\\x%02x", bytes[i]);
        }
    }
    if (to < len) sb_printf(out, "…");
    buf_byte(out, '\0');
}

static void
check_write_diverged(const tape_entry *e, const void *buf, size_t capa)
{
    const tape_iov *iov = entry_find_iov(e, 1);
    if (!iov) return;

    size_t n = iov->bytes.len;
    if (n > capa) n = capa;
    if (memcmp(buf, iov->bytes.ptr, n) == 0) return;

    size_t at = 0;
    while (at < n && ((const uint8_t *)buf)[at] == iov->bytes.ptr[at]) at++;

    tape_buf was = { 0 }, now = { 0 };
    excerpt(&was, iov->bytes.ptr, iov->bytes.len, at);
    excerpt(&now, buf, capa, at);

    tape_diverged("entry %zu writes different bytes than were recorded.\n"
                  "  first difference at byte %zu\n"
                  "  recorded:  %s\n"
                  "  replayed:  %s",
                  tape.cursor - 1, at, (const char *)was.ptr, (const char *)now.ptr);
}

ssize_t
rb_tape_replay_write(int fd, const void *buf, size_t capa)
{
    const tape_entry *e = tape_next(RB_TAPE_IO_WRITE);
    check_write_diverged(e, buf, capa);
    return take_io_result(e);
}

#ifdef HAVE_WRITEV
void
rb_tape_record_writev(int fd, const struct iovec *iov, int iovcnt, ssize_t ret, int err)
{
    tape_entry *e = entry_begin(RB_TAPE_IO_WRITE);
    if (!e) return;

    size_t total = 0;
    for (int i = 0; i < iovcnt; i++) total += iov[i].iov_len;

    put_u32(&e->args, (uint32_t)fd);
    put_u32(&e->args, (uint32_t)total);

    /* One gather capture holding every iovec concatenated -- Watt's layout --
     * truncated to the bytes the kernel accepted, as in rb_tape_record_write. */
    size_t accepted = ret > 0 ? (size_t)ret : 0;
    if (accepted) {
        if (tape.bytes + accepted > TAPE_CEILING_BYTES) { tape_discard(); return; }
        e->iovs = tape_realloc(e->iovs, (e->n_iovs + 1) * sizeof(tape_iov));
        tape_iov *cap = &e->iovs[e->n_iovs++];
        cap->arg_index = 1;
        memset(&cap->bytes, 0, sizeof(cap->bytes));
        for (int i = 0; i < iovcnt && cap->bytes.len < accepted; i++) {
            size_t take = accepted - cap->bytes.len;
            if (take > iov[i].iov_len) take = iov[i].iov_len;
            buf_push(&cap->bytes, iov[i].iov_base, take);
        }
        tape.bytes += accepted;
    }

    put_io_result(e, ret, err);
    entry_commit(e);
}

ssize_t
rb_tape_replay_writev(int fd, const struct iovec *iov, int iovcnt)
{
    const tape_entry *e = tape_next(RB_TAPE_IO_WRITE);

    /* Flatten the presented iovecs so they can be compared against the recorded
     * gather, which was stored concatenated. */
    tape_buf flat = { 0 };
    for (int i = 0; i < iovcnt; i++) buf_push(&flat, iov[i].iov_base, iov[i].iov_len);
    check_write_diverged(e, flat.ptr, flat.len);
    buf_free(&flat);

    return take_io_result(e);
}
#endif

/* ── Effect: filesystem ───────────────────────────────────────────────────────
 *
 * The fd lifecycle and every stat go on the tape, so replay never touches the
 * filesystem: a recorded run can be replayed on a machine that has none of the
 * original files. `open` returns the recorded fd without opening anything, and
 * every subsequent call on it is served from the tape.
 *
 * `struct stat` is memcpy'd whole. It is platform-shaped, so a tape does not
 * travel between platforms -- which is already true of a tape's raw payloads.
 */

/** Stash `(ret, errno)` as the entry's return value; both matter to the caller. */
static void
put_result(tape_entry *e, int ret, int err)
{
    put_i64(&e->ret, (int64_t)ret);
    put_i64(&e->ret, (int64_t)err);
}

/** Restore `(ret, errno)` from an entry. Replaying errno is what lets a recorded
 *  `Errno::ENOENT` be raised again identically. */
static int
take_result(const tape_entry *e)
{
    errno = (int)get_i64(e->ret.ptr + 8);
    return (int)get_i64(e->ret.ptr);
}

/**
 * A path is a gather arg, so replay can check it -- the same trick as
 * check_write_diverged. A program that opens a *different* file than the one
 * recorded has diverged, and we would rather say so than serve it the wrong
 * bytes.
 */
static void
check_path_diverged(const tape_entry *e, const char *path)
{
    const tape_iov *iov = entry_find_iov(e, 0);
    if (!iov) return;

    size_t len = strlen(path);
    if (len == iov->bytes.len && memcmp(path, iov->bytes.ptr, len) == 0) return;

    tape_diverged("entry %zu touches a different path than was recorded.\n"
                  "  recorded:  %.*s\n"
                  "  replayed:  %s",
                  tape.cursor - 1, (int)iov->bytes.len, (const char *)iov->bytes.ptr, path);
}

void
rb_tape_record_open(const char *path, int flags, int fd, int err)
{
    tape_entry *e = entry_begin(RB_TAPE_FS_OPEN);
    if (!e) return;
    put_u32(&e->args, (uint32_t)flags);
    entry_iov(e, 0, path, strlen(path));
    put_result(e, fd, err);
    entry_commit(e);
}

int
rb_tape_replay_open(const char *path)
{
    const tape_entry *e = tape_next(RB_TAPE_FS_OPEN);
    check_path_diverged(e, path);
    return take_result(e);
}

int
rb_tape_close(int fd)
{
    if (rb_tape_replaying()) {
        return take_result(tape_next(RB_TAPE_FS_CLOSE));
    }

    int ret = close(fd);
    int err = errno;

    if (rb_tape_recording()) {
        tape_entry *e = entry_begin(RB_TAPE_FS_CLOSE);
        if (e) {
            put_u32(&e->args, (uint32_t)fd);
            put_result(e, ret, err);
            entry_commit(e);
        }
    }
    errno = err;
    return ret;
}

/** The three stat flavors differ only in how the file is named. */
static int
tape_stat(int effect, const char *path, int fd, struct stat *st,
          int (*call)(const char *, int, struct stat *))
{
    if (rb_tape_replaying()) {
        const tape_entry *e = tape_next(effect);
        if (path) check_path_diverged(e, path);
        const tape_iov *iov = entry_find_iov(e, 1);
        if (iov && iov->bytes.len == sizeof(*st)) memcpy(st, iov->bytes.ptr, sizeof(*st));
        return take_result(e);
    }

    int ret = call(path, fd, st);
    int err = errno;

    if (rb_tape_recording()) {
        tape_entry *e = entry_begin(effect);
        if (e) {
            if (path) entry_iov(e, 0, path, strlen(path));
            else put_u32(&e->args, (uint32_t)fd);
            /* The filled struct is a scatter arg: captured post-call. */
            if (ret == 0) entry_iov(e, 1, st, sizeof(*st));
            put_result(e, ret, err);
            entry_commit(e);
        }
    }
    errno = err;
    return ret;
}

static int call_stat(const char *path, int fd, struct stat *st)  { (void)fd; return stat(path, st); }
static int call_lstat(const char *path, int fd, struct stat *st) { (void)fd; return lstat(path, st); }
static int call_fstat(const char *path, int fd, struct stat *st) { (void)path; return fstat(fd, st); }

int
rb_tape_isatty(int fd)
{
    if (rb_tape_replaying()) {
        return take_result(tape_next(RB_TAPE_FS_ISATTY));
    }

    int ret = isatty(fd);
    int err = errno;

    if (rb_tape_recording()) {
        tape_entry *e = entry_begin(RB_TAPE_FS_ISATTY);
        if (e) {
            put_u32(&e->args, (uint32_t)fd);
            put_result(e, ret, err);
            entry_commit(e);
        }
    }
    errno = err;
    return ret;
}

int rb_tape_stat(const char *path, struct stat *st)  { return tape_stat(RB_TAPE_FS_STAT,  path, -1, st, call_stat); }
int rb_tape_lstat(const char *path, struct stat *st) { return tape_stat(RB_TAPE_FS_LSTAT, path, -1, st, call_lstat); }
int rb_tape_fstat(int fd, struct stat *st)           { return tape_stat(RB_TAPE_FS_FSTAT, NULL, fd, st, call_fstat); }

/* ── Effect: filesystem mutation ──────────────────────────────────────────────
 *
 * These were the last effects still running for real during replay, and they made
 * "replay is hermetic" false in the quietest way available: a replayed run
 * reported `no divergence` while creating a directory on the real disk. It
 * half-executed -- the mkdir ran, the write inside it was suppressed.
 *
 * The leak was not the worst of it. A program that *cleans up after itself* could
 * not be replayed at all: the recording run deleted its temp file, so the replay
 * run's unlink hit ENOENT and raised. Writing a temp file and removing it is what
 * most of a test suite does.
 *
 * On replay these touch nothing and serve the recorded result, exactly as read,
 * write and open already do. The filesystem is entirely virtual: the mkdir is
 * suppressed, and the stat that observes it afterwards comes off the tape saying
 * the directory is there.
 */
static const char *const tape_fs_op_name[] = {
    "mkdir", "rmdir", "unlink", "rename", "chmod", "fchmod", "chown",
    "lchown", "symlink", "link", "truncate", "ftruncate", "utimes",
};

static int
fs_mutate_replay(int op, const char *a, const char *b)
{
    const tape_entry *e = tape_next(RB_TAPE_FS_MUTATE);

    if (e->args.len >= 4) {
        int rec_op = (int)get_u32(e->args.ptr);
        if (rec_op != op) {
            tape_diverged("entry %zu: recorded fs.%s, but the program called fs.%s",
                          tape.cursor - 1,
                          rec_op < RB_TAPE_FS_OP_MAX ? tape_fs_op_name[rec_op] : "?",
                          op < RB_TAPE_FS_OP_MAX ? tape_fs_op_name[op] : "?");
        }
    }
    /* The paths are gather args, so replay checks them -- a program that unlinks a
     * different file than it recorded has diverged. */
    if (a) check_path_diverged(e, a);
    (void)b;

    return take_result(e);
}

static void
fs_mutate_record(int op, const char *a, const char *b, int ret, int err)
{
    if (!rb_tape_recording()) return;
    tape_entry *e = entry_begin(RB_TAPE_FS_MUTATE);
    if (!e) return;
    put_u32(&e->args, (uint32_t)op);
    if (a) entry_iov(e, 0, a, strlen(a));
    if (b) entry_iov(e, 1, b, strlen(b));
    put_result(e, ret, err);
    entry_commit(e);
}

/* One-path mutations. Each is a drop-in for its syscall: same signature, same
 * semantics, errno included -- so a call site changes by one identifier. */
#define TAPE_FS_1(fn, OP, call)                          \
    {                                                    \
        if (rb_tape_replaying()) {                       \
            return fs_mutate_replay(OP, path, NULL);     \
        }                                                \
        int ret = (call);                                \
        int err = errno;                                 \
        fs_mutate_record(OP, path, NULL, ret, err);      \
        errno = err;                                     \
        return ret;                                      \
    }

int rb_tape_mkdir(const char *path, mode_t mode) TAPE_FS_1(mkdir, RB_TAPE_FS_OP_MKDIR, mkdir(path, mode))
int rb_tape_rmdir(const char *path)              TAPE_FS_1(rmdir, RB_TAPE_FS_OP_RMDIR, rmdir(path))
int rb_tape_unlink(const char *path)             TAPE_FS_1(unlink, RB_TAPE_FS_OP_UNLINK, unlink(path))
int rb_tape_chmod(const char *path, mode_t mode) TAPE_FS_1(chmod, RB_TAPE_FS_OP_CHMOD, chmod(path, mode))
int rb_tape_truncate(const char *path, off_t len) TAPE_FS_1(truncate, RB_TAPE_FS_OP_TRUNCATE, truncate(path, len))

int
rb_tape_chown(const char *path, uid_t owner, gid_t group)
    TAPE_FS_1(chown, RB_TAPE_FS_OP_CHOWN, chown(path, owner, group))

int
rb_tape_lchown(const char *path, uid_t owner, gid_t group)
    TAPE_FS_1(lchown, RB_TAPE_FS_OP_LCHOWN, lchown(path, owner, group))

int
rb_tape_utimes(const char *path, const struct timeval *times)
    TAPE_FS_1(utimes, RB_TAPE_FS_OP_UTIMES, utimes(path, times))

#undef TAPE_FS_1

/* Two-path mutations. */
#define TAPE_FS_2(OP, call)                              \
    {                                                    \
        if (rb_tape_replaying()) {                       \
            return fs_mutate_replay(OP, from, to);       \
        }                                                \
        int ret = (call);                                \
        int err = errno;                                 \
        fs_mutate_record(OP, from, to, ret, err);        \
        errno = err;                                     \
        return ret;                                      \
    }

int rb_tape_rename(const char *from, const char *to)  TAPE_FS_2(RB_TAPE_FS_OP_RENAME, rename(from, to))
int rb_tape_link(const char *from, const char *to)    TAPE_FS_2(RB_TAPE_FS_OP_LINK, link(from, to))
int rb_tape_symlink(const char *from, const char *to) TAPE_FS_2(RB_TAPE_FS_OP_SYMLINK, symlink(from, to))

#undef TAPE_FS_2

/* fd-based mutations. The fd is a *replayed* fd, so these must not reach the host
 * either -- there is nothing real behind it. */
int
rb_tape_fchmod(int fd, mode_t mode)
{
    if (rb_tape_replaying()) return fs_mutate_replay(RB_TAPE_FS_OP_FCHMOD, NULL, NULL);
    int ret = fchmod(fd, mode);
    int err = errno;
    fs_mutate_record(RB_TAPE_FS_OP_FCHMOD, NULL, NULL, ret, err);
    errno = err;
    return ret;
}

int
rb_tape_ftruncate(int fd, off_t len)
{
    if (rb_tape_replaying()) return fs_mutate_replay(RB_TAPE_FS_OP_FTRUNCATE, NULL, NULL);
    int ret = ftruncate(fd, len);
    int err = errno;
    fs_mutate_record(RB_TAPE_FS_OP_FTRUNCATE, NULL, NULL, ret, err);
    errno = err;
    return ret;
}

/*
 * The file position.
 *
 * This one hides. `File.read` does not just read -- it sizes its buffer first,
 * from `st_size - lseek(fd, 0, SEEK_CUR)` (remain_size, io.c). With the fstat on
 * the tape and the lseek off it, replay asked the *fake* fd where it was, got -1
 * back, and fell through to a 1024-byte default. The read that followed no longer
 * matched the read on the tape, and every File.read in the language was affected.
 *
 * The position is also observable directly, through IO#pos and IO#seek.
 */
off_t
rb_tape_lseek(int fd, off_t offset, int whence)
{
    if (rb_tape_replaying()) {
        const tape_entry *e = tape_next(RB_TAPE_FS_LSEEK);
        if (e->args.len >= 4) {
            int rec_fd = (int)get_u32(e->args.ptr);
            if (rec_fd != fd) {
                tape_diverged("entry %zu: recorded a seek on fd %d, but the program seeked fd %d",
                              tape.cursor - 1, rec_fd, fd);
            }
        }
        off_t ret = (off_t)get_i64(e->ret.ptr);
        if (ret < 0 && e->ret.len >= 16) errno = (int)get_i64(e->ret.ptr + 8);
        return ret;
    }

    off_t ret = lseek(fd, offset, whence);
    int err = errno;

    if (rb_tape_recording()) {
        tape_entry *e = entry_begin(RB_TAPE_FS_LSEEK);
        if (e) {
            put_u32(&e->args, (uint32_t)fd);
            put_u32(&e->args, (uint32_t)whence);
            put_i64(&e->args, (int64_t)offset);
            put_i64(&e->ret, (int64_t)ret);
            put_i64(&e->ret, (int64_t)(ret < 0 ? err : 0));
            entry_commit(e);
        }
    }
    errno = err;
    return ret;
}

/* ── Effect: directory iteration ──────────────────────────────────────────── */

#include <dirent.h>

DIR *
rb_tape_opendir(const char *path)
{
    if (rb_tape_replaying()) {
        const tape_entry *e = tape_next(RB_TAPE_FS_OPENDIR);
        check_path_diverged(e, path);
        if (take_result(e) != 0) {
            return NULL;   /* the recorded opendir failed; errno is restored */
        }
        /* A real handle, so closedir() stays valid. Never actually read. */
        DIR *real = opendir("/");
        if (real == NULL) {
            tape_diverged("replay could not open \"/\": %s", strerror(errno));
        }
        return real;
    }

    DIR *dirp = opendir(path);
    int err = errno;

    if (rb_tape_recording()) {
        tape_entry *e = entry_begin(RB_TAPE_FS_OPENDIR);
        if (e) {
            entry_iov(e, 0, path, strlen(path));
            put_result(e, dirp == NULL ? -1 : 0, err);
            entry_commit(e);
        }
    }
    errno = err;
    return dirp;
}

struct dirent *
rb_tape_readdir(DIR *dirp)
{
    /* readdir's contract is that it returns storage it owns and the caller copies
     * d_name at once -- so a static is exactly right here. */
    static struct dirent slot;

    if (rb_tape_replaying()) {
        const tape_entry *e = tape_next(RB_TAPE_FS_READDIR);
        if (take_result(e) != 0) {
            return NULL;   /* end of directory, or a recorded error */
        }
        const tape_iov *iov = entry_find_iov(e, 0);
        memset(&slot, 0, sizeof(slot));
        if (iov) {
            size_t n = iov->bytes.len;
            if (n >= sizeof(slot.d_name)) {
                n = sizeof(slot.d_name) - 1;
            }
            memcpy(slot.d_name, iov->bytes.ptr, n);
            slot.d_name[n] = '\0';
        }
        if (e->args.len >= 4) {
            slot.d_type = (unsigned char)get_u32(e->args.ptr);
        }
        return &slot;
    }

    errno = 0;
    struct dirent *ep = readdir(dirp);
    int err = errno;

    if (rb_tape_recording()) {
        tape_entry *e = entry_begin(RB_TAPE_FS_READDIR);
        if (e) {
            put_u32(&e->args, ep ? (uint32_t)ep->d_type : 0);
            if (ep) {
                entry_iov(e, 0, ep->d_name, strlen(ep->d_name));
            }
            put_result(e, ep == NULL ? -1 : 0, err);
            entry_commit(e);
        }
    }
    errno = err;
    return ep;
}

int
rb_tape_closedir(DIR *dirp)
{
    if (rb_tape_replaying()) {
        closedir(dirp);   /* the "/" handle we handed out; the result is taped */
        return (int)take_result(tape_next(RB_TAPE_FS_CLOSEDIR));
    }

    int ret = closedir(dirp);
    int err = errno;

    if (rb_tape_recording()) {
        tape_entry *e = entry_begin(RB_TAPE_FS_CLOSEDIR);
        if (e) {
            put_result(e, ret, err);
            entry_commit(e);
        }
    }
    errno = err;
    return ret;
}

/* ── Effect: the process ──────────────────────────────────────────────────── */

long
rb_tape_getpid(void)
{
    if (rb_tape_replaying()) {
        const tape_entry *e = tape_next(RB_TAPE_PROC_GETPID);
        return (long)get_i64(e->ret.ptr);
    }

    long pid = (long)getpid();

    if (rb_tape_recording()) {
        tape_entry *e = entry_begin(RB_TAPE_PROC_GETPID);
        if (e) {
            put_i64(&e->ret, (int64_t)pid);
            entry_commit(e);
        }
    }
    return pid;
}

/* ── Effect: the environment ──────────────────────────────────────────────── */

const char *
rb_tape_getenv(const char *name)
{
    if (rb_tape_replaying()) {
        const tape_entry *e = tape_next(RB_TAPE_ENV_GET);
        check_path_diverged(e, name);   /* reading a *different* variable is a divergence */
        if (take_result(e) != 0) {
            return NULL;   /* the variable was unset when recorded */
        }
        const tape_iov *iov = entry_find_iov(e, 1);
        return iov ? (const char *)iov->bytes.ptr : NULL;
    }

    const char *val = getenv(name);

    if (rb_tape_recording()) {
        tape_entry *e = entry_begin(RB_TAPE_ENV_GET);
        if (e) {
            entry_iov(e, 0, name, strlen(name));
            if (val) {
                /* The trailing NUL goes on the tape too, so the replayed pointer
                 * is a valid C string straight out of the iov buffer. */
                entry_iov(e, 1, val, strlen(val) + 1);
            }
            put_result(e, val ? 0 : -1, 0);
            entry_commit(e);
        }
    }
    return val;
}

/* ── Canonical addresses ──────────────────────────────────────────────────── */

/* Open-addressed pointer -> stand-in map. Only objects whose address is actually
 * observed (a default to_s or inspect) ever land here, so it stays small. */
typedef struct {
    uintptr_t key;      /* 0 = empty */
    uintptr_t value;
} addr_slot;

static struct {
    addr_slot *slots;
    size_t cap;         /* always a power of two */
    size_t len;
    size_t next;        /* the next stand-in to hand out */
} addrs;

#define TAPE_ADDR_BASE 0x7f0000000000ULL   /* looks like a real heap address */
#define TAPE_ADDR_STEP 16                  /* Ruby objects are 16-byte aligned */

static void
addrs_grow(void)
{
    size_t cap = addrs.cap ? addrs.cap * 2 : 1024;
    addr_slot *slots = tape_calloc(cap, sizeof(addr_slot));
    for (size_t i = 0; i < addrs.cap; i++) {
        if (addrs.slots[i].key == 0) {
            continue;
        }
        size_t j = (size_t)(addrs.slots[i].key >> 4) & (cap - 1);
        while (slots[j].key != 0) {
            j = (j + 1) & (cap - 1);
        }
        slots[j] = addrs.slots[i];
    }
    free(addrs.slots);
    addrs.slots = slots;
    addrs.cap = cap;
}

uintptr_t
rb_tape_canonical_addr(const void *ptr)
{
    uintptr_t key = (uintptr_t)ptr;
    if (key == 0) {
        return 0;
    }
    if (addrs.len * 2 >= addrs.cap) {
        addrs_grow();
    }

    size_t i = (size_t)(key >> 4) & (addrs.cap - 1);
    while (addrs.slots[i].key != 0) {
        if (addrs.slots[i].key == key) {
            return addrs.slots[i].value;
        }
        i = (i + 1) & (addrs.cap - 1);
    }

    /* First sighting. We never evict: if Ruby reuses a freed address, the new
     * object gets the old stand-in -- which is what Ruby itself does with real
     * addresses, so the semantics match. */
    uintptr_t value = (uintptr_t)(TAPE_ADDR_BASE + addrs.next * TAPE_ADDR_STEP);
    addrs.next++;
    addrs.slots[i].key = key;
    addrs.slots[i].value = value;
    addrs.len++;
    return value;
}

/* ── Encode ───────────────────────────────────────────────────────────────── */

static int64_t
now_millis(void)
{
    struct timespec ts;
    /* The recorder's own clock read must not land on the tape. */
    clock_gettime(CLOCK_REALTIME, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/** postcard(SpooledTape). See the layout comment at the top of this file. */
static void
encode_spooled_tape(tape_buf *out, const char *entry_fqn)
{
    pc_str(out, "ruby");            /* assembly_hash -- see RUBY.md §5 */
    pc_str(out, entry_fqn);         /* entry_fqn */
    pc_svarint(out, now_millis());  /* recorded_at */
    buf_byte(out, 0);               /* session_id: None */
    pc_str(out, ruby_version);      /* build_id */

    pc_uvarint(out, tape.n_entries);
    for (size_t i = 0; i < tape.n_entries; i++) {
        const tape_entry *e = &tape.entries[i];
        pc_svarint(out, e->func_index);
        buf_byte(out, e->action);
        pc_bytes(out, e->args.ptr, e->args.len);
        pc_uvarint(out, e->n_iovs);
        for (size_t j = 0; j < e->n_iovs; j++) {
            buf_byte(out, e->iovs[j].arg_index);
            pc_bytes(out, e->iovs[j].bytes.ptr, e->iovs[j].bytes.len);
        }
        pc_bytes(out, e->ret.ptr, e->ret.len);
    }
}

/**
 * Write the tape: a version byte, then a zstd frame. Written to a temp path and
 * renamed, so a reader never sees a partial file.
 */
static void
tape_write_file(void)
{
    tape_buf body = { 0 };
    encode_spooled_tape(&body, "main");

    size_t bound = ZSTD_compressBound(body.len);
    uint8_t *z = tape_malloc(bound);
    size_t zlen = ZSTD_compress(z, bound, body.ptr, body.len, TAPE_ZSTD_LEVEL);
    if (ZSTD_isError(zlen)) {
        rb_warn("tape: zstd failed: %s", ZSTD_getErrorName(zlen));
        goto done;
    }

    char tmp[PATH_MAX];
    snprintf(tmp, sizeof(tmp), "%s.tmp", tape.path);

    FILE *f = fopen(tmp, "wb");
    if (!f) {
        rb_warn("tape: cannot open %s: %s", tmp, strerror(errno));
        goto done;
    }
    uint8_t version = TAPE_FORMAT_VERSION;
    int ok = fwrite(&version, 1, 1, f) == 1 && fwrite(z, 1, zlen, f) == zlen;
    ok = (fclose(f) == 0) && ok;

    if (!ok || rename(tmp, tape.path) != 0) {
        rb_warn("tape: cannot write %s: %s", tape.path, strerror(errno));
        unlink(tmp);
        goto done;
    }
    fprintf(stderr, "[tape] %zu entries -> %s (%zu bytes)\n", tape.n_entries, tape.path, zlen + 1);

  done:
    free(z);
    buf_free(&body);
}

/* ── Decode ───────────────────────────────────────────────────────────────── */

typedef struct {
    const uint8_t *p;
    const uint8_t *end;
} tape_reader;

static uint64_t
rd_uvarint(tape_reader *r)
{
    uint64_t v = 0;
    int shift = 0;
    while (r->p < r->end) {
        uint8_t byte = *r->p++;
        v |= (uint64_t)(byte & 0x7f) << shift;
        if (!(byte & 0x80)) return v;
        shift += 7;
    }
    rb_fatal("tape: truncated varint");
}

static int64_t
rd_svarint(tape_reader *r)
{
    uint64_t v = rd_uvarint(r);
    return (int64_t)(v >> 1) ^ -(int64_t)(v & 1);
}

static uint8_t
rd_byte(tape_reader *r)
{
    if (r->p >= r->end) rb_fatal("tape: truncated");
    return *r->p++;
}

/** varint(len) ++ raw, copied into `out`. */
static void
rd_bytes(tape_reader *r, tape_buf *out)
{
    size_t n = (size_t)rd_uvarint(r);
    if ((size_t)(r->end - r->p) < n) rb_fatal("tape: truncated payload");
    memset(out, 0, sizeof(*out));
    buf_push(out, r->p, n);
    r->p += n;
}

static void
rd_skip_str(tape_reader *r)
{
    size_t n = (size_t)rd_uvarint(r);
    if ((size_t)(r->end - r->p) < n) rb_fatal("tape: truncated string");
    r->p += n;
}

static void
tape_read_file(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) rb_fatal("tape: cannot open %s: %s", path, strerror(errno));

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size < 1) rb_fatal("tape: %s is empty", path);

    uint8_t *raw = tape_malloc((size_t)size);
    if (fread(raw, 1, (size_t)size, f) != (size_t)size) rb_fatal("tape: short read on %s", path);
    fclose(f);

    if (raw[0] != TAPE_FORMAT_VERSION) {
        rb_fatal("tape: unsupported format version %d (expected %d)", raw[0], TAPE_FORMAT_VERSION);
    }

    unsigned long long dlen = ZSTD_getFrameContentSize(raw + 1, (size_t)size - 1);
    if (dlen == ZSTD_CONTENTSIZE_ERROR || dlen == ZSTD_CONTENTSIZE_UNKNOWN) {
        rb_fatal("tape: %s is not a valid zstd frame", path);
    }
    uint8_t *body = tape_malloc((size_t)dlen);
    size_t got = ZSTD_decompress(body, (size_t)dlen, raw + 1, (size_t)size - 1);
    if (ZSTD_isError(got)) rb_fatal("tape: zstd decode failed: %s", ZSTD_getErrorName(got));
    free(raw);

    tape_reader r = { body, body + got };
    rd_skip_str(&r);        /* assembly_hash */
    rd_skip_str(&r);        /* entry_fqn */
    (void)rd_svarint(&r);   /* recorded_at */
    if (rd_byte(&r)) rd_skip_str(&r);  /* session_id: Option<String> */
    rd_skip_str(&r);        /* build_id */

    tape.n_entries = (size_t)rd_uvarint(&r);
    tape.entries = tape_calloc(tape.n_entries ? tape.n_entries : 1, sizeof(tape_entry));

    for (size_t i = 0; i < tape.n_entries; i++) {
        tape_entry *e = &tape.entries[i];
        e->func_index = (int32_t)rd_svarint(&r);
        e->action = rd_byte(&r);
        rd_bytes(&r, &e->args);
        e->n_iovs = (size_t)rd_uvarint(&r);
        e->iovs = e->n_iovs ? tape_calloc(e->n_iovs, sizeof(tape_iov)) : NULL;
        for (size_t j = 0; j < e->n_iovs; j++) {
            e->iovs[j].arg_index = rd_byte(&r);
            rd_bytes(&r, &e->iovs[j].bytes);
        }
        rd_bytes(&r, &e->ret);
    }
    free(body);
}

/* ── Inspect ──────────────────────────────────────────────────────────────── */

/* Longest payload rendered into a cell before it is elided. A tape can hold a
 * multi-megabyte file read; the table is for reading, not for dumping. */
#define TAPE_INSPECT_MAX_CELL 48

/**
 * Display width of a cell: UTF-8 characters, not bytes. Padding the table by
 * byte count would misalign every row containing a multi-byte character, and
 * Ruby's strings are UTF-8 by default -- box-drawing output is not an edge case.
 */
static size_t
cell_width(const tape_buf *b)
{
    size_t w = 0;
    for (size_t i = 0; i < b->len; i++) {
        if (!utf8_cont_p(b->ptr[i])) w++;
    }
    return w;
}

/**
 * Render a captured payload the way Watt renders a value: text as a quoted,
 * escaped string (`"hello\n"`); binary as hex (`<a5 d4 39>`). Watt picks by
 * static type; Ruby has none here, so we pick by content.
 *
 * High bytes count as text: Ruby strings are UTF-8, so a payload full of `─`
 * is text, not binary. It is passed through raw -- the terminal renders it.
 */
static void
render_payload(tape_buf *out, const uint8_t *p, size_t len)
{
    size_t textish = 0;
    for (size_t i = 0; i < len; i++) {
        if (p[i] == '\n' || p[i] == '\t' || p[i] == '\r'
            || (p[i] >= 0x20 && p[i] < 0x7f) || p[i] >= 0x80) textish++;
    }
    int as_text = len > 0 && textish * 10 >= len * 9;  /* >=90% */

    size_t show = len < TAPE_INSPECT_MAX_CELL ? len : TAPE_INSPECT_MAX_CELL;

    if (as_text) {
        /* Never cut a multi-byte character in half. */
        while (show > 0 && show < len && utf8_cont_p(p[show])) show--;

        buf_byte(out, '"');
        for (size_t i = 0; i < show; i++) {
            switch (p[i]) {
              case '\n': sb_printf(out, "\\n"); break;
              case '\t': sb_printf(out, "\\t"); break;
              case '\r': sb_printf(out, "\\r"); break;
              case '"':  sb_printf(out, "\\\""); break;
              case '\\': sb_printf(out, "\\\\"); break;
              default:
                if (p[i] >= 0x80 || (p[i] >= 0x20 && p[i] < 0x7f)) buf_byte(out, p[i]);
                else sb_printf(out, "\\x%02x", p[i]);
            }
        }
        buf_byte(out, '"');
    }
    else {
        buf_byte(out, '<');
        for (size_t i = 0; i < show; i++) sb_printf(out, i ? " %02x" : "%02x", p[i]);
        buf_byte(out, '>');
    }
    if (show < len) sb_printf(out, " (+%zu bytes)", len - show);
}

/** Effects whose first gather arg is a filesystem path. */
static int
path_effect_p(int func_index)
{
    return func_index == RB_TAPE_FS_OPEN
        || func_index == RB_TAPE_FS_STAT
        || func_index == RB_TAPE_FS_LSTAT
        || func_index == RB_TAPE_FS_OPENDIR
        || func_index == RB_TAPE_ENV_GET;
}

/** The Args cell: the entry's scalar arguments, comma-joined, as Watt shows them. */
static void
render_args(tape_buf *out, const tape_entry *e)
{
    const uint8_t *a = e->args.ptr;
    switch (e->func_index) {
      case RB_TAPE_IO_READ:
      case RB_TAPE_IO_WRITE:
        if (e->args.len >= 8) {
            sb_printf(out, "fd %u, %u", (unsigned)get_u32(a), (unsigned)get_u32(a + 4));
        }
        break;
      case RB_TAPE_RANDOM_BYTES:
      case RB_TAPE_FS_READDIR: {
        const tape_iov *nm = entry_find_iov(e, 0);
        if (nm) render_payload(out, nm->bytes.ptr, nm->bytes.len);
        break;
      }
      case RB_TAPE_FS_CLOSE:
      case RB_TAPE_FS_FSTAT:
      case RB_TAPE_FS_ISATTY:
        if (e->args.len >= 4) sb_printf(out, "%u", (unsigned)get_u32(a));
        break;
      default:
        if (path_effect_p(e->func_index)) {
            /* The path is a gather arg, not a scalar; show it here rather than
             * burying it among the iovs in the Action column. */
            const tape_iov *p = entry_find_iov(e, 0);
            if (p) render_payload(out, p->bytes.ptr, p->bytes.len);
        }
        break;
    }
}

/** The Action cell: `resume(ret, iov[N]=...)`, `halt()`, or `abort()`. */
static void
render_action(tape_buf *out, const tape_entry *e)
{
    if (e->action == RB_TAPE_ACTION_HALT || e->action == RB_TAPE_ACTION_ABORT) {
        long long status = e->ret.len >= 8 ? (long long)get_i64(e->ret.ptr) : 0;
        sb_printf(out, "%s(%lld)", e->action == RB_TAPE_ACTION_HALT ? "halt" : "abort", status);
        return;
    }

    sb_printf(out, "resume(");
    if (e->func_index == RB_TAPE_CLOCK_REALTIME || e->func_index == RB_TAPE_CLOCK_MONOTONIC) {
        if (e->ret.len >= 16) {
            sb_printf(out, "%lld.%09lld", (long long)get_i64(e->ret.ptr),
                      (long long)get_i64(e->ret.ptr + 8));
        }
    }
    else if (e->ret.len >= 8) {
        long long ret = (long long)get_i64(e->ret.ptr);
        sb_printf(out, "%lld", ret);
        /* A failed effect is only half-told by its return value. The errno is the
         * other half -- and it is the half replay re-raises -- so a reader who
         * sees `resume(-1)` and nothing else has been told nothing at all. */
        if (ret < 0 && e->ret.len >= 16) {
            int err = (int)get_i64(e->ret.ptr + 8);
            if (err) sb_printf(out, " %s", strerror(err));
        }
    }
    for (size_t i = 0; i < e->n_iovs; i++) {
        /* iov[0] of a path effect is already the Args column; don't repeat it. */
        if (e->iovs[i].arg_index == 0 && path_effect_p(e->func_index)) continue;
        sb_printf(out, ", iov[%u]=", (unsigned)e->iovs[i].arg_index);
        render_payload(out, e->iovs[i].bytes.ptr, e->iovs[i].bytes.len);
    }
    buf_byte(out, ')');
}

#define TAPE_INSPECT_COLS 5

static void
rule(size_t width, char fill)
{
    putchar('+');
    for (size_t i = 0; i < width; i++) putchar(fill);
    puts("+");
}

/**
 * Print the tape as a table, in the shape of `watt tape inspect`: outer borders
 * only, a `+===+` rule under the header, three spaces between columns.
 */
void
rb_tape_inspect(const char *path)
{
    tape_read_file(path);

    static const char *const header[TAPE_INSPECT_COLS] = { "#", "Func", "Signature", "Args", "Action" };

    /* One cell buffer per column per row, plus the header row. */
    size_t nrows = tape.n_entries + 1;
    tape_buf *cells = tape_calloc(nrows * TAPE_INSPECT_COLS, sizeof(tape_buf));

    for (int c = 0; c < TAPE_INSPECT_COLS; c++) {
        sb_printf(&cells[c], "%s", header[c]);
    }
    for (size_t i = 0; i < tape.n_entries; i++) {
        const tape_entry *e = &tape.entries[i];
        tape_buf *row = &cells[(i + 1) * TAPE_INSPECT_COLS];
        sb_printf(&row[0], "%zu", i);
        sb_printf(&row[1], "%s", tape_effect_fqn[e->func_index]);
        sb_printf(&row[2], "%s", tape_effect_sig[e->func_index]);
        render_args(&row[3], e);
        render_action(&row[4], e);
    }

    size_t w[TAPE_INSPECT_COLS] = { 0 };
    for (size_t r = 0; r < nrows; r++) {
        for (int c = 0; c < TAPE_INSPECT_COLS; c++) {
            size_t len = cell_width(&cells[r * TAPE_INSPECT_COLS + c]);
            if (len > w[c]) w[c] = len;
        }
    }

    /* "| " + cells joined by 3 spaces + " |" */
    size_t inner = 2;
    for (int c = 0; c < TAPE_INSPECT_COLS; c++) inner += w[c] + (c ? 3 : 0);

    rule(inner, '-');
    for (size_t r = 0; r < nrows; r++) {
        fputs("| ", stdout);
        for (int c = 0; c < TAPE_INSPECT_COLS; c++) {
            if (c) fputs("   ", stdout);
            tape_buf *cell = &cells[r * TAPE_INSPECT_COLS + c];
            fwrite(cell->ptr, 1, cell->len, stdout);
            for (size_t pad = cell_width(cell); pad < w[c]; pad++) putchar(' ');
        }
        puts(" |");
        if (r == 0) rule(inner, '=');
    }
    rule(inner, '-');

    for (size_t i = 0; i < nrows * TAPE_INSPECT_COLS; i++) buf_free(&cells[i]);
    free(cells);
}

/* ── Lifecycle ────────────────────────────────────────────────────────────── */

static char *
tape_strdup(const char *s)
{
    size_t n = strlen(s) + 1;
    char *out = tape_malloc(n);
    memcpy(out, s, n);
    return out;
}

void
rb_tape_record_to(const char *path)
{
    rb_nativethread_lock_initialize(&tape_lock);
    tape.recording = 1;
    tape.path = tape_strdup(path);
}

void
rb_tape_replay_from(const char *path)
{
    rb_nativethread_lock_initialize(&tape_lock);
    tape.replaying = 1;
    tape.path = tape_strdup(path);
    tape_read_file(path);
}

void
rb_tape_finish(int exit_status)
{
    int aborted = exit_status != EXIT_SUCCESS;

    if (tape.replaying) {
        /* The terminal entry is consumed here, not by a chokepoint -- so a run
         * that reached every recorded effect ends with the cursor one short. */
        int expected = aborted ? RB_TAPE_KERNEL_ABORT : RB_TAPE_KERNEL_HALT;
        const tape_entry *e = tape_next(expected);

        int64_t recorded = e->ret.len >= 8 ? get_i64(e->ret.ptr) : 0;
        if (recorded != (int64_t)exit_status) {
            tape_diverged("recorded exit status %lld, but the program exited %d",
                          (long long)recorded, exit_status);
        }

        if (tape.cursor != tape.n_entries) {
            rb_warn("tape: replay stopped %zu entries early; program diverged",
                    tape.n_entries - tape.cursor);
        }
        else {
            fprintf(stderr, "[tape] replayed %zu entries, no divergence\n", tape.n_entries);
        }
        (void)e;
        return;
    }
    if (!tape.recording) return;
    if (tape.dropped) {
        rb_warn("tape: recording was dropped; nothing written");
        return;
    }

    /* Terminal entry carrying the exit status, so a reader can tell a clean exit
     * from a crash -- Watt's Halt/Abort. */
    tape_entry *e = entry_begin(aborted ? RB_TAPE_KERNEL_ABORT : RB_TAPE_KERNEL_HALT);
    if (e) {
        e->action = aborted ? RB_TAPE_ACTION_ABORT : RB_TAPE_ACTION_HALT;
        put_i64(&e->ret, (int64_t)exit_status);
        entry_commit(e);
    }

    tape.recording = 0;  /* stop taping our own writes */
    tape_write_file();
}
