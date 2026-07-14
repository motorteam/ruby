#ifndef RUBY_TAPE_H                                      /*-*-C-*-vi:se ft=c:*/
#define RUBY_TAPE_H
/**
 * @file
 * Deterministic record/replay of a Ruby program's observable behavior.
 *
 * A "tape" is a log of every crossing of the program->host boundary, and
 * nothing else. Replay re-runs the program but serves each effect from the
 * tape instead of calling libc, so execution is reproduced exactly.
 *
 * Effects are captured at CRuby's *internal libc-wrapper layer* (see the
 * chokepoints in tape.c), not at the Ruby-method layer: one patch at
 * internal_read_func covers every IO method in the language. The wire format
 * is Watt's (see RUBY.md).
 */
#include "ruby/internal/config.h"
#include <stddef.h>
#include <stdint.h>
#include <time.h>

/**
 * The effect table -- Ruby's equivalent of Watt's kernel externals. Each entry
 * is a chokepoint where nondeterminism enters the program. The index is the
 * `func_index` written to the tape, so **append only**: reordering invalidates
 * every existing tape.
 */
enum rb_tape_effect {
    RB_TAPE_CLOCK_REALTIME  = 0,
    RB_TAPE_CLOCK_MONOTONIC = 1,
    RB_TAPE_RANDOM_BYTES    = 2,
    RB_TAPE_IO_READ         = 3,
    RB_TAPE_IO_WRITE        = 4,
    /* Terminal effects. Like Watt's core.kernel.halt / panic_at, these get their
     * own func_index rather than riding on another effect's. */
    RB_TAPE_KERNEL_HALT     = 5,
    RB_TAPE_KERNEL_ABORT    = 6,
    /* Filesystem. Recording these is what makes replay *hermetic*: the tape
     * carries the fd lifecycle and every stat, so a replay never touches the
     * filesystem and the original files need not exist. */
    RB_TAPE_FS_OPEN         = 7,
    RB_TAPE_FS_CLOSE        = 8,
    RB_TAPE_FS_STAT         = 9,
    RB_TAPE_FS_LSTAT        = 10,
    RB_TAPE_FS_FSTAT        = 11,
    /* isatty is not a curiosity: Ruby picks stdout's buffering from it
     * (io.c:8619), so an unrecorded isatty silently ties a tape to whether it
     * was recorded under a terminal or a pipe. */
    RB_TAPE_FS_ISATTY       = 12,
    /* The file position. Not a curiosity either: `File.read` sizes its buffer
     * from `st_size - lseek(fd, 0, SEEK_CUR)` (remain_size, io.c). With the fstat
     * taped and the lseek not, replay asked the *fake* fd where it was, got -1,
     * and fell back to a 1024-byte default -- so the read no longer matched the
     * one on the tape. Every File.read was affected. */
    RB_TAPE_FS_LSEEK        = 13,
    RB_TAPE_FS_OPENDIR      = 14,
    RB_TAPE_FS_READDIR      = 15,
    RB_TAPE_FS_CLOSEDIR     = 16,
    /* Ruby reads ENV per name (getenv_with_lock, hash.c), where CPython
     * snapshots it into a dict at startup -- so this is a per-read effect rather
     * than the one-shot snapshot the Python port records. */
    RB_TAPE_ENV_GET         = 17,
    /* The pid is read once and cached, so it costs one entry -- but it leaks into
     * ordinary output (`$$`, the test runner's own banner) and into temp-file
     * names, so an unrecorded pid diverges on the *bytes* and on the *paths*. */
    RB_TAPE_PROC_GETPID     = 18,
    /* Every filesystem *mutation* -- mkdir, unlink, rename, chmod, ... They share
     * one func_index and carry the op as data, because from the tape's point of
     * view they are all the same shape: (op, one or two paths) -> (result, errno).
     * And replay does the same thing with every one of them: serve the recorded
     * result and touch nothing. See rb_tape_fs_op. */
    RB_TAPE_FS_MUTATE       = 19,
    RB_TAPE_EFFECT_MAX
};

/**
 * The filesystem mutations, as an op code on RB_TAPE_FS_MUTATE. Append only.
 *
 * These were the last effects still executing for real during replay, and they
 * made "replay is hermetic" false in the most dangerous way available: quietly.
 * A replayed run reported `no divergence` while creating a directory on the real
 * disk -- it half-executed, running the mkdir and suppressing the write inside it.
 *
 * And the leak was not even the worst of it. A program that *cleans up after
 * itself* could not be replayed at all: the recording run deleted its temp file,
 * so the replay run's unlink hit ENOENT and raised. Writing a temp file and
 * removing it is what most of a test suite does.
 */
enum rb_tape_fs_op {
    RB_TAPE_FS_OP_MKDIR     = 0,
    RB_TAPE_FS_OP_RMDIR     = 1,
    RB_TAPE_FS_OP_UNLINK    = 2,
    RB_TAPE_FS_OP_RENAME    = 3,
    RB_TAPE_FS_OP_CHMOD     = 4,
    RB_TAPE_FS_OP_FCHMOD    = 5,
    RB_TAPE_FS_OP_CHOWN     = 6,
    RB_TAPE_FS_OP_LCHOWN    = 7,
    RB_TAPE_FS_OP_SYMLINK   = 8,
    RB_TAPE_FS_OP_LINK      = 9,
    RB_TAPE_FS_OP_TRUNCATE  = 10,
    RB_TAPE_FS_OP_FTRUNCATE = 11,
    RB_TAPE_FS_OP_UTIMES    = 12,
    RB_TAPE_FS_OP_MAX
};

/** Outcome tag for a recorded entry. Mirrors Watt's `Action`. */
enum rb_tape_action {
    RB_TAPE_ACTION_HALT   = 0, /**< terminal: normal exit */
    RB_TAPE_ACTION_ABORT  = 1, /**< terminal: uncaught exception */
    RB_TAPE_ACTION_RESUME = 2  /**< the effect returned; replay serves it from the tape */
};

/** Enable recording to `path`. Called from ruby.c on --tape-record. */
void rb_tape_record_to(const char *path);

/** Enable replay from `path`. Called from ruby.c on --tape-replay. */
void rb_tape_replay_from(const char *path);

/**
 * Decode `path` and print it as a table -- the equivalent of `watt tape inspect`,
 * in the same shape. Reads the tape statically; nothing is executed.
 */
void rb_tape_inspect(const char *path);

/**
 * Seal the tape and write it out. `exit_status` becomes the terminal entry's
 * payload, so a reader sees how the program ended -- Watt's `halt(nil)`.
 */
void rb_tape_finish(int exit_status);

int rb_tape_recording(void);
int rb_tape_replaying(void);

/**
 * Suspend the tape while the loader reaches for program text.
 *
 * Loading the program is not running the program. A `require` opens twice: the
 * $LOAD_PATH probe in `rb_file_load_ok` (file.c) goes through `rb_cloexec_open`
 * and so is a chokepoint, while Prism reads the source it settles on with a raw
 * `open`+`mmap` (prism/source.c) that no chokepoint can see. Taping only the
 * probe put an open on the tape whose read and close never arrived. Rather than
 * drag Prism's mmap onto the tape, the loader runs paused -- the same rule Watt
 * (which stores the assembly beside the tape) and the CPython port (which keeps
 * imports off it) already follow.
 *
 * Nests: a require inside a required file pauses twice and unpauses twice.
 */
void rb_tape_pause(void);
void rb_tape_unpause(void);

/**
 * Declare the calling thread to be VM infrastructure, not the program. Nothing it
 * does will ever reach the tape.
 *
 * Called once by the timer thread. It wakes on a wall-clock schedule -- every few
 * milliseconds, whether or not the program did anything -- and reads the monotonic
 * clock to decide which sleeping thread is due. Those reads are not the program
 * asking for the time; they never flow back into it. Taping them sprayed entries
 * across the effect stream at points that depended on nothing but how long the
 * last syscall happened to take.
 */
void rb_tape_thread_off(void);

/**
 * Peek at argv for a tape flag, before ruby_init(). Called from main.c.
 *
 * Everything else here can wait until the tape is armed. The hash salt cannot: it
 * is drawn in Init_RandomSeedCore, inside ruby_init(), before the command line is
 * parsed -- so it never reached the tape, and `"x".hash` differed between record
 * and replay. Nor can it be recorded and restored afterwards, because every
 * st_table built during startup was hashed with the live salt and re-seeding would
 * leave them all unsearchable. It has to be pinned *before* startup, which means
 * knowing this early that a tape is coming.
 */
void rb_tape_scan_argv(int argc, char **argv);

/** True if the hash salt should be pinned rather than drawn from entropy. */
int rb_tape_pinned_seed_p(void);

/**
 * True if a tape is active at all. The chokepoints test this first so an
 * untaped run pays a single predictable branch and nothing else.
 */
#define RB_TAPE_ACTIVE() (rb_tape_recording() || rb_tape_replaying())

/* ── Per-effect record/replay pairs. ───────────────────────────────────────
 * Each chokepoint in the tree calls exactly one of these, so the patch at the
 * call site stays two or three lines and the marshalling lives here.
 */

/**
 * Clocks. `clock_id` is the platform clockid the program actually asked for; it
 * rides in args so replay can check it and so `--tape-inspect` can name it.
 *
 * A taped clock and an untaped clock in the same loop is a *divergent loop*, and
 * that is not a hypothetical: `sleep` computes its pthread_cond_timedwait
 * deadline from the realtime clock (native_cond_timeout, thread_pthread.c) and
 * then tests for completion against the monotonic one (sleep_hrtime, thread.c).
 * With realtime taped and monotonic live, replay froze the deadline in the past
 * -- so the wait returned instantly, the monotonic clock said "not yet", and the
 * loop spun, burning one tape entry per turn until it ran off the end. Both
 * clocks have to move together, and they only do that if both are on the tape.
 *
 * The payoff beyond correctness: a program that sleeps ten seconds replays
 * instantly, because time advances on the tape rather than on the wall.
 */
void rb_tape_record_clock(int effect, int clock_id, const struct timespec *ts);
void rb_tape_replay_clock(int effect, int clock_id, struct timespec *ts);

/* The clockid names for the two clocks CRuby reads without being told one.
 * `rb_timespec_now` may reach the wall clock through gettimeofday on a platform
 * with no clock_gettime, and gettimeofday has no clockid at all -- so name one,
 * and keep the ids on the tape stable whichever path the build takes. */
#ifdef CLOCK_REALTIME
# define RB_TAPE_CLOCKID_REALTIME CLOCK_REALTIME
#else
# define RB_TAPE_CLOCKID_REALTIME 0
#endif
#ifdef CLOCK_MONOTONIC
# define RB_TAPE_CLOCKID_MONOTONIC CLOCK_MONOTONIC
#else
# define RB_TAPE_CLOCKID_MONOTONIC 1
#endif

/**
 * Which effect a clockid belongs to. CLOCK_REALTIME is the wall clock; every
 * other clock -- MONOTONIC, PROCESS_CPUTIME_ID, THREAD_CPUTIME_ID, the BSD
 * variants -- is a monotonic-family read and shares one func_index, with the
 * exact id recorded in args to tell them apart.
 */
static inline int
rb_tape_clock_effect(int clock_id)
{
#ifdef CLOCK_REALTIME
    if (clock_id == CLOCK_REALTIME) return RB_TAPE_CLOCK_REALTIME;
#endif
    return RB_TAPE_CLOCK_MONOTONIC;
}

void rb_tape_record_random(const void *buf, size_t len, int ret);
int  rb_tape_replay_random(void *buf, size_t len);

/*
 * `err` is the errno the syscall left behind, and on a failure it is not a
 * detail -- it *is* the result. -1/EAGAIN (a nonblocking read that found
 * nothing) and -1/EPIPE (the peer is gone) send the caller down completely
 * different paths, and a tape that records only the -1 cannot tell them apart.
 *
 * It is recorded only when ret < 0; errno after a successful syscall is stale,
 * and handing stale garbage back to a replayed process helps nobody. Pass the
 * errno captured *immediately* after the syscall -- anything in between (a
 * retry, a wait, a GVL reacquire) will have clobbered it.
 */
void   rb_tape_record_read(int fd, const void *buf, size_t capa, ssize_t ret, int err);
ssize_t rb_tape_replay_read(int fd, void *buf, size_t capa);

void   rb_tape_record_write(int fd, const void *buf, size_t capa, ssize_t ret, int err);
ssize_t rb_tape_replay_write(int fd, const void *buf, size_t capa);

/* ── Filesystem: drop-in wrappers ─────────────────────────────────────────────
 * Each has the syscall's exact signature and semantics, errno included. Under a
 * tape they serve from it or record to it; otherwise they *are* the syscall. So
 * a call site changes by one identifier.
 *
 * On replay these never touch the filesystem: `open` hands back the recorded fd
 * without opening anything, and every later call on that fd is served from the
 * tape. An fd operation we failed to record would therefore hit a descriptor
 * that was never opened and fail loudly -- which is the behavior we want.
 */
#include <sys/stat.h>

int rb_tape_close(int fd);
int rb_tape_isatty(int fd);
int rb_tape_fstat(int fd, struct stat *st);
int rb_tape_stat(const char *path, struct stat *st);
int rb_tape_lstat(const char *path, struct stat *st);
off_t rb_tape_lseek(int fd, off_t offset, int whence);

/* The filesystem mutations. Drop-ins, like the rest: same signature, same
 * semantics, errno included. On replay they touch nothing and serve the recorded
 * result -- so the mkdir does not happen, and the stat that observes it afterwards
 * comes off the tape saying the directory is there. */
#include <sys/time.h>
int rb_tape_mkdir(const char *path, mode_t mode);
int rb_tape_rmdir(const char *path);
int rb_tape_unlink(const char *path);
int rb_tape_rename(const char *from, const char *to);
int rb_tape_chmod(const char *path, mode_t mode);
int rb_tape_fchmod(int fd, mode_t mode);
int rb_tape_chown(const char *path, uid_t owner, gid_t group);
int rb_tape_lchown(const char *path, uid_t owner, gid_t group);
int rb_tape_symlink(const char *from, const char *to);
int rb_tape_link(const char *from, const char *to);
int rb_tape_truncate(const char *path, off_t len);
int rb_tape_ftruncate(int fd, off_t len);
int rb_tape_utimes(const char *path, const struct timeval *times);

/**
 * `open` is the exception to the drop-in shape. `rb_cloexec_open` runs fcntl
 * fixups on the fd it just opened, and a replayed fd was never really opened --
 * so replay has to return *before* them. The two halves are exposed separately
 * and called from either end of that function.
 */
int  rb_tape_replay_open(const char *path);
void rb_tape_record_open(const char *path, int flags, int fd, int err);

/*
 * A deterministic stand-in for an object's address.
 *
 * `rb_any_to_s` (object.c:721) formats `#<Foo:%p>` -- the raw heap address --
 * and `inspect` delegates to it. So `puts obj` on any plain object leaks ASLR
 * and allocation history straight into ordinary program output, and every such
 * program diverges on replay.
 *
 * (object_id is fine: since 3.5 it is a monotonic counter, not an address.)
 *
 * Under a tape, hand out a sequential stand-in instead: the first address asked
 * about gets one, the second another, and the same address always gets the same
 * one. Allocation order is deterministic given a deterministic effect stream, so
 * the order these are *requested* in is deterministic too, and replay reproduces
 * them exactly. Values are spaced like real addresses so a to_s still reads like
 * a to_s.
 */
uintptr_t rb_tape_canonical_addr(const void *ptr);

/*
 * Directory iteration.
 *
 * readdir returns entries in raw filesystem order -- no sort -- so it is
 * nondeterministic across machines and across any change to the directory. And
 * without it on the tape, a program that lists a directory cannot be replayed
 * once the directory is gone.
 *
 * On replay rb_tape_opendir hands back a real DIR* on "/", so closedir stays
 * valid -- the same trick as rb_tape_replay_open handing back a real fd. The real
 * handle is never read; every entry comes off the tape.
 */
#include <dirent.h>
DIR *rb_tape_opendir(const char *path);
struct dirent *rb_tape_readdir(DIR *dirp);
int rb_tape_closedir(DIR *dirp);

/** ENV. Returns the recorded value on replay; NULL for an unset variable. */
const char *rb_tape_getenv(const char *name);

/** getpid, taped. `long` rather than rb_pid_t so this header stays free of ruby.h. */
long rb_tape_getpid(void);

/**
 * Make the std streams' TTY-ness match the tape. Defined in io.c (it needs
 * rb_io_t); called from ruby.c right after the tape is armed.
 *
 * Ruby decides whether stdout is a TTY in Init_IO -- long before the tape
 * exists -- and that one bit picks line-buffered vs fully-buffered writes
 * (FMODE_TTY, io.c:1966). Recording isatty at its call site is therefore too
 * late: a tape taken through a pipe would replay under a terminal with a
 * completely different write sequence, and diverge. So on replay we force the
 * std streams back to the buffering they had when the tape was cut.
 */
void rb_tape_reconcile_stdio_tty(void);

#ifdef HAVE_WRITEV
#include <sys/uio.h>
/**
 * Buffered writes (`puts` and friends) reach the kernel through `writev`, not
 * `write`. Both are recorded as `io.write`: the gather payload is the
 * concatenation of every iovec, with no per-buffer framing -- which is exactly
 * how Watt captures a gather arg, so the two agree byte for byte.
 */
void   rb_tape_record_writev(int fd, const struct iovec *iov, int iovcnt, ssize_t ret, int err);
ssize_t rb_tape_replay_writev(int fd, const struct iovec *iov, int iovcnt);
#endif

#endif /* RUBY_TAPE_H */
