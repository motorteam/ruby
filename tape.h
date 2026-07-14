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
    /* 13 is fs.lseek in the Python port; Ruby does not hook it yet. */
    RB_TAPE_FS_OPENDIR      = 14,
    RB_TAPE_FS_READDIR      = 15,
    RB_TAPE_FS_CLOSEDIR     = 16,
    /* Ruby reads ENV per name (getenv_with_lock, hash.c), where CPython
     * snapshots it into a dict at startup -- so this is a per-read effect rather
     * than the one-shot snapshot the Python port records. */
    RB_TAPE_ENV_GET         = 17,
    RB_TAPE_EFFECT_MAX
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
 * True if a tape is active at all. The chokepoints test this first so an
 * untaped run pays a single predictable branch and nothing else.
 */
#define RB_TAPE_ACTIVE() (rb_tape_recording() || rb_tape_replaying())

/* ── Per-effect record/replay pairs. ───────────────────────────────────────
 * Each chokepoint in the tree calls exactly one of these, so the patch at the
 * call site stays two or three lines and the marshalling lives here.
 */

void rb_tape_record_clock(int effect, const struct timespec *ts);
void rb_tape_replay_clock(int effect, struct timespec *ts);

void rb_tape_record_random(const void *buf, size_t len, int ret);
int  rb_tape_replay_random(void *buf, size_t len);

void   rb_tape_record_read(int fd, const void *buf, size_t capa, ssize_t ret);
ssize_t rb_tape_replay_read(int fd, void *buf, size_t capa);

void   rb_tape_record_write(int fd, const void *buf, size_t capa, ssize_t ret);
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
void   rb_tape_record_writev(int fd, const struct iovec *iov, int iovcnt, ssize_t ret);
ssize_t rb_tape_replay_writev(int fd, const struct iovec *iov, int iovcnt);
#endif

#endif /* RUBY_TAPE_H */
