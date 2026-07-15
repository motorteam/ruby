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
#include <sys/socket.h>   /* struct sockaddr, socklen_t -- for the socket effects */

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
    /* libc's realpath(3). `File.realpath` does not walk the path with lstat on the
     * happy path -- it hands the whole thing to libc, which does its stats *inside
     * libc*, where no chokepoint in this tree can see them. So a successful
     * File.realpath put **nothing at all** on the tape.
     *
     * That was invisible until replay stopped creating directories for real. Then:
     * the recorded run's realpath succeeded silently, the replayed run's failed with
     * ENOENT (the directory was never made), CRuby fell back to its own lstat-walking
     * emulation, and the extra stats desynced the tape. The fifth time this tree has
     * taught the same lesson -- the funnel is never where you think. */
    RB_TAPE_FS_REALPATH     = 20,
    /* The rest of the path-based surface. All of these became *load-bearing* the
     * moment replay stopped creating directories for real: a program whose temp
     * directory was never made now asks the live filesystem about it, and gets a
     * different answer than the recording did. `Dir.chdir` into a suppressed tmpdir
     * raises ENOENT, and the test blows up in setup with nothing on the tape to say
     * why. */
    RB_TAPE_FS_GETCWD       = 21,
    RB_TAPE_FS_ACCESS       = 22,   /* access(2) and eaccess -- File.readable? &c. */
    RB_TAPE_FS_READLINK     = 23,
    /**
     * Not an effect -- a *marker*. Written whenever the thread doing the recording
     * changes, carrying the new thread's serial.
     *
     * A tape is one linear log, and two threads doing IO at once append to it in
     * whatever order they finish -- a truthful recording, but not one that replays as
     * a single sequence, because the second run will not interleave them the same way.
     * The tape is instead cut into one stream per *resource* (see tape_stream in
     * tape.c): same-resource effects stay ordered, disjoint ones float. The markers are
     * how the decoder knows which thread performed each effect, so that effects with no
     * shared host object of their own -- a clock read, an ENV lookup -- are keyed to
     * the performing thread rather than falsely ordered against another thread's.
     *
     * The marker rides on the effect table rather than in the entry struct, so the wire
     * format stays Watt's, byte for byte.
     */
    RB_TAPE_VM_THREAD       = 24,
    /**
     * Subprocesses. Replay was still *really spawning them* -- and that is not just
     * a leak, it is a deadlock.
     *
     * A recorded run spawns a child, writes it a script down a pipe, and waits. On
     * replay the fork and exec were untaped, so they happened for real -- but the
     * write that feeds the child is an effect, and effects are suppressed. So the
     * child sat forever on an empty stdin, the parent sat forever in waitpid, and the
     * test hung. 45 of them did.
     *
     * Half-executing a subprocess is the same mistake as half-executing a mkdir, and
     * it has the same fix: don't. On replay the spawn returns the recorded pid without
     * forking anything, waitpid returns the recorded status without waiting, and the
     * child's output comes back off the tape -- where it was already being recorded,
     * as ordinary reads of a pipe.
     */
    RB_TAPE_PROC_SPAWN      = 25,
    RB_TAPE_PROC_WAITPID    = 26,
    /* The fds a pipe hands back. They have to come off the tape too, or the reads that
     * follow them arrive on descriptors the recording never saw. */
    RB_TAPE_FS_PIPE         = 27,
    /* fcntl(fd, F_GETFL) -- how `IO.new(fd)` interrogates a descriptor before it will
     * wrap it. On a replayed fd, which was never opened, it fails, IO.new raises
     * EBADF, and the program dies in a place that has nothing to do with tapes. This
     * is the same trap rb_cloexec_open already sidesteps by returning before its own
     * fixups; IO.new asks on the program's behalf, so it has to be answered instead. */
    RB_TAPE_FS_FCNTL        = 28,
    /**
     * Readiness -- `IO.select`, and everything that waits on it.
     *
     * This is the effect the fiber scheduler is *made of*. `test/fiber/scheduler.rb`
     * is a loop around `IO.select`, and which descriptors it names on the way out is
     * what decides which fiber resumes next. Untaped, a replayed scheduler selects on
     * descriptors that were never opened, gets an answer the recording never saw, and
     * resumes its fibers in a different order -- so their effects arrive in a different
     * order, and the tape says the thread read where it should have closed.
     *
     * The readiness verdict, not the polling: what goes on the tape is *which fds came
     * back ready*, which is the only thing the program can see.
     */
    RB_TAPE_IO_SELECT       = 29,
    /**
     * Program text -- the half of the loader that *is* an effect.
     *
     * The founding rule was "program text is not an effect", and for a program whose
     * source is static and present at replay it is exactly right: Watt stores the
     * assembly beside the tape and recompiles; CPython keeps imports off the tape.
     *
     * It is exactly wrong for a program that **writes code at runtime and then loads
     * it**, which is what a package manager's test suite does for a living. Replay
     * (correctly) never creates the directories the recording created, so a `require`
     * that succeeded when the tape was cut finds nothing -- and because the loader ran
     * paused, *nothing on the tape disagreed*. The program's state simply parted company
     * with the recording, silently, and diverged a thousand effects later somewhere
     * unrelated. Forty-nine test files.
     *
     * So the line is drawn where it actually holds: **the interpreter's own library
     * directories -- the load path as it stands before any user code runs -- are off the
     * tape.** They are versioned with the binary, and a tape already carries a build_id,
     * so a tape only ever replays against the ruby that cut it. Everything else is the
     * program's, might differ between runs, and goes on the tape.
     *
     * That also buys something the tape could not do before: **it now notices when the
     * program's own source has changed.** Edit a file, replay an old tape, and the load
     * diverges at the file -- which is Watt's assembly hash, arrived at from the other
     * direction, and it is what makes "record before a refactor, replay after" mean
     * anything.
     */
    RB_TAPE_FS_LOADOK       = 30,   /* the $LOAD_PATH probe's verdict (rb_file_load_ok) */
    RB_TAPE_FS_LOADFILE     = 31,   /* the source Prism actually read */
    /**
     * The real name of a path -- `getattrlist(2)`, and the sixth time the funnel was
     * not where we thought.
     *
     * On macOS every *plain* component of every glob is resolved by
     * `replace_real_basename` (dir.c), which asks getattrlist for three things at once:
     * does this path exist, what type is it, and what is its real on-disk spelling.
     * It is not open, not stat, not lstat -- it is a single Darwin syscall that no
     * chokepoint in this tree could see, and `Dir.glob` asks it *before* it opens
     * anything.
     *
     * Untaped, a replayed glob therefore asked the *live* filesystem about a directory
     * that replay had (correctly) declined to create, was told path_noent, and returned
     * zero matches **having emitted not one tape entry**. Nothing disagreed, because
     * nothing was recorded. The program's Ruby-level state simply parted company with
     * the recording -- `Gem::Specification.stubs` came back empty -- and diverged
     * hundreds of effects later somewhere with nothing to do with directories.
     *
     * This is the whole rubygems cluster, and it is not a rubygems bug: a seven-line
     * program that globs a tmpdir and then deletes it could not replay.
     *
     * (On Linux there is no getattrlist, USE_NAME_ON_FS is 0, and glob reaches the same
     * directories through do_opendir instead -- which is why this hid so well.)
     */
    RB_TAPE_FS_REALNAME     = 32,

    /* Sockets. A socket's fd is created by a syscall the tape did not hook, so on replay
     * it is a real kernel fd whose *number* need not match the recording's -- and every
     * later read, write and fcntl is keyed by that number. The fix is the one files use:
     * serve the recorded fd and never really open. That makes the fd fully virtual, so
     * the whole lifecycle has to come off the tape too -- connect, accept (a new fd),
     * bind, listen, the names, the options, and address resolution. Append only. */
    RB_TAPE_NET_SOCKET      = 33,   /* socket(2) and socketpair(2) -- returns the fd(s) */
    RB_TAPE_NET_CONNECT     = 34,
    RB_TAPE_NET_ACCEPT      = 35,   /* accept(2) -- returns a new fd and the peer address */
    RB_TAPE_NET_BIND        = 36,
    RB_TAPE_NET_LISTEN      = 37,
    RB_TAPE_NET_SOCKNAME    = 38,   /* getsockname/getpeername -- returns an address */
    RB_TAPE_NET_SOCKOPT     = 39,   /* getsockopt (setsockopt just returns a status) */
    RB_TAPE_NET_GETADDRINFO = 40,   /* getaddrinfo(3) -- returns the resolved list */
    RB_TAPE_NET_RECVFROM    = 41,   /* recv/recvfrom -- bytes plus the peer address */
    RB_TAPE_NET_GETNAMEINFO = 42,   /* getnameinfo(3) -- returns host/serv strings */
    RB_TAPE_NET_SEND        = 43,   /* send/sendto/sendmsg -- the byte count returned */
    RB_TAPE_NET_RECVMSG     = 44,   /* recvmsg -- bytes, control (passed fds), address */

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
    /* chdir mutates the process, not the filesystem -- but it is the same shape and
     * replay must do the same thing with it: serve the recorded result and stay put.
     * A replayed program does not have the directory it chdir'd into, because replay
     * is what declined to create it. */
    RB_TAPE_FS_OP_CHDIR     = 13,
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

/* Exported: ext/socket asks these to decide whether a setup fixup (nonblock, cloexec)
 * should touch a fd that, on replay, is virtual. */
RUBY_SYMBOL_EXPORT_BEGIN
int rb_tape_recording(void);
int rb_tape_replaying(void);
RUBY_SYMBOL_EXPORT_END

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

/**
 * Snapshot the interpreter's own library directories: the load path as it stands
 * *before any user code runs*. Called from ruby.c right after the tape is armed.
 *
 * Everything under them is the stdlib -- versioned with the binary, guaranteed present
 * at replay, and therefore not an effect. Everything else is the program's.
 */
void rb_tape_snapshot_stdlib_path(void);

/** True if `path` is the program's own text rather than the interpreter's library. */
int rb_tape_program_text_p(const char *path);

/* The loader, for program text only. See RB_TAPE_FS_LOADOK. */
int rb_tape_replay_loadok(const char *path);
void rb_tape_record_loadok(const char *path, int ok);

/** Returns a pointer into the tape (valid for the process's life), or NULL. */
const unsigned char *rb_tape_replay_loadfile(const char *path, size_t *len);
void rb_tape_record_loadfile(const char *path, const unsigned char *bytes, size_t len);

/**
 * getattrlist(2)'s answer, taped. See RB_TAPE_FS_REALNAME.
 *
 * `objtype` is the platform's fsobj_type_t, carried opaquely: the tape does not need to
 * know what VREG means, only that the replay must be told the same thing the recording
 * was. Returns 0 and fills `name`/`objtype`, or -1 with errno restored.
 */
int  rb_tape_replay_realname(const char *path, char *name, size_t cap, int *objtype);
void rb_tape_record_realname(const char *path, const char *name, int objtype, int ret, int err);

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
/* Exported: ext/socket fstats a descriptor it received over a unix socket (SCM_RIGHTS)
 * to decide whether to wrap it as a Socket or an IO; on replay that fd is virtual. */
RUBY_SYMBOL_EXPORT_BEGIN
int rb_tape_fstat(int fd, struct stat *st);
RUBY_SYMBOL_EXPORT_END
int rb_tape_stat(const char *path, struct stat *st);
int rb_tape_lstat(const char *path, struct stat *st);
off_t rb_tape_lseek(int fd, off_t offset, int whence);

/**
 * realpath(3), taped. A drop-in: same signature, same semantics, errno included.
 * On replay it resolves nothing -- it hands back the path libc resolved when the
 * tape was cut, so a replayed program can realpath a directory that no longer
 * exists (and, more to the point, one that replay deliberately never created).
 */
char *rb_tape_realpath(const char *path, char *resolved);

int rb_tape_chdir(const char *path);
char *rb_tape_getcwd(char *buf, size_t size);
/**
 * access(2). `call` is access or file.c's eaccess (which is static there, so it comes
 * in as a pointer, the way tape_stat takes stat/lstat/fstat). `effective` only
 * separates the two on the tape: File.readable? and File.readable_real? ask different
 * questions and may get different answers.
 */
int rb_tape_access(const char *path, int mode, int effective, int (*call)(const char *, int));
ssize_t rb_tape_readlink(const char *path, char *buf, size_t size);

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

/* Subprocesses. `long` for the same reason. On replay none of these touch the host:
 * the spawn returns the recorded pid without forking, and the wait returns the
 * recorded status without waiting. */
long rb_tape_replay_spawn(void);
void rb_tape_record_spawn(long pid, int err);

long rb_tape_replay_waitpid(int *status);
void rb_tape_record_waitpid(long pid, int status, int err);

int rb_tape_pipe(int descriptors[2], int (*call)(int[2]));

/**
 * Sockets. Each is a drop-in for the syscall it wraps: on record it makes the real call
 * and tapes the result; on replay it serves the recording and touches no kernel socket, so
 * the fd it returns is virtual, exactly like a replayed file's. The peer/socket addresses
 * that accept/getsockname/recvfrom fill are scatter args, captured post-call and rewritten
 * on replay. See RB_TAPE_NET_SOCKET.
 *
 * Unlike every other tape hook, these are called from ext/socket -- a separately linked
 * extension -- so they have to be *exported* to be found at load time. The rest of the
 * tape lives entirely inside the ruby binary and needs no export.
 */
RUBY_SYMBOL_EXPORT_BEGIN
int rb_tape_socket(int domain, int type, int protocol);
int rb_tape_socketpair(int domain, int type, int protocol, int sv[2]);
int rb_tape_connect(int fd, const struct sockaddr *addr, socklen_t len);
int rb_tape_bind(int fd, const struct sockaddr *addr, socklen_t len);
int rb_tape_listen(int fd, int backlog);
int rb_tape_accept(int fd, struct sockaddr *addr, socklen_t *addrlen);
int rb_tape_getsockname(int fd, struct sockaddr *addr, socklen_t *addrlen);
int rb_tape_getpeername(int fd, struct sockaddr *addr, socklen_t *addrlen);
int rb_tape_getsockopt(int fd, int level, int optname, void *optval, socklen_t *optlen);
int rb_tape_setsockopt(int fd, int level, int optname, const void *optval, socklen_t optlen);
ssize_t rb_tape_send(int fd, const void *buf, size_t len, int flags);
ssize_t rb_tape_sendto(int fd, const void *buf, size_t len, int flags,
                       const struct sockaddr *to, socklen_t tolen);
ssize_t rb_tape_sendmsg(int fd, const struct msghdr *msg, int flags);
ssize_t rb_tape_recv(int fd, void *buf, size_t len, int flags);
ssize_t rb_tape_recvfrom(int fd, void *buf, size_t len, int flags,
                         struct sockaddr *from, socklen_t *fromlen);
ssize_t rb_tape_recvmsg(int fd, struct msghdr *msg, int flags);
RUBY_SYMBOL_EXPORT_END

/** fcntl(fd, cmd) -- the one-argument commands, F_GETFL and friends. */
int rb_tape_fcntl(int fd, int cmd);

/**
 * Readiness. The three sets are *scatter* args: on replay they are rewritten to name
 * exactly the descriptors the recording found ready, and nothing is polled.
 *
 * The fds are passed as plain arrays rather than fd_sets so this header stays free of
 * CRuby's rb_fdset_t -- the caller in thread.c does the marshalling, which is a dozen
 * lines and keeps the platform's fd_set representation out of the wire format.
 */
int rb_tape_replay_select(int *rfds, int *rn, int *wfds, int *wn, int *efds, int *en);
void rb_tape_record_select(int ret, int err,
                           const int *rfds, int rn, const int *wfds, int wn,
                           const int *efds, int en);

/**
 * Stop recording, permanently, on this process.
 *
 * Called in the child of a bare `fork` (no exec). The child inherits a live recorder
 * and, at its exit, would seal *its* tape over the parent's file -- so a program that
 * forked ended up with a tape of the child instead of the program. The child's
 * behavior is not lost: what the parent can observe of it comes back through the pipe
 * it reads, which is already an effect.
 */
void rb_tape_disarm(void);

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
