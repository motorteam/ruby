/**********************************************************************

  tape_view.c - the call tree of a replayed run, and a viewer for it

  The equivalent of `watt tape view`. Watt gets its call tree by recompiling
  the program with `span_enter`/`span_exit` imports and replaying against that
  instrumented build; the tree is *never* stored on the tape. We do the same
  thing, but Ruby hands us the instrumentation for free: RUBY_EVENT_CALL /
  RETURN / C_CALL / C_RETURN / B_CALL / B_RETURN, delivered through TracePoint.
  So the tape stays small and the tree is re-derived on demand.

  The one hard rule: **the tracer must never run user Ruby code.** We are inside
  a replay, and a user-defined `#inspect` that called Time.now would consume a
  tape entry and manufacture a divergence out of thin air. So every value is
  formatted in C (see fmt_value), and the only Ruby method we ever invoke is
  Binding#local_variable_get, which is C-implemented and touches no effect.

**********************************************************************/

#include "ruby/internal/config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "internal.h"
#include "internal/hash.h"
#include "ruby/debug.h"
#include "ruby/ruby.h"
#include "tape.h"
#include "tape_view.h"
#include "vm_core.h"

#define VIEW_EVENTS (RUBY_EVENT_CALL   | RUBY_EVENT_RETURN | \
                     RUBY_EVENT_C_CALL | RUBY_EVENT_C_RETURN | \
                     RUBY_EVENT_B_CALL | RUBY_EVENT_B_RETURN)

/* Frames deeper than this are recorded but not walked into; a runaway recursion
 * should not take the viewer down with it. */
#define VIEW_MAX_DEPTH 256

/* Lines of source shown above and below the call site -- Watt's SOURCE_CONTEXT. */
#define VIEW_SOURCE_CONTEXT 4

typedef enum {
    SPAN_RETURNED,  /* the call returned normally */
    SPAN_ESCAPED    /* unwound by an exception, or still open when the run ended */
} span_state;

typedef struct {
    int parent;
    int *children;
    size_t n_children, cap_children;

    char *label;    /* "Integer#+", "Object#ask", "block in <main>" */
    char *args;     /* "prompt=\"Speak your name, mortal: \"" */
    char *rv;       /* rendered return value; NULL while open */
    char *file;     /* call-site path, or NULL */
    int line;

    int depth;
    int is_c;
    span_state state;
} view_node;

static struct {
    view_node *nodes;
    size_t n, cap;

    int open;       /* the innermost span currently on the stack */
    int enabled;    /* --tape-view was given: render at the end */
    int tracing;    /* the program is still running: keep building the tree */
    int in_hook;    /* re-entrancy guard: our own rb_funcall re-enters the hook */
    VALUE tp;       /* the TracePoint, GC-pinned */
} view;

/* ── Strings ──────────────────────────────────────────────────────────────── */

static char *
dupstr(const char *s)
{
    if (!s) return NULL;
    size_t n = strlen(s) + 1;
    char *out = xmalloc(n);
    memcpy(out, s, n);
    return out;
}

static char *
vfmt(const char *fmt, ...)
{
    va_list ap;
    char buf[1024];
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return dupstr(buf);
}

/* ── Value formatting, without running a line of Ruby ─────────────────────────
 *
 * Watt renders a value from its static type (`value_display`). We have no
 * static types, so we switch on the object's C-level shape. Crucially this
 * calls no user code: `#inspect` and `#to_s` are off limits inside a replay.
 */

static void
fmt_string(VALUE v, char *out, size_t cap)
{
    const char *p = RSTRING_PTR(v);
    long len = RSTRING_LEN(v);
    size_t o = 0;

    if (o < cap - 1) out[o++] = '"';
    for (long i = 0; i < len && o < cap - 8; i++) {
        unsigned char c = (unsigned char)p[i];
        switch (c) {
          case '\n': out[o++] = '\\'; out[o++] = 'n'; break;
          case '\t': out[o++] = '\\'; out[o++] = 't'; break;
          case '"':  out[o++] = '\\'; out[o++] = '"'; break;
          case '\\': out[o++] = '\\'; out[o++] = '\\'; break;
          default:
            if (c >= 0x80 || (c >= 0x20 && c < 0x7f)) out[o++] = (char)c;
            else o += (size_t)snprintf(out + o, cap - o, "\\x%02x", c);
        }
    }
    if (len > 0 && o >= cap - 8) { out[o++] = '.'; out[o++] = '.'; out[o++] = '.'; }
    if (o < cap - 1) out[o++] = '"';
    out[o] = '\0';
}

static void
fmt_value(VALUE v, char *out, size_t cap)
{
    switch (TYPE(v)) {
      case T_NIL:    snprintf(out, cap, "nil"); return;
      case T_TRUE:   snprintf(out, cap, "true"); return;
      case T_FALSE:  snprintf(out, cap, "false"); return;
      case T_FIXNUM: snprintf(out, cap, "%ld", FIX2LONG(v)); return;
      case T_FLOAT:  snprintf(out, cap, "%g", RFLOAT_VALUE(v)); return;
      case T_SYMBOL: snprintf(out, cap, ":%s", rb_id2name(SYM2ID(v))); return;
      case T_STRING: fmt_string(v, out, cap); return;
      case T_ARRAY:  snprintf(out, cap, "[%ld items]", RARRAY_LEN(v)); return;
      case T_HASH:   snprintf(out, cap, "{%ld pairs}", (long)rb_hash_size_num(v)); return;
      case T_BIGNUM: snprintf(out, cap, "<bignum>"); return;
      default: break;
    }
    /* rb_obj_class + rb_class2name are C-level; no user code runs. */
    const char *cls = rb_class2name(rb_obj_class(v));
    snprintf(out, cap, "#<%s>", cls ? cls : "?");
}

/* ── Tree ─────────────────────────────────────────────────────────────────── */

static int
node_new(int parent)
{
    if (view.n == view.cap) {
        view.cap = view.cap ? view.cap * 2 : 1024;
        view.nodes = xrealloc(view.nodes, view.cap * sizeof(view_node));
    }
    view_node *n = &view.nodes[view.n];
    memset(n, 0, sizeof(*n));
    n->parent = parent;
    n->line = -1;
    n->state = SPAN_ESCAPED;  /* until proven otherwise, as in Watt */
    return (int)view.n++;
}

static void
node_add_child(int parent, int child)
{
    if (parent < 0) return;
    view_node *p = &view.nodes[parent];
    if (p->n_children == p->cap_children) {
        p->cap_children = p->cap_children ? p->cap_children * 2 : 4;
        p->children = xrealloc(p->children, p->cap_children * sizeof(int));
    }
    p->children[p->n_children++] = child;
}

/**
 * The args cell: the method's declared parameters and their values at entry.
 *
 * `rb_tracearg_parameters` names them; the binding supplies the values. We only
 * ever call Binding#local_variable_get, which is implemented in C -- so no user
 * code runs and the replay is not perturbed.
 */
static char *
capture_args(rb_trace_arg_t *arg, rb_event_flag_t ev)
{
    if (ev & (RUBY_EVENT_C_CALL | RUBY_EVENT_C_RETURN)) return NULL;  /* no binding */

    VALUE params = rb_tracearg_parameters(arg);
    if (!RB_TYPE_P(params, T_ARRAY) || RARRAY_LEN(params) == 0) return NULL;

    VALUE binding = rb_tracearg_binding(arg);
    if (NIL_P(binding)) return NULL;

    char buf[768];
    size_t o = 0;
    ID lvget = rb_intern("local_variable_get");

    for (long i = 0; i < RARRAY_LEN(params) && o < sizeof(buf) - 64; i++) {
        VALUE pair = RARRAY_AREF(params, i);
        if (!RB_TYPE_P(pair, T_ARRAY) || RARRAY_LEN(pair) < 2) continue;
        VALUE name = RARRAY_AREF(pair, 1);
        if (!RB_TYPE_P(name, T_SYMBOL)) continue;

        VALUE val = rb_funcall(binding, lvget, 1, name);

        char rendered[256];
        fmt_value(val, rendered, sizeof(rendered));
        o += (size_t)snprintf(buf + o, sizeof(buf) - o, "%s%s=%s",
                              o ? ", " : "", rb_id2name(SYM2ID(name)), rendered);
    }
    return o ? dupstr(buf) : NULL;
}

/**
 * "Class#method" for an instance method, "Class.method" for a singleton one,
 * "block in ..." for a block.
 *
 * A singleton method's defined_class is the singleton class, which has no name
 * of its own -- so `Time.now` would otherwise render as a bare `now`. Ask the
 * singleton for the object it is attached to and name *that*.
 */
static char *
capture_label(rb_trace_arg_t *arg, rb_event_flag_t ev)
{
    VALUE mid = rb_tracearg_method_id(arg);
    VALUE klass = rb_tracearg_defined_class(arg);

    const char *m = NIL_P(mid) ? "<main>" : rb_id2name(SYM2ID(mid));
    const char *k = NULL;
    const char *sep = "#";

    if (!NIL_P(klass) && RB_TYPE_P(klass, T_CLASS) && FL_TEST(klass, FL_SINGLETON)) {
        VALUE owner = rb_class_attached_object(klass);
        VALUE name = (RB_TYPE_P(owner, T_CLASS) || RB_TYPE_P(owner, T_MODULE))
                     ? rb_mod_name(owner) : Qnil;
        if (RB_TYPE_P(name, T_STRING)) { k = RSTRING_PTR(name); sep = "."; }
    }
    else if (!NIL_P(klass)) {
        VALUE name = rb_mod_name(klass);
        if (RB_TYPE_P(name, T_STRING)) k = RSTRING_PTR(name);
    }

    int block = (ev & (RUBY_EVENT_B_CALL | RUBY_EVENT_B_RETURN)) != 0;
    if (block) return k ? vfmt("block in %s%s%s", k, sep, m) : vfmt("block in %s", m);
    return k ? vfmt("%s%s%s", k, sep, m) : vfmt("%s", m);
}

static void
view_enter(rb_trace_arg_t *arg, rb_event_flag_t ev)
{
    int id = node_new(view.open);
    node_add_child(view.open, id);

    view_node *n = &view.nodes[id];
    n->depth = view.open < 0 ? 0 : view.nodes[view.open].depth + 1;
    n->is_c = (ev & (RUBY_EVENT_C_CALL | RUBY_EVENT_C_RETURN)) != 0;
    n->label = capture_label(arg, ev);

    if (n->depth < VIEW_MAX_DEPTH) {
        n->args = capture_args(arg, ev);
    }

    VALUE path = rb_tracearg_path(arg);
    if (RB_TYPE_P(path, T_STRING)) n->file = dupstr(RSTRING_PTR(path));
    VALUE line = rb_tracearg_lineno(arg);
    if (FIXNUM_P(line)) n->line = (int)FIX2INT(line);

    view.open = id;
}

static void
view_exit(rb_trace_arg_t *arg)
{
    if (view.open < 0) return;

    view_node *n = &view.nodes[view.open];
    char rendered[256];
    fmt_value(rb_tracearg_return_value(arg), rendered, sizeof(rendered));
    n->rv = dupstr(rendered);
    n->state = SPAN_RETURNED;

    view.open = n->parent;
}

static void
view_hook(VALUE tpval, void *unused)
{
    if (!view.tracing) return;
    if (view.in_hook) return;  /* our own local_variable_get re-enters here */
    view.in_hook = 1;

    rb_trace_arg_t *arg = rb_tracearg_from_tracepoint(tpval);
    rb_event_flag_t ev = rb_tracearg_event_flag(arg);

    if (ev & (RUBY_EVENT_CALL | RUBY_EVENT_C_CALL | RUBY_EVENT_B_CALL)) {
        view_enter(arg, ev);
    }
    else {
        view_exit(arg);
    }

    view.in_hook = 0;
}

void
rb_tape_view_enable(void)
{
    view.enabled = 1;
    view.tracing = 1;

    /* Top-level code emits no CALL event -- it is not a method -- so without a
     * synthetic root every top-level call would be its own tree. Watt's viewer
     * shows one trunk; so do we. */
    view.open = node_new(-1);
    view.nodes[view.open].label = dupstr("<main>");
    view.nodes[view.open].state = SPAN_RETURNED;
    view.nodes[view.open].rv = dupstr("nil");

    view.tp = rb_tracepoint_new(Qnil, VIEW_EVENTS, view_hook, NULL);
    rb_gc_register_mark_object(view.tp);
    rb_tracepoint_enable(view.tp);
}

void
rb_tape_view_stop(void)
{
    if (!view.tracing) return;
    view.tracing = 0;
    if (view.tp) rb_tracepoint_disable(view.tp);
}

int
rb_tape_viewing(void)
{
    return view.enabled;
}

/* ── Rendering ────────────────────────────────────────────────────────────── */

/* The visible (expanded) rows, recomputed whenever the tree is folded. */
static int *rows;
static size_t n_rows, cap_rows;
static char *expanded;  /* one flag per node */

static void
rows_push(int id)
{
    if (n_rows == cap_rows) {
        cap_rows = cap_rows ? cap_rows * 2 : 256;
        rows = xrealloc(rows, cap_rows * sizeof(int));
    }
    rows[n_rows++] = id;
}

static void
rows_walk(int id)
{
    rows_push(id);
    if (!expanded[id]) return;
    view_node *n = &view.nodes[id];
    for (size_t i = 0; i < n->n_children; i++) rows_walk(n->children[i]);
}

static void
rows_rebuild(void)
{
    n_rows = 0;
    for (size_t i = 0; i < view.n; i++) {
        if (view.nodes[i].parent < 0) rows_walk((int)i);
    }
}

/** `func(a = 1, b = 2) ↦ rv`, or ` ↗` for a span an exception unwound. */
static void
row_text(int id, char *out, size_t cap)
{
    view_node *n = &view.nodes[id];
    size_t o = 0;

    for (int i = 0; i < n->depth && o < cap - 4; i++) {
        o += (size_t)snprintf(out + o, cap - o, "  ");
    }

    const char *marker = n->n_children ? (expanded[id] ? "▾ " : "▸ ") : "  ";
    o += (size_t)snprintf(out + o, cap - o, "%s%s", marker, n->label ? n->label : "?");

    if (n->args) o += (size_t)snprintf(out + o, cap - o, "(%s)", n->args);

    if (n->state == SPAN_ESCAPED) {
        snprintf(out + o, cap - o, " ↗");
    }
    else if (n->rv) {
        snprintf(out + o, cap - o, " ↦ %s", n->rv);
    }
}

/* ── Terminal ─────────────────────────────────────────────────────────────── */

static struct termios saved_term;
static int raw_mode;

static void
term_raw(void)
{
    if (tcgetattr(STDIN_FILENO, &saved_term) < 0) return;
    struct termios t = saved_term;
    t.c_lflag &= ~(unsigned long)(ICANON | ECHO);
    t.c_cc[VMIN] = 1;
    t.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &t);
    raw_mode = 1;
    fputs("\x1b[?1049h", stdout);  /* alternate screen, as ratatui does */
}

static void
term_restore(void)
{
    if (!raw_mode) return;
    fputs("\x1b[?1049l\x1b[?25h", stdout);
    fflush(stdout);
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &saved_term);
    raw_mode = 0;
}

static void
term_size(int *w, int *h)
{
    struct winsize ws;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_row > 0) {
        *w = ws.ws_col;
        *h = ws.ws_row;
    }
    else {
        *w = 80;
        *h = 24;
    }
}

/* ── Source pane ──────────────────────────────────────────────────────────── */

/**
 * Print the call site with context, the call line marked.
 *
 * Read with plain stdio, deliberately: the tape's own IO chokepoints must not
 * see this. The viewer runs after the replay has finished, but a file read that
 * went through rb_tape_* would still try to take an entry off the tape.
 */
static int
draw_source(int id, int top, int height, int width)
{
    view_node *n = &view.nodes[id];
    int row = top;

    printf("\x1b[%d;1H\x1b[7m %-*s\x1b[0m", row++, width - 1,
           n->file ? n->file : "(no source)");

    if (!n->file || n->line < 1) return row;

    FILE *f = fopen(n->file, "r");
    if (!f) {
        printf("\x1b[%d;1H  (cannot read %s)", row++, n->file);
        return row;
    }

    int from = n->line - VIEW_SOURCE_CONTEXT;
    if (from < 1) from = 1;
    int to = n->line + VIEW_SOURCE_CONTEXT;

    char line[512];
    int lineno = 0;
    while (fgets(line, sizeof(line), f) && lineno < to && row < top + height) {
        lineno++;
        if (lineno < from) continue;
        line[strcspn(line, "\n")] = '\0';

        if (lineno == n->line) {
            printf("\x1b[%d;1H\x1b[1;33m%5d │ %.*s\x1b[0m", row++, lineno, width - 9, line);
        }
        else {
            printf("\x1b[%d;1H\x1b[2m%5d │ %.*s\x1b[0m", row++, lineno, width - 9, line);
        }
    }
    fclose(f);
    return row;
}

/* ── The viewer ───────────────────────────────────────────────────────────── */

static void
draw(int cursor, int scroll)
{
    int w, h;
    term_size(&w, &h);

    int tree_h = h * 2 / 3;
    int src_h = h - tree_h - 1;

    fputs("\x1b[2J\x1b[?25l", stdout);

    printf("\x1b[1;1H\x1b[7m ruby tape view — %zu calls   "
           "j/k move  l/h expand/collapse  q quit \x1b[0m", view.n);

    for (int i = 0; i < tree_h - 1; i++) {
        size_t r = (size_t)(scroll + i);
        if (r >= n_rows) break;

        char text[1024];
        row_text(rows[r], text, sizeof(text));

        int selected = (int)r == cursor;
        view_node *n = &view.nodes[rows[r]];

        printf("\x1b[%d;1H", i + 2);
        if (selected) fputs("\x1b[7m", stdout);
        else if (n->is_c) fputs("\x1b[2m", stdout);   /* C methods recede */
        else if (n->state == SPAN_ESCAPED) fputs("\x1b[31m", stdout);

        printf("%-*.*s\x1b[0m", w - 1, w - 1, text);
    }

    if (n_rows > 0) draw_source(rows[cursor], tree_h + 1, src_h, w);
    fflush(stdout);
}

static void
view_run(void)
{
    if (view.n == 0) {
        fputs("[tape] no calls were traced\n", stderr);
        return;
    }

    expanded = xcalloc(view.n, 1);
    /* Open the first two levels: the whole tree collapsed shows nothing useful,
     * fully expanded is a wall. */
    for (size_t i = 0; i < view.n; i++) {
        if (view.nodes[i].depth < 2) expanded[i] = 1;
    }
    rows_rebuild();

    term_raw();

    int cursor = 0, scroll = 0;
    for (;;) {
        int w, h;
        term_size(&w, &h);
        int page = h * 2 / 3 - 1;

        if (cursor < scroll) scroll = cursor;
        if (cursor >= scroll + page) scroll = cursor - page + 1;

        draw(cursor, scroll);

        int c = getchar();
        if (c == 'q' || c == 27 || c == EOF) break;

        int id = rows[cursor];
        switch (c) {
          case 'j': if ((size_t)cursor + 1 < n_rows) cursor++; break;
          case 'k': if (cursor > 0) cursor--; break;
          case 'l':
            if (view.nodes[id].n_children && !expanded[id]) {
                expanded[id] = 1;
                rows_rebuild();
            }
            else if ((size_t)cursor + 1 < n_rows) cursor++;
            break;
          case 'h':
            if (expanded[id] && view.nodes[id].n_children) {
                expanded[id] = 0;
                rows_rebuild();
            }
            else if (view.nodes[id].parent >= 0) {
                /* Jump to the parent row, as Watt's viewer does. */
                for (size_t r = 0; r < n_rows; r++) {
                    if (rows[r] == view.nodes[id].parent) { cursor = (int)r; break; }
                }
            }
            break;
          case 'g': cursor = 0; break;
          case 'G': cursor = (int)n_rows - 1; break;
          default: break;
        }
    }

    term_restore();
}

/** Print the tree as plain text, for a pipe or a terminal-less run. */
static void
view_dump(void)
{
    int color = isatty(STDOUT_FILENO);
    for (size_t i = 0; i < view.n; i++) {
        view_node *n = &view.nodes[i];
        for (int d = 0; d < n->depth; d++) fputs("  ", stdout);
        printf("%s", n->label ? n->label : "?");
        if (n->args) printf("(%s)", n->args);
        if (n->state == SPAN_ESCAPED) printf(" ↗");
        else if (n->rv) printf(" ↦ %s", n->rv);
        if (n->file && n->line > 0) {
            if (color) printf("   \x1b[2m%s:%d\x1b[0m", n->file, n->line);
            else printf("   %s:%d", n->file, n->line);
        }
        putchar('\n');
    }
}

void
rb_tape_view_show(void)
{
    if (!view.enabled) return;
    view.enabled = 0;
    rb_tape_view_stop();

    if (isatty(STDIN_FILENO) && isatty(STDOUT_FILENO)) view_run();
    else view_dump();
}
