#ifndef RUBY_TAPE_VIEW_H                                 /*-*-C-*-vi:se ft=c:*/
#define RUBY_TAPE_VIEW_H
/**
 * @file
 * The call tree of a replayed run, and a viewer for it -- `watt tape view`.
 *
 * The tree is not on the tape. Watt re-derives it by replaying against a
 * trace-instrumented recompilation; we re-derive it from Ruby's own
 * RUBY_EVENT_CALL / RETURN / C_CALL / C_RETURN / B_CALL / B_RETURN, which cost
 * nothing to emit and mean the tape stays small.
 */

/** Start tracing. Called from ruby.c when --tape-view is given. */
void rb_tape_view_enable(void);

int rb_tape_viewing(void);

/**
 * Stop tracing. Must be called as soon as the program's body finishes and
 * before VM teardown: the hook reads bindings off live frames, and teardown
 * dismantles them underneath it.
 */
void rb_tape_view_stop(void);

/** Render the tree. Pure C -- safe to call during teardown. */
void rb_tape_view_show(void);

#endif /* RUBY_TAPE_VIEW_H */
