/**********************************************************************

  main.c -

  $Author$
  created at: Fri Aug 19 13:19:58 JST 1994

  Copyright (C) 1993-2007 Yukihiro Matsumoto

**********************************************************************/

/*!
 * \mainpage Developers' documentation for Ruby
 *
 * This documentation is produced by applying Doxygen to
 * <a href="https://github.com/ruby/ruby">Ruby's source code</a>.
 * It is still under construction (and even not well-maintained).
 * If you are familiar with Ruby's source code, please improve the doc.
 */
#undef RUBY_EXPORT
#include "ruby.h"
#include "tape.h"
#include "vm_debug.h"
#include "internal/sanitizers.h"
#ifdef HAVE_LOCALE_H
#include <locale.h>
#endif

#if defined RUBY_DEVEL && !defined RUBY_DEBUG_ENV
# define RUBY_DEBUG_ENV 1
#endif
#if defined RUBY_DEBUG_ENV && !RUBY_DEBUG_ENV
# undef RUBY_DEBUG_ENV
#endif

RUBY_GLOBAL_SETUP

static int
rb_main(int argc, char **argv)
{
    RUBY_INIT_STACK;
    /* Before ruby_init(), because the hash salt is drawn inside it -- in
     * Init_RandomSeedCore, before the command line is parsed and long before the
     * tape is armed. An unpinned salt makes `"x".hash` differ between record and
     * replay, and it cannot be re-seeded after the fact without corrupting every
     * st_table built during startup. So a taped run has to be recognized here, and
     * this peek at argv is the whole of what that takes. See tape.h. */
    rb_tape_scan_argv(argc, argv);
    ruby_init();
    return ruby_run_node(ruby_options(argc, argv));
}

#ifdef _WIN32
#define main(argc, argv) w32_main(argc, argv)
static int main(int argc, char **argv);
int wmain(void) {return main(0, NULL);}
#endif

int
main(int argc, char **argv)
{
#if defined(RUBY_DEBUG_ENV) || USE_RUBY_DEBUG_LOG
    ruby_set_debug_option(getenv("RUBY_DEBUG"));
#endif
#ifdef HAVE_LOCALE_H
    setlocale(LC_CTYPE, "");
#endif

    ruby_sysinit(&argc, &argv);
    return ruby_start_main(rb_main, argc, argv);
}
