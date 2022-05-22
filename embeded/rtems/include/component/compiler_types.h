#ifndef COMPONENT_COMPILER_TYPES_H_
#define COMPONENT_COMPILER_TYPES_H_

#ifndef ASM

/* Indirect macros required for expanded argument pasting, eg. __LINE__. */
#define ___PASTE(a,b) a##b
#define __PASTE(a,b) ___PASTE(a,b)

/* Compiler specific macros. */
#if defined(__clang__)
#include "component/compiler-clang.h"
#elif defined(__GNUC__)
/* The above compilers also define __GNUC__, so order is important here. */
#include "component/compiler-gcc.h"
#else
#error "Unknown compiler"
#endif

/*
 * Some architectures need to provide custom definitions of macros provided
 * by linux/compiler-*.h, and can do so using asm/compiler.h. We include that
 * conditionally rather than using an asm-generic wrapper in order to avoid
 * build failures if any C compilation, which will include this file via an
 * -include argument in c_flags, occurs prior to the asm-generic wrappers being
 * generated.
 */
#ifdef CONFIG_HAVE_ARCH_COMPILER_H
#include "bsp/asm/compiler.h"
#endif

/* Don't. Just don't. */
#define __deprecated
#endif /* ASM */
/*
 * The below symbols may be defined for one or more, but not ALL, of the above
 * compilers. We don't consider that to be an error, so set them to nothing.
 * For example, some of them are for compiler specific plugins.
 */
#ifndef __designated_init
# define __designated_init
#endif

#ifndef __latent_entropy
# define __latent_entropy
#endif

#ifndef __randomize_layout
# define __randomize_layout __designated_init
#endif

#ifndef __no_randomize_layout
# define __no_randomize_layout
#endif

#ifndef randomized_struct_fields_start
# define randomized_struct_fields_start
# define randomized_struct_fields_end
#endif

#ifndef __visible
#define __visible
#endif

/*
 * Assume alignment of return value.
 */
#ifndef __assume_aligned
#define __assume_aligned(a, ...)
#endif

#ifndef asm_volatile_goto
#define asm_volatile_goto(x...) asm goto(x)
#endif

/* Are two types/vars the same type (ignoring qualifiers)? */
#define __same_type(a, b) __builtin_types_compatible_p(typeof(a), typeof(b))

/* Is this type a native word size -- useful for atomic operations */
#define __native_word(t) \
	(sizeof(t) == sizeof(char) || sizeof(t) == sizeof(short) || \
	 sizeof(t) == sizeof(int) || sizeof(t) == sizeof(long))

#ifndef __attribute_const__
#define __attribute_const__	__attribute__((__const__))
#endif

#ifndef __noclone
#define __noclone
#endif

/* Helpers for emitting diagnostics in pragmas. */
#ifndef __diag
#define __diag(string)
#endif

#ifndef __diag_GCC
#define __diag_GCC(version, severity, string)
#endif

#ifndef __copy
# define __copy(symbol)
#endif

#define __diag_push()	__diag(push)
#define __diag_pop()	__diag(pop)

#define __diag_ignore(compiler, version, option, comment) \
	__diag_ ## compiler(version, ignore, option)
#define __diag_warn(compiler, version, option, comment) \
	__diag_ ## compiler(version, warn, option)
#define __diag_error(compiler, version, option, comment) \
	__diag_ ## compiler(version, error, option)

/*
 * From the GCC manual:
 *
 * Many functions have no effects except the return value and their
 * return value depends only on the parameters and/or global
 * variables.  Such a function can be subject to common subexpression
 * elimination and loop optimization just as an arithmetic operator
 * would be.
 * [...]
 */
#ifndef __pure
#define __pure			__attribute__((pure))
#endif
#ifndef __aligned
#define __aligned(x)		__attribute__((aligned(x)))
#endif
#ifndef __aligned_largest
#define __aligned_largest	__attribute__((aligned))
#endif
#ifndef __printf
#define __printf(a, b)		__attribute__((format(printf, a, b)))
#endif
#ifndef __scanf
#define __scanf(a, b)		__attribute__((format(scanf, a, b)))
#endif
#ifndef __maybe_unused
#define __maybe_unused		__attribute__((unused))
#endif
#ifndef __always_unused
#define __always_unused		__attribute__((unused))
#endif
#ifndef __mode
#define __mode(x)		__attribute__((mode(x)))
#endif
#ifndef __malloc
#define __malloc		__attribute__((__malloc__))
#endif
#ifndef __used
#define __used			__attribute__((__used__))
#endif
#ifndef __noreturn
#define __noreturn		__attribute__((noreturn))
#endif
#ifndef __packed
#define __packed		__attribute__((packed))
#endif
#ifndef __weak
#define __weak			__attribute__((weak))
#endif
#ifndef __alias
#define __alias(symbol)		__attribute__((alias(#symbol)))
#endif
#ifndef __cold
#define __cold			__attribute__((cold))
#endif
#ifndef __section
#define __section(S)		__attribute__((__section__(#S)))
#endif
#ifndef __must_check
#define __must_check		__attribute__((warn_unused_result))
#endif
#ifndef notrace
#define notrace			__attribute__((no_instrument_function))
#endif

/*
 * it doesn't make sense on ARM (currently the only user of __naked)
 * to trace naked functions because then mcount is called without
 * stack and frame pointer being set up and there is no chance to
 * restore the lr register to the value before mcount was called.
 */
#define __naked			            __attribute__((naked)) notrace
#define __compiler_offsetof(a, b)	__builtin_offsetof(a, b)

/*
 * Feature detection for gnu_inline (gnu89 extern inline semantics). Either
 * __GNUC_STDC_INLINE__ is defined (not using gnu89 extern inline semantics,
 * and we opt in to the gnu89 semantics), or __GNUC_STDC_INLINE__ is not
 * defined so the gnu89 semantics are the default.
 */
#ifdef __GNUC_STDC_INLINE__
# define __gnu_inline	__attribute__((gnu_inline))
#else
# define __gnu_inline
#endif

/*
 * Force always-inline if the user requests it so via the .config.
 * GCC does not warn about unused static inline functions for
 * -Wunused-function.  This turns out to avoid the need for complex #ifdef
 * directives.  Suppress the warning in clang as well by using "unused"
 * function attribute, which is redundant but not harmful for gcc.
 * Prefer gnu_inline, so that extern inline functions do not emit an
 * externally visible function. This makes extern inline behave as per gnu89
 * semantics rather than c99. This prevents multiple symbol definition errors
 * of extern inline functions at link time.
 * A lot of inline functions can cause havoc with function tracing.
 */

#define __inline__ inline
#define __inline   inline
#define noinline	__attribute__((noinline))

#ifndef __always_inline
#define __always_inline inline __attribute__((always_inline))
#endif

/*
 * Rather then using noinline to prevent stack consumption, use
 * noinline_for_stack instead.  For documentation reasons.
 */
#define noinline_for_stack noinline

#endif /* COMPONENT_COMPILER_TYPES_H_ */
