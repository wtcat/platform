/*
 * Copyright 2022 wtcat
 */
#ifndef BASE_GENERIC_H_
#define BASE_GENERIC_H_

#include <sys/types.h>
#include <sys/param.h>

#include <rtems/score/basedefs.h>
#include <rtems/counter.h>

#include "base/compiler.h"
#include "base/bitops.h"
#include "base/byteorder.h"
#include "base/ilog2.h"
#include "base/minmax.h"

#ifdef __cplusplus
extern "C"{
#endif

#define BUG() rtems_panic("BUG")
#define BUG_ON(condition)	do { if (__predict_false(condition)) BUG(); } while(0)
#define	WARN_ON(condition)	({ int _warn_on = !!(condition); \
    __predict_false(_warn_on); })

#undef	ALIGN
#define	ALIGN(x, y)		roundup2((x), (y))
#undef PTR_ALIGN
#define	PTR_ALIGN(p, a)		((__typeof(p))ALIGN((uintptr_t)(p), (a)))
#define	DIV_ROUND_UP(x, n)	howmany(x, n)
#define	DIV_ROUND_UP_ULL(x, n)	DIV_ROUND_UP((unsigned long long)(x), (n))
// #define	DIV_ROUND_CLOSEST(x, divisor) (((x) + ((divisor) / 2)) / (divisor))
#define	FIELD_SIZEOF(t, f)	sizeof(((t *)0)->f)

/*
 * Divide positive or negative dividend by positive or negative divisor
 * and round to closest integer. Result is undefined for negative
 * divisors if the dividend variable type is unsigned and for negative
 * dividends if the divisor variable type is unsigned.
 */
#define DIV_ROUND_CLOSEST(x, divisor)(			\
{							\
	typeof(x) __x = x;				\
	typeof(divisor) __d = divisor;			\
	(((typeof(x))-1) > 0 ||				\
	 ((typeof(divisor))-1) > 0 ||			\
	 (((__x) > 0) == ((__d) > 0))) ?		\
		(((__x) + ((__d) / 2)) / (__d)) :	\
		(((__x) - ((__d) / 2)) / (__d));	\
}							\
)

/*
 * The "pr_debug()" and "pr_devel()" macros should produce zero code
 * unless DEBUG is defined:
 */
#ifdef DEBUG
#define pr_debug(fmt, ...) \
        log(LOG_DEBUG, fmt, ##__VA_ARGS__)
#define pr_devel(fmt, ...) \
	log(LOG_DEBUG, pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_debug(fmt, ...) \
        ({ if (0) log(LOG_DEBUG, fmt, ##__VA_ARGS__); 0; })
#define pr_devel(fmt, ...) \
	({ if (0) log(LOG_DEBUG, pr_fmt(fmt), ##__VA_ARGS__); 0; })
#endif

#ifndef pr_fmt
#define pr_fmt(fmt) fmt
#endif

/*
 * Print a one-time message (analogous to WARN_ONCE() et al):
 */
#define printk_once(...) do {			\
	static bool __print_once;		\
						\
	if (!__print_once) {			\
		__print_once = true;		\
		printk(__VA_ARGS__);		\
	}					\
} while (0)

/*
 * Log a one-time message (analogous to WARN_ONCE() et al):
 */
#define log_once(level,...) do {		\
	static bool __log_once;			\
						\
	if (!__log_once) {			\
		__log_once = true;		\
		log(level, __VA_ARGS__);	\
	}					\
} while (0)

#define pr_emerg(fmt, ...) \
	log(LOG_EMERG, pr_fmt(fmt), ##__VA_ARGS__)
#define pr_alert(fmt, ...) \
	log(LOG_ALERT, pr_fmt(fmt), ##__VA_ARGS__)
#define pr_crit(fmt, ...) \
	log(LOG_CRIT, pr_fmt(fmt), ##__VA_ARGS__)
#define pr_crit_once(fmt, ...) \
	log_once(LOG_CRIT, pr_fmt(fmt), ##__VA_ARGS__)
#define pr_err(fmt, ...) \
	log(LOG_ERR, pr_fmt(fmt), ##__VA_ARGS__)
#define pr_warning(fmt, ...) \
	log(LOG_WARNING, pr_fmt(fmt), ##__VA_ARGS__)
#define pr_warn pr_warning
#define pr_warn_once(fmt, ...) \
	log_once(LOG_WARNING, pr_fmt(fmt), ##__VA_ARGS__)
#define pr_notice(fmt, ...) \
	log(LOG_NOTICE, pr_fmt(fmt), ##__VA_ARGS__)
#define pr_info(fmt, ...) \
	log(LOG_INFO, pr_fmt(fmt), ##__VA_ARGS__)
#define pr_info_once(fmt, ...) \
	log_once(LOG_INFO, pr_fmt(fmt), ##__VA_ARGS__)
#define pr_cont(fmt, ...) \
	printk(KERN_CONT fmt, ##__VA_ARGS__)

#ifndef WARN
#define WARN(condition, format...) ({                                   \
        int __ret_warn_on = !!(condition);                              \
        if (unlikely(__ret_warn_on))                                    \
                pr_warning(format);                                     \
        unlikely(__ret_warn_on);                                        \
})
#endif

#define container_of(ptr, type, member)	RTEMS_CONTAINER_OF(ptr, type, member)
#define	ARRAY_SIZE(x) RTEMS_ARRAY_SIZE(x)

/*
 * This looks more complex than it should be. But we need to
 * get the type for the ~ right in round_down (it needs to be
 * as wide as the result!), and we want to evaluate the macro
 * arguments just once each.
 */
#define __round_mask(x, y) ((__typeof__(x))((y) - 1))
#define round_up(x, y)     ((((x) - 1) | __round_mask(x, y)) + 1)
#define round_down(x, y)   ((x) & ~__round_mask(x, y))

static inline uintmax_t
mult_frac(uintmax_t x, uintmax_t multiplier, uintmax_t divisor) {
	uintmax_t q = (x / divisor);
	uintmax_t r = (x % divisor);
	return ((q * multiplier) + ((r * multiplier) / divisor));
}

static inline void udelay(int usec) {
	uint32_t ns = 1000 * (uint32_t)usec;
	_Assert((uint32_t)usec <= UINT32_MAX / 1000);
	rtems_counter_delay_nanoseconds(ns);
}

#ifdef __cplusplus
}
#endif
#endif /* BASE_GENERIC_H_ */
