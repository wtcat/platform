/*
 * Copyright 2022 wtcat
 * 
 * This is a simple log component
 */
#ifndef BASEWORK_LOG_H_
#define BASEWORK_LOG_H_

#include <stdarg.h>

#ifdef __cplusplus
extern "C"{
#endif

#define LOGLEVEL_EMERG		0	/* system is unusable */
#define LOGLEVEL_ERR		1	/* error conditions */
#define LOGLEVEL_WARNING	2	/* warning conditions */
#define LOGLEVEL_NOTICE		3	/* normal but significant condition */
#define LOGLEVEL_INFO		4	/* informational */
#define LOGLEVEL_DEBUG		5	/* debug-level messages */

#ifndef CONFIG_LOGLEVEL
#define CONFIG_LOGLEVEL   LOGLEVEL_INFO
#endif

#ifndef pr_fmt
#define pr_fmt(fmt) fmt
#endif

struct printer {
    int (*format)(void *context, const char *fmt, va_list ap);
    void *context;
};

/*
 * virt_format - Virtual format output
 * @printer: printer object
 * @fmt: format string
 * return the number of bytes that has been output
 */
static inline int virt_format(struct printer *printer, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int len = printer->format(printer->context, fmt, ap);
    va_end(ap);
    return len;
}

extern struct printer *__log_default_printer;
extern struct printer __log_printer[];

/*
 * Log handles
 */
#define klog (&__log_printer[0]) /* for kernel and driver */
#define ulog (&__log_printer[1]) /* for application */
#define flog (&__log_printer[2]) /* file log */

/**
 * __pr_generic - Print an generic message
 * @printer: printer object
 * @level: log output level
 * @fmt: format string
 * @...: arguments for the format string
 *
 * This macro expands to a virt_format with loglevel. It uses pr_fmt() to
 * generate the format string.
 */
#ifndef _MSC_VER
#define __pr_generic(printer, level, fmt, ...) \
({ \
    (CONFIG_LOGLEVEL >= (level))? \
	    virt_format((printer), pr_fmt(fmt), ##__VA_ARGS__): 0;\
})

#else /* _MSC_VER */
static inline int __pr_generic(struct printer *printer, int level, 
    const char *fmt, ...) {
    if (level <= CONFIG_LOGLEVEL) {
        va_list ap;
        va_start(ap, fmt);
        int len = printer->format(printer->context, fmt, ap);
        va_end(ap);
        return len;
    }
    return 0;
}
#endif /* !_MSC_VER */

/**
 * pr_emerg - Print an emergency-level message
 * @fmt: format string
 * @...: arguments for the format string
 *
 * This macro expands to a virt_format with LOGLEVEL_EMERG loglevel. It uses pr_fmt() to
 * generate the format string.
 */
#define pr_emerg(fmt, ...) \
    __pr_generic(__log_default_printer, LOGLEVEL_EMERG, fmt, ##__VA_ARGS__)
#define npr_emerg(printer, fmt, ...) \
    __pr_generic((printer), LOGLEVEL_EMERG, fmt, ##__VA_ARGS__)

/**
 * pr_err - Print an error-level message
 * @fmt: format string
 * @...: arguments for the format string
 *
 * This macro expands to a virt_format with LOGLEVEL_ERR loglevel. It uses pr_fmt() to
 * generate the format string.
 */
#define pr_err(fmt, ...) \
    __pr_generic(__log_default_printer, LOGLEVEL_ERR, fmt, ##__VA_ARGS__)
#define npr_err(printer, fmt, ...) \
    __pr_generic((printer), LOGLEVEL_ERR, fmt, ##__VA_ARGS__)

/**
 * pr_warn - Print a warning-level message
 * @fmt: format string
 * @...: arguments for the format string
 *
 * This macro expands to a virt_format with LOGLEVEL_WARNING loglevel. It uses pr_fmt()
 * to generate the format string.
 */
#define pr_warn(fmt, ...) \
    __pr_generic(__log_default_printer, LOGLEVEL_WARNING, fmt, ##__VA_ARGS__)
#define npr_warn(printer, fmt, ...) \
    __pr_generic((printer), LOGLEVEL_WARNING, fmt, ##__VA_ARGS__)

/**
 * pr_notice - Print a notice-level message
 * @fmt: format string
 * @...: arguments for the format string
 *
 * This macro expands to a virt_format with LOGLEVEL_NOTICE loglevel. It uses pr_fmt() to
 * generate the format string.
 */
#define pr_notice(fmt, ...) \
    __pr_generic(__log_default_printer, LOGLEVEL_NOTICE, fmt, ##__VA_ARGS__)
#define npr_notice(printer, fmt, ...) \
    __pr_generic((printer), LOGLEVEL_NOTICE, fmt, ##__VA_ARGS__)

/**
 * pr_info - Print an info-level message
 * @fmt: format string
 * @...: arguments for the format string
 *
 * This macro expands to a virt_format with LOGLEVEL_INFO loglevel. It uses pr_fmt() to
 * generate the format string.
 */
#define pr_info(fmt, ...) \
    __pr_generic(__log_default_printer, LOGLEVEL_INFO, fmt, ##__VA_ARGS__)
#define npr_info(printer, fmt, ...) \
    __pr_generic((printer), LOGLEVEL_INFO, fmt, ##__VA_ARGS__)

/**
 * pr_info - Print an debug-level message
 * @fmt: format string
 * @...: arguments for the format string
 *
 * This macro expands to a virt_format with LOGLEVEL_DEBUG loglevel. It uses pr_fmt() to
 * generate the format string.
 */
#define pr_dbg(fmt, ...) \
    __pr_generic(__log_default_printer, LOGLEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define npr_dbg(printer, fmt, ...) \
    __pr_generic((printer), LOGLEVEL_DEBUG, fmt, ##__VA_ARGS__)

/*
 * log_init - Initialize log component
 * @pr: default log printer
 * return 0 if success
 */
static inline int pr_log_init(struct printer *pr) {
    if (pr && pr->format) {
        __log_default_printer = pr;
        return 0;
    }
    return -1;
}

#ifdef __cplusplus
}
#endif
#endif /* BASEWORK_LOG_H_ */