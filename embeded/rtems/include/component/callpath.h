/*
 * Copyright (c) 2022 wtcat
 */

#ifndef COMPONENT_CALLPATH_H_
#define COMPONENT_CALLPATH_H_

#ifdef __cplusplus
extern "C"{
#endif

#if defined(__rtems__)
#include <rtems/printer.h>
typedef struct rtems_printer callpath_printer_t;
#else
#include <stdarg.h>
typedef struct {
  void *context;
  int  (*printer)(void *, const char *format, va_list ap);
} callpath_printer_t;
#endif

int callpath_print(void *thread, const callpath_printer_t *printer);
int callpath_print_current(const callpath_printer_t *printer);

#ifdef __cplusplus
}
#endif
#endif /* COMPONENT_CALLPATH_H_ */
