/*
 * Copyright 2022 wtcat
 */
#include <stdio.h>
#include <unistd.h>
#include <rtems/printer.h>
#include <rtems/sysinit.h>

#include "base/log.h"

#define LOG_MAX_CHAN 3

struct printer *__log_default_printer;
struct printer  __log_printer[LOG_MAX_CHAN];

static void log_init(void) {
    rtems_print_printer_printk((void *)&__log_printer[0]);
    rtems_print_printer_printf((void *)&__log_printer[1]);
    pr_log_init(&__log_printer[0]);
}

RTEMS_SYSINIT_ITEM(log_init, 
    RTEMS_SYSINIT_BSP_START, 
    RTEMS_SYSINIT_ORDER_MIDDLE);

// static void log_post(void) {
//     if (access("/var", F_OK)) {
//         mkdir("/var", 0666);
//     }
//     FILE *fp = fopen("/var/log.h", "rw");
//     if (fp != NULL)
//         rtems_print_printer_fprintf((void *)&__log_printer[2], NULL);
// }
// RTEMS_SYSINIT_ITEM(log_init, 
//     RTEMS_SYSINIT_BSP_START, 
//     RTEMS_SYSINIT_ORDER_MIDDLE);