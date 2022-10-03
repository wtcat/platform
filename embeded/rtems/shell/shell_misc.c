/*
 * Copyright 2022 wtcat
 */
#include "configs/rtems_confdefs.h"

#include <errno.h>
#include <bsp/irq-info.h>

#include "shell/shell_vt100.h"
#include "shell/shell_utils.h"


/* Macro to send VT100 commands. */
#define SHELL_VT100_CMD(_cmd_) \
    do {				   \
        static const char cmd[] = _cmd_; \
        fprintf(stdout, "%s", cmd);	\
    } while (0)

static int shell_main_clear(int argc, char *argv[]) {
    (void) argv;
    if (argc > 1)
        return -EINVAL;
    SHELL_VT100_CMD(SHELL_VT100_CURSORHOME);
    SHELL_VT100_CMD(SHELL_VT100_CLEARSCREEN);
    return 0;
}

static int shell_main_reboot(int argc, char *argv[]) {
    extern void bsp_reset(void);
    (void) argv;
    if (argc > 1)
        return -EINVAL;
    bsp_reset();
    return 0;
}

SHELL_CMDS_DEFINE(misc_cmds,
    {
        .name = "clear",                         /* name */
        .usage = "clear     # Clear screen",      /* usage */
        .topic = "rtems",                         /* topic */
        .command = shell_main_clear                /* command */
    },
{
    .name = "reboot",                                   /* name */
    .usage = "Reboot system immidateliy",                /* usage */
    .topic = "rtems",                                    /* topic */
    .command = shell_main_reboot                          /* command */
    }
);

#ifdef CONFIGURE_SHELL_COMMAND_IRQ
SHELL_CMDS_REGISTER(_misc, 
    &bsp_interrupt_shell_command
);
#endif /* CONFIGURE_SHELL_COMMAND_IRQ */
