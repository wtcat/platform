/*
 * Copyright 2022 wtcat
 */
#include "configs/rtems_confdefs.h"

#if defined(CONFIGURE_SHELL_COMMAND_RAP) || \
    defined(CONFIGURE_SHELL_COMMAND_RTL)
#include "shell/shell_utils.h"

extern int shell_rap (int argc, char* argv[]);
extern int rtems_rtl_shell_command (int argc, char* argv[]);

SHELL_CMDS_DEFINE(dl_cmds,
#ifdef CONFIGURE_SHELL_COMMAND_RAP
{
        .name     = "rap",
        .usage   = "Runtime load",
        .topic   = "rtems",
        .command = shell_rap,
    },
#endif /* CONFIGURE_SHELL_COMMAND_RAP */
#ifdef CONFIGURE_SHELL_COMMAND_RTL
    {
        .name     = "rtl",
        .usage   = "Runtime Linker",
        .topic   = "rtems",
        .command = rtems_rtl_shell_command,  
    }
#endif /* CONFIGURE_SHELL_COMMAND_RTL */
);

#endif /* */
