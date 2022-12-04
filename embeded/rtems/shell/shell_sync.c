/*
 * Copyright 2022 wtcat
 */
#include "configs/rtems_confdefs.h"

#ifdef CONFIGURE_SHELL_COMMAND_SYNC
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include <rtems.h>
#include <rtems/shell.h>

#include "shell/shell_utils.h"


static int shell_cmd_sync(int argc, char *argv[]) {
    (void) argv;
    if (argc > 1) {
        printf("error: too many arguments!\n");
        return -EINVAL;
    }
    sync();
    return 0;
}

SHELL_CMDS_DEFINE(sync_cmds,
	{
		.name = "sync",
		.usage = "sync -- flush all files to disk",
		.topic = "misc",
		.command = shell_cmd_sync
	}
);
#endif /* CONFIGURE_SHELL_COMMAND_SYNC */
