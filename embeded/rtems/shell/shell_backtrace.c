/*
 * Copyright 2022 wtcat
 */

#include "bsp/board/shellconf.h"

#if defined(CONFIGURE_SHELL_COMMANDS_BT) && \
	defined(CONFIGURE_BACKTRACE)
#include <stdlib.h>
#include <rtems/score/threadimpl.h>
#include <rtems/printer.h>
#include "bsp/asm/unwind.h"

#include "shell/shell_utils.h"

static int shell_cmd_backtrace(int argc, char **argv) {
	if (argc <= 2) {
		rtems_printer printer;
		rtems_print_printer_printf(&printer);
		if (argc == 1) {
			__stack_backtrace(&printer, NULL, NULL);
			return 0;
		} else {
			ISR_lock_Context lock_context;
			Thread_Control *the_thread;
			rtems_id id = (rtems_id)strtoul(argv[1], NULL, 16);
			the_thread = _Thread_Get(id, &lock_context);
			_ISR_lock_ISR_enable(&lock_context);
			if (the_thread) {
				__stack_backtrace(&printer, NULL, the_thread);
				return 0;
			} else {
				printf("Error: Invalid object id!\n");
			}
		}
	}
	return -EINVAL;
}

SHELL_CMDS_DEFINE(backtrace_cmds) {
	{
		.name = "bt",
		.usage = "bt [object ID]",
		.topic = "misc",
		.command = shell_cmd_backtrace
	},
    SHELL_CMD_TERMINAL
};
#endif //CONFIGURE_SHELL_COMMANDS_BT && CONFIGURE_BACKTRACE
