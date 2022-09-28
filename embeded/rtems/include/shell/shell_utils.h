/*
 * Copyright 2022 wtcat
 */
#ifndef SHELL_UTILS_H_
#define SHELL_UTILS_H_

#include <rtems/sysinit.h>
#include <rtems/shell.h>

#ifdef __cplusplus
extern "C"{
#endif

#define SHELL_CMD_TERMINAL {NULL, NULL, NULL, NULL, NULL, NULL, 0, 0, 0}
#define SHELL_CMDS_DEFINE(_name) \
    static rtems_shell_cmd_t cmds_##_name[]; \
	static void _shell_cmds_register_##_name(void) { \
		rtems_shell_cmd_t *cmd = cmds_##_name; \
		while (cmd->command) \
			rtems_shell_add_cmd_struct(cmd++); \
	} \
	RTEMS_SYSINIT_ITEM(_shell_cmds_register_##_name, \
		RTEMS_SYSINIT_STD_FILE_DESCRIPTORS, \
		RTEMS_SYSINIT_ORDER_LAST); \
    static rtems_shell_cmd_t cmds_##_name[] = 


#ifdef __cplusplus
}
#endif
#endif /* SHELL_UTILS_H_ */
