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

#define SHELL_CMDS_DEFINE(_name, ...) \
	static void _shell_cmds_register_##_name(void) { \
		static rtems_shell_cmd_t _cmds_##_name[] = { __VA_ARGS__ }; \
		for (unsigned int _i = 0; _i < RTEMS_ARRAY_SIZE(_cmds_##_name); _i++) \
			rtems_shell_add_cmd_struct(&_cmds_##_name[_i]); \
	} \
	RTEMS_SYSINIT_ITEM(_shell_cmds_register_##_name, \
		RTEMS_SYSINIT_STD_FILE_DESCRIPTORS, \
		RTEMS_SYSINIT_ORDER_LAST)


#ifdef __cplusplus
}
#endif
#endif /* SHELL_UTILS_H_ */
