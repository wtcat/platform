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

/* Create and register shell commands */
#define SHELL_CMDS_DEFINE(_name, ...) \
	static void _shell_cmds_register_##_name(void) { \
		static rtems_shell_cmd_t _cmds_##_name[] = { __VA_ARGS__ }; \
		for (unsigned int _i = 0; _i < RTEMS_ARRAY_SIZE(_cmds_##_name); _i++) \
			rtems_shell_add_cmd_struct(&_cmds_##_name[_i]); \
	} \
	_SHELL_INIT(_shell_cmds_register_##_name)

/* Register shell commands */
#define SHELL_CMDS_REGISTER(_name, ...) \
	static void __shell_cmds_register_##_name(void) { \
		rtems_shell_cmd_t *_cmds_##_name[] = { __VA_ARGS__ }; \
		for (unsigned int _i = 0; _i < RTEMS_ARRAY_SIZE(_cmds_##_name); _i++) \
			rtems_shell_add_cmd_struct(_cmds_##_name[_i]); \
	} \
	_SHELL_INIT(__shell_cmds_register_##_name)

#define _SHELL_INIT(_init_fn) \
	RTEMS_SYSINIT_ITEM(_init_fn, \
		RTEMS_SYSINIT_STD_FILE_DESCRIPTORS, \
		RTEMS_SYSINIT_ORDER_LAST)


static inline int shell_run_script(const char *file) {
    char *stcript[1] = { (char *)file };
    return rtems_shell_script_file(1, stcript);
}

#ifdef __cplusplus
}
#endif
#endif /* SHELL_UTILS_H_ */
