/*
 * CopyRight 2022 wtcat
 */
#include "configs/rtems_confdefs.h"

#include <rtems/shell.h>


#ifndef CONFIGURE_SHELL_PRIO
#define CONFIGURE_SHELL_PRIO (CONFIGURE_MAXIMUM_PRIORITY / 2)
#endif
#ifndef CONFIGURE_SHELL_STKSIZE
# ifdef __rtems_libbsd__
# define CONFIGURE_SHELL_STKSIZE (16*1024)
# else
# define CONFIGURE_SHELL_STKSIZE 4096
# endif
#endif

void _shell_init(void) {
	rtems_status_code sc;

    sc = rtems_shell_init(
		"root", 
		CONFIGURE_SHELL_STKSIZE, 
		CONFIGURE_SHELL_PRIO,
		"/dev/console", 
		false, 
		false, 
	#ifdef CONFIGURE_SHELL_LOGIN
		rtems_shell_login_check
	#else
		NULL
	#endif
	);
    if (sc != RTEMS_SUCCESSFUL)
        printf("Error***: Create shell failed(%s)\n", rtems_status_text(sc));
}
