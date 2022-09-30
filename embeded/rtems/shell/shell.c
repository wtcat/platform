/*
 * CopyRight 2022 wtcat
 */
#include "bsp/board/sysconf.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#define __need_getopt_newlib
#include <getopt.h>
#include <rtems/thread.h>

#include "shell/shell_utils.h"


static const char shell_usage[] = {
	"shell -d <dev> -p <prio> -s <stacksize> [-l]\n" 
	"options:\n"
	"  @ create shell console\n"
	"  -d shell device\n"
	"  -p shell task priority\n"
	"  -s shell stack size\n"
	"  -l login check\n"
};
static rtems_mutex shell_mutex = RTEMS_MUTEX_INITIALIZER("shell");
static bool shell_ready;

static int shell_cmd_main(int argc, char *argv[]) {
	struct getopt_data getopt_reent;
    rtems_status_code sc;
    rtems_shell_login_check_t login = NULL;
    const char *dev = "/dev/console";
    size_t stk_size = 0;
    int prio = -1;
	int ch;

    rtems_mutex_lock(&shell_mutex);
    if (shell_ready)
        goto _out;
    memset(&getopt_reent, 0, sizeof(getopt_data));
	while ((ch = getopt_r(argc, argv, "p:s:d:l", &getopt_reent)) != -1) {
		switch (ch) {
		case 'p':
			prio = (int)strtoul(getopt_reent.optarg, NULL, 10);
			break;
		case 's':
			stk_size = (size_t)strtoul(getopt_reent.optarg, NULL, 10);
			break;
		case 'd':
			dev = getopt_reent.optarg;
			break;
		case 'l':
            login = rtems_shell_login_check;
			break;
		default:
			goto _inv_err;
		}
	}
    if (strncmp("/dev/", dev, 5) || prio < 1 || stk_size < 4096)
        goto _inv_err;
    sc = rtems_shell_init("shel", stk_size, prio,
        dev, false, false, login);
    if (sc != RTEMS_SUCCESSFUL) {
        fprintf(stderr, "Error***: Create shell failed(%s)\n", rtems_status_text(sc));
        rtems_mutex_unlock(&shell_mutex);
        return -rtems_status_code_to_errno(sc);
    }
    shell_ready = true;
_out:
    rtems_mutex_unlock(&shell_mutex);
    return 0;

_inv_err:
    rtems_mutex_unlock(&shell_mutex);
	fprintf(stderr, "Invalid format or parameters\n");
	return -EINVAL;
}

SHELL_CMDS_DEFINE(console_cmds,
	{
		.name = "shell",
		.usage = shell_usage,
		.topic = "rtems",
		.command = shell_cmd_main
	}
);

#include <rtems/shellconfig.h>
