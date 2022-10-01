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


struct shell_arg {
	char devname[32];
	int prio;
	size_t stksz;
	rtems_shell_login_check_t login;
};

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
static struct shell_arg *shell_env;
static bool shell_ready;

static int shell_cmd_main(int argc, char *argv[]) {
	struct getopt_data getopt_reent;
    const char *dev = "/dev/console";
	int ch;

    rtems_mutex_lock(&shell_mutex);
	if (shell_ready)
		goto _out;
	shell_env = calloc(1, sizeof(*shell_env));
	if (!shell_env) {
		rtems_mutex_unlock(&shell_mutex);
		return -ENOMEM;
	}
    memset(&getopt_reent, 0, sizeof(getopt_data));
	while ((ch = getopt_r(argc, argv, "p:s:d:l", &getopt_reent)) != -1) {
		switch (ch) {
		case 'p':
			shell_env->prio = (int)strtoul(getopt_reent.optarg, NULL, 10);
			break;
		case 's':
			shell_env->stksz = (size_t)strtoul(getopt_reent.optarg, NULL, 10);
			break;
		case 'd':
			dev = getopt_reent.optarg;
			break;
		case 'l':
            shell_env->login = rtems_shell_login_check;
			break;
		default:
			goto _inv_err;
		}
	}
	if (strncmp("/dev/", dev, 5) || 
		!shell_env->prio || 
		shell_env->stksz < 4096)
		goto _inv_err;
	strncpy(shell_env->devname, dev, sizeof(shell_env->devname)-1);
	shell_ready = true;
_out:
    rtems_mutex_unlock(&shell_mutex);
    return 0;

_inv_err:
	if (shell_env) {
		free(shell_env);
		shell_env = NULL;
	}
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

void _shell_init(void) {
	rtems_status_code sc;
	if (!shell_env) {
		printf("Error***: no shell argument!\n");
		return;
	}
    sc = rtems_shell_init("root", shell_env->stksz, shell_env->prio,
        shell_env->devname, false, false, shell_env->login);
    if (sc != RTEMS_SUCCESSFUL) {
        printf("Error***: Create shell failed(%s)\n", rtems_status_text(sc));
        rtems_mutex_unlock(&shell_mutex);
        return;
    }
	free(shell_env);
	shell_env = NULL;
}

#include <rtems/shellconfig.h>
