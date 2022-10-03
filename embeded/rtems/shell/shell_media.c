/*
 * Copyright 2022 wtcat
 */
#include "configs/rtems_confdefs.h"

#ifdef CONFIGURE_SHELL_COMMAND_DISK
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#define __need_getopt_newlib
#include <getopt.h>

#include <rtems/media.h>
#include "shell/shell_utils.h"


static const char media_usage[] = {
	"media -p <prio> -s <stacksize>\n" 
	"options:\n"
	"  @ create media server\n"
	"  -p server priority\n"
	"  -s server stack size\n"
};

static int shell_cmd_media(int argc, char **argv) {
	struct getopt_data getopt_reent;
    rtems_status_code sc;
    size_t stk_size = 0;
    int prio = -1;
	int ch;

	memset(&getopt_reent, 0, sizeof(getopt_data));
	while ((ch = getopt_r(argc, argv, "p:s:", &getopt_reent)) != -1) {
		switch (ch) {
		case 'p':
			prio = strtoul(getopt_reent.optarg, NULL, 10);
			break;
		case 's':
			stk_size = strtoul(getopt_reent.optarg, NULL, 10);
			break;
		default:
            goto _inv_err;
		}
	}

    if (prio < 1 || !stk_size) 
        goto _inv_err;
    rtems_media_initialize();
    sc = rtems_media_server_initialize(prio, stk_size,
        RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES);
    if (sc != RTEMS_SUCCESSFUL) {
        fprintf(stderr, "create media server failed(%s)\n", rtems_status_text(sc));
        return -rtems_status_code_to_errno(sc);
    }
    return 0;

_inv_err:
	fprintf(stderr, "Invalid format or parameters\n");
	return -EINVAL;
}

SHELL_CMDS_DEFINE(media_cmds,
	{
		.name = "media",
		.usage = media_usage,
		.topic = "misc",
		.command = shell_cmd_media
	}
);
#endif /* CONFIGURE_SHELL_COMMAND_DISK */
