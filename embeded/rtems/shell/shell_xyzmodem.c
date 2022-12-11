/*
 * Copyright 2022 wtcat
 */
#include "configs/rtems_confdefs.h"

#ifdef CONFIGURE_SHELL_COMMAND_XYZMODEM
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#define __need_getopt_newlib
#include <getopt.h>

#include "shell/shell_utils.h"
#include "base/ymodem.h"
#include "base/log.h"

static const char ymodem_usage[] = {
	"ry [-d device] [-o offset] [filepath]\n"
	"options:\n"
	"  @ Ymodem file receive protocol\n"
	"  -d serial device\n"
	"  -o file offset\n"
};

static int shell_cmd_ry(int argc, char **argv) {
	struct getopt_data getopt_reent;
	const char *dev = "/dev/console";
	char path_buffer[128];
	const char *path = NULL;
	off_t ofs = 0;
	int ch;

	memset(&getopt_reent, 0, sizeof(getopt_data));


	while ((ch = getopt_r(argc, argv, "d:o:", &getopt_reent)) != -1) {
		switch (ch) {
		case 'd':
			dev = getopt_reent.optarg;
			break;
		case 'o':
			ofs = strtol(getopt_reent.optarg, NULL, 16);
			break;
		default:
			printf("invalid option (-%c)\n", ch);
			return -EINVAL;
		}
	}
	if (argc > 1) {
		if (getopt_reent.optind == 0) {
			printf("inalid command format\n");
			return -EINVAL;
		}
		path = argv[getopt_reent.optind];
		if (path[0] != '/') {
			size_t slen, cwdlen;

			getcwd(path_buffer, sizeof(path_buffer));
			cwdlen = strlen(path_buffer);
			slen = strlen(path);
			if (cwdlen + slen + 1>= sizeof(path_buffer)) {
				printf("file name is too long\n");
				return -EINVAL;
			}
			strcat(path_buffer, "/");
			path = strcat(path_buffer, path);
		}
	} else {
		path = getcwd(path_buffer, sizeof(path_buffer));
	}

	return rym_download_file(dev, path, ofs);
}


SHELL_CMDS_DEFINE(yzmodem_cmds,
	{
		.name = "ry",
		.usage = ymodem_usage,
		.topic = "misc",
		.command = shell_cmd_ry
	}
);
#endif /* CONFIGURE_SHELL_COMMAND_XYZMODEM */
