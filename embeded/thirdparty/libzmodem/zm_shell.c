/*
 * Copyright 2022 wtcat
 */
#include "zm_namespace.h"

#include <errno.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#define __need_getopt_newlib
#include <getopt.h>

#include "shell/shell_utils.h"
#include "base/log.h"
#include "zmodem.h"

static const char zmodem_usage[] = {
	"rz [-d device] [-o offset] [filepath]\n"
	"options:\n"
	"  @ Zmodem file receive protocol\n"
	"  -d serial device\n"
	"  -o file offset\n"
};


static bool approver_cb(const char *filename, size_t size, 
    time_t date) {
    (void) date;
    pr_dbg("Sender requests to send %s: %zu bytes\n", filename, size);
    return true;
}

static bool tick_cb(const char *fname, long bytes_sent, long bytes_total, 
    long last_bps, int min_left, int sec_left) {
    static long last_sec_left = 0;
    if (last_sec_left != sec_left && sec_left != 0) {
        fprintf(stderr, "%s: Bytes Received:%7ld/%7ld   BPS:%-8ld ETA %02d:%02d\n",
            fname, bytes_sent, bytes_total,
            last_bps, min_left, sec_left);
        last_sec_left = sec_left;
    }
    usleep(10000);
    return true;
}

static void complete_cb(const char *filename, int result, size_t size, time_t date) {
  if (result == RZSZ_NO_ERROR)
    printf("'%s': received\n", filename);
  else
    printf("'%s': failed to receive\n", filename);
}

static int shell_cmd_rz(int argc, char **argv) {
	struct getopt_data getopt_reent;
	const char *dev = "/dev/console";
	char path_buffer[128];
	const char *path = NULL;
    uint32_t bps = 0;
	off_t ofs = 0;
    size_t bytes;
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

    (void) ofs;
    (void) dev;
    bytes = zmodem_receive(NULL, /* use current directory */
        approver_cb, /* receive everything */
        tick_cb,
        complete_cb,
        bps,
        RZSZ_FLAGS_NONE);
    printf("Received %zu bytes.\n", bytes);
    return 0;
}

SHELL_CMDS_DEFINE(zmodem_cmds,
	{
		.name = "rz",
		.usage = zmodem_usage,
		.topic = "misc",
		.command = shell_cmd_rz
	}
);
