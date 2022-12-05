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
#include "base/xyz_modem.h"

static const char xyzmodem_usage[] = {
	"xyz -[x|y|z] -f <filename>\n" 
	"options:\n"
	"  @ XYZ Modem file transmit protocol\n"
	"  -x|y|z protocol type\n"
	"  -f receive file name\n"
};

static int xyzmodem_transmit(const char *filename, connection_info_t *conn) {
    char mbuffer[1024];
    int fd, len, err;

    fd = open(filename, O_CREAT | O_TRUNC | O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);
    if (fd < 0) {
        fprintf(stderr, "%s: open %s failed\n", __func__, filename);
        return -ENOENT;
    }
    if (xyzModem_stream_open(conn, &err)) {
        fprintf(stderr, "Open xyz-modem device failed(%s)\n", xyzModem_error(err));
        close(fd);
        return -EIO;
    }
    while ((len = xyzModem_stream_read(mbuffer, 1024, &err)) > 0) {
        int ret = write(fd, mbuffer, len);
        if (ret != len) {
            fprintf(stderr, "Write file %s failed\n", filename);
            return -EIO;
        }
    }
    close(fd);
    xyzModem_stream_close(&err);
    xyzModem_stream_terminate(false, NULL);
    return 0;
}

static int shell_cmd_xyzmodem(int argc, char **argv) {
	struct getopt_data getopt_reent;
    connection_info_t conn;
	const char *filename = NULL;
	int ch;

	memset(&getopt_reent, 0, sizeof(getopt_data));
    conn.mode = 0;
	while ((ch = getopt_r(argc, argv, "f:xyz", &getopt_reent)) != -1) {
		switch (ch) {
		case 'f':
			filename = getopt_reent.optarg;
			break;
		case 'x':
			conn.mode = xyzModem_xmodem;
			break;
		case 'y':
			conn.mode = xyzModem_ymodem;
			break;
		case 'z':
		default:
			puts("Invalid format or paramters\n");
			return -EINVAL;
		}
	}
	if (!filename || conn.mode == 0) {
		fprintf(stderr, "Invalid format or paramters\n");
		return -EINVAL;
	}
	return xyzmodem_transmit(filename, &conn);
}

SHELL_CMDS_DEFINE(xyzmodem_cmds,
	{
		.name = "xyzmodem",
		.usage = xyzmodem_usage,
		.topic = "misc",
		.command = shell_cmd_xyzmodem
	}
);
#endif /* CONFIGURE_SHELL_COMMAND_XYZMODEM */
