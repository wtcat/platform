/*
 * Copyright 2022 wtcat
 */
#include "bsp/board/shellconf.h"

#ifdef CONFIGURE_SHELL_COMMAND_DISK
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#define __need_getopt_newlib
#include <getopt.h>

#include <rtems/blkdev.h>
#include <rtems/shell.h>

static void fdisk_usage(void) {
	static const char usage[] = {
		"fdisk -d <dev> -p <log_dev> -s <start_blk> -n <blks>\n" 
		"Options:\n"
		" -d physical device\n"
		" -p logic device\n"
		" -s start media block\n"
		" -n block numbers\n"
	};
	puts(usage);
}

static int shell_cmd_fdisk(int argc, char **argv) {
	struct getopt_data getopt_reent;
	char *parent = NULL;
    char *partition = NULL;
	unsigned long start = -1, nblks = 0;
	uint32_t media_blksz = 0;
	rtems_status_code sc;
	int ch;

	memset(&getopt_reent, 0, sizeof(getopt_data));
	while ((ch = getopt_r(argc, argv, "d:p:s:n:", &getopt_reent)) != -1) {
		switch (ch) {
		case 'd':
			parent = getopt_reent.optarg;
			break;
		case 'p':
			partition = getopt_reent.optarg;
			break;
		case 's':
			start = strtoul(getopt_reent.optarg, NULL, 10);
			break;
		case 'n':
			nblks = strtoul(getopt_reent.optarg, NULL, 10);
			break;
		default:
			fdisk_usage();
			return -EINVAL;
		}
	}
	if (!parent || !partition || 
		start == (unsigned long)-1 || nblks == 0) {
		fprintf(stderr, "Invalid format\n");
		return -EINVAL;
	}
	if (strncmp("/dev/", parent, 5) || 
		strncmp("/dev/", partition, 5)) {
		fprintf(stderr, "Invalid device name\n");
		return -EINVAL;
	}
	sc = rtems_blkdev_create_partition(partition, parent, start, nblks);
	if (sc != RTEMS_SUCCESSFUL) {
		fprintf(stderr, "Create partition(%s) failed(%s)\n", 
			partition, rtems_status_text(sc));
		return -1;
	}
	int fd = open(parent, O_RDONLY);
	if (fd < 0) {
		fprintf(stderr, "Open (%s) failed\n", parent);
		return -ENXIO;
	}
	rtems_disk_fd_get_media_block_size(fd, &media_blksz);
	close(fd);
	printf("Created paritition: %s[%lu, %lu] -> %s(BlockSize: %d)\n", parent, start, 
		start+nblks, partition, media_blksz);
	return 0;
}

rtems_shell_cmd_t shell_fdisk_command = {
    "fdisk",
    "Disk partition tool",
    "file",
    shell_cmd_fdisk
};
#endif /* CONFIGURE_SHELL_COMMAND_DISK */
