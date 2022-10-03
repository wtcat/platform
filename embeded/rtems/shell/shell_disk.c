/*
 * Copyright 2022 wtcat
 */
#include "configs/rtems_confdefs.h"

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
#include <rtems/ramdisk.h>

#include "shell/shell_utils.h"


static const char disk_usage[] = {
	"fdisk -d <dev> -p <log_dev> -s <start_blk> -n <blks>\n" 
	"options:\n"
	"  @ create logic-disk from phsical disk\n"
	"  -d physical device\n"
	"  -p logic device\n"
	"  -s start media block\n"
	"  -n block numbers\n"
};

static const char ramdisk_usage[] = {
	"mkrd -d <dev> -s <blksize> -n <blks>\n" 
	"options:\n"
	"  @ create ramdisk\n"
	"  -d device name\n"
	"  -s block size\n"
	"  -n block numbers\n"
};

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
			goto _inv_err;
		}
	}
	if (!parent || !partition || 
		start == (unsigned long)-1 || nblks == 0) {
		goto _inv_err;
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
	printf("created paritition: %s[%lu, %lu] -> %s(BlockSize: %d)\n", parent, start, 
		start+nblks, partition, media_blksz);
	return 0;
_inv_err:
	fprintf(stderr, "Invalid format\n");
	return -EINVAL;
}

static int shell_cmd_ramdisk(int argc, char **argv) {
	struct getopt_data getopt_reent;
	char *devname = NULL;
	unsigned long blksz = 0, nblks = 0;
	rtems_status_code sc;
	int ch;

	memset(&getopt_reent, 0, sizeof(getopt_data));
	while ((ch = getopt_r(argc, argv, "d:s:n:", &getopt_reent)) != -1) {
		switch (ch) {
		case 'd':
			devname = getopt_reent.optarg;
			break;
		case 's':
			blksz = strtoul(getopt_reent.optarg, NULL, 10);
			break;
		case 'n':
			nblks = strtoul(getopt_reent.optarg, NULL, 10);
			break;
		default:
			puts("Invalid format or paramters\n");
			return -EINVAL;
		}
	}
	if (!devname || blksz == 0 || nblks == 0) {
		fprintf(stderr, "Invalid format or paramters\n");
		return -EINVAL;
	}
	if (strncmp("/dev/", devname, 5)) {
		fprintf(stderr, "Invalid device name(%s)\n", devname);
		return -EINVAL;
	}
	sc = ramdisk_register(blksz, nblks, false, devname);
	if (sc != RTEMS_SUCCESSFUL) {
		fprintf(stderr, "Create ramdisk(%s) failed(%s)\n", 
			devname, rtems_status_text(sc));
		return -rtems_status_code_to_errno(sc);
	}
	printf("created ramdisk: %s size(0x%lx)\n", devname, blksz*nblks);
	return 0;
}

SHELL_CMDS_DEFINE(disk_cmds,
	{
		.name = "fdisk",
		.usage = disk_usage,
		.topic = "files",
		.command = shell_cmd_fdisk
	},
	{
		.name = "mkrd",
		.usage = ramdisk_usage,
		.topic = "files",
		.command = shell_cmd_ramdisk
	}
);

#endif /* CONFIGURE_SHELL_COMMAND_DISK */
