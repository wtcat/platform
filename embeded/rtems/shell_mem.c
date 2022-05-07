/*
 * Copyright (c) 2020 Actions Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
/*
 * Copyright (c) 2022 wtcat
 *
 */

#include "bsp/sysconf.h"

#ifdef CONFIGURE_SHELL_COMMANDS_MEM
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <rtems/shell.h>
#include <rtems/printer.h>
#include <rtems/sysinit.h>

struct printer_ctx {
	rtems_printer printer;
	void (*free)(void *);
};

#define MAX_LINE_LENGTH_BYTES (64)
#define DEFAULT_LINE_LENGTH_BYTES (16)
#define DISP_LINE_LEN	16

static void printer_acquire(int argc, char **argv,
	struct printer_ctx *ctx) {
	ctx->free = NULL;
	if (argc == 5) {
		if (argv[3][0] == '>') {
			if (strncmp(argv[4], "/dev", 4)) {
				FILE *fp = fopen(argv[4], "w");
				if (fp != NULL) {
					rtems_print_printer_fprintf(&ctx->printer, fp);
					ctx->free = (void (*)(void *))fclose;
					return;
				}
			}
			printf("Error***: Invalid file(%s)\n", argv[4]);
		} else {
			printf("Error***: Invalid redirect format\n");
		}
	}
	rtems_print_printer_printf(&ctx->printer);
}

static void printer_release(struct printer_ctx *ctx) {
	if (ctx->free)
		ctx->free(&ctx->printer.context);
}

static void print_buffer(const rtems_printer *printer, const char *addr, 
	int width, int count, int linelen, unsigned long disp_addr) {
	int i, thislinelen;
	const char *data;
	/* linebuf as a union causes proper alignment */
	union linebuf {
		uint32_t ui[MAX_LINE_LENGTH_BYTES/sizeof(uint32_t) + 1];
		uint16_t us[MAX_LINE_LENGTH_BYTES/sizeof(uint16_t) + 1];
		uint8_t  uc[MAX_LINE_LENGTH_BYTES/sizeof(uint8_t) + 1];
	} lb;

	if (linelen * width > MAX_LINE_LENGTH_BYTES)
		linelen = MAX_LINE_LENGTH_BYTES / width;
	if (linelen < 1)
		linelen = DEFAULT_LINE_LENGTH_BYTES / width;

	if (disp_addr == -1)
		disp_addr = (unsigned long)addr;

	while (count) {
		thislinelen = linelen;
		data = (const char *)addr;

		rtems_printf(printer, "%08x:", (unsigned int)disp_addr);

		/* check for overflow condition */
		if (count < thislinelen)
			thislinelen = count;

		/* Copy from memory into linebuf and print hex values */
		for (i = 0; i < thislinelen; i++) {
			if (width == 4) {
				lb.ui[i] = *(volatile uint32_t *)data;
				rtems_printf(printer, " %08x", lb.ui[i]);
			} else if (width == 2) {
				lb.us[i] = *(volatile uint16_t *)data;
				rtems_printf(printer, " %04x", lb.us[i]);
			} else {
				lb.uc[i] = *(volatile uint8_t *)data;
				rtems_printf(printer, " %02x", lb.uc[i]);
			}
			data += width;
		}

		while (thislinelen < linelen) {
			/* fill line with whitespace for nice ASCII print */
			for (i = 0; i < width * 2 + 1; i++)
				rtems_printf(printer, " ");
			linelen--;
		}

		/* Print data in ASCII characters */
		for (i = 0; i < thislinelen * width; i++) {
			if (lb.uc[i] < 0x20 || lb.uc[i] > 0x7e)
				lb.uc[i] = '.';
		}
		lb.uc[i] = '\0';
		rtems_printf(printer, "    %s\n", lb.uc);

		/* update references */
		addr += thislinelen * width;
		disp_addr += thislinelen * width;
		count -= thislinelen;
	}
}

static int do_mem_mw(const rtems_printer *printer, int width, 
	int argc, char **argv) {
	unsigned long writeval;
	unsigned long addr, count;
	char *buf;

	(void) printer;
	if (argc < 3)
		return -EINVAL;

	addr = strtoul(argv[1], NULL, 16);
	writeval = strtoul(argv[2], NULL, 16);

	if (argc == 4)
		count = strtoul(argv[3], NULL, 16);
	else
		count = 1;

	buf = (char *)addr;
	while (count-- > 0) {
		if (width == 4)
			*((uint32_t *)buf) = (uint32_t)writeval;
		else if (width == 2)
			*((uint16_t *)buf) = (uint16_t)writeval;
		else
			*((uint8_t *)buf) = (uint8_t)writeval;
		buf += width;
	}

	return 0;
}

static int do_mem_md(const rtems_printer *printer, int width, 
	int argc, char **argv) {
	unsigned long addr;
	int count;

	if (argc < 2)
		return -EINVAL;

	addr = strtoul(argv[1], NULL, 16);

	if (argc == 3)
		count = strtoul(argv[2], NULL, 16);
	else
		count = 1;

	print_buffer(printer, (char *)addr, width, count, 
		DISP_LINE_LEN / width, -1);
	return 0;
}

static int shell_cmd_mdw(int argc, char **argv) {
	struct printer_ctx ctx;
	printer_acquire(argc, argv, &ctx);
	int ret = do_mem_md(&ctx.printer, 4, argc, argv);
	printer_release(&ctx);
	return ret;
}

static int shell_cmd_mdh(int argc, char **argv) {
	struct printer_ctx ctx;
	printer_acquire(argc, argv, &ctx);
	int ret = do_mem_md(&ctx.printer, 2, argc, argv);
	printer_release(&ctx);
	return ret;
}

static int shell_cmd_mdb(int argc, char **argv) {
	struct printer_ctx ctx;
	printer_acquire(argc, argv, &ctx);
	int ret = do_mem_md(&ctx.printer, 1, argc, argv);
	printer_release(&ctx);
	return ret;
}

static int shell_cmd_mww(int argc, char **argv) {
	struct printer_ctx ctx;
	printer_acquire(argc, argv, &ctx);
	int ret = do_mem_mw(&ctx.printer, 4, argc, argv);
	printer_release(&ctx);
	return ret;
}

static int shell_cmd_mwh(int argc, char **argv) {
	struct printer_ctx ctx;
	printer_acquire(argc, argv, &ctx);
	int ret = do_mem_mw(&ctx.printer, 2, argc, argv);
	printer_release(&ctx);
	return ret;
}

static int shell_cmd_mwb(int argc, char **argv) {
	struct printer_ctx ctx;
	printer_acquire(argc, argv, &ctx);
	int ret = do_mem_mw(&ctx.printer, 1, argc, argv);
	printer_release(&ctx);
	return ret;
}


static rtems_shell_cmd_t shell_mdw_command = {
    "mdw",
    "display memory by word: mdw address [,count]",
    "misc",
    shell_cmd_mdw
};

static rtems_shell_cmd_t shell_mdh_command = {
    "mdh",
    "display memory by half-word: mdh address [,count]",
    "misc",
    shell_cmd_mdh
};

static rtems_shell_cmd_t shell_mdb_command = {
    "mdb",
    "display memory by byte: mdb address [,count]",
    "misc",
    shell_cmd_mdb
};

static rtems_shell_cmd_t shell_mww_command = {
    "mww",
    "memory write (fill) by word: mww address value [,count]",
    "misc",
    shell_cmd_mww
};

static rtems_shell_cmd_t shell_mwh_command = {
    "mwh",
    "memory write (fill) by half-word: mwh address value [,count]",
    "misc",
    shell_cmd_mwh
};

static rtems_shell_cmd_t shell_mwb_command = {
    "mwb",
    "memory write (fill) by byte: mwb address value [,count]",
    "misc",
    shell_cmd_mwb
};

static void shell_mem_register(void) {
	rtems_shell_add_cmd_struct(&shell_mdw_command);
	rtems_shell_add_cmd_struct(&shell_mdh_command);
	rtems_shell_add_cmd_struct(&shell_mdb_command);
	rtems_shell_add_cmd_struct(&shell_mww_command);
	rtems_shell_add_cmd_struct(&shell_mwh_command);
	rtems_shell_add_cmd_struct(&shell_mwb_command);
}

RTEMS_SYSINIT_ITEM(shell_mem_register, 
	RTEMS_SYSINIT_STD_FILE_DESCRIPTORS + 10, 
	RTEMS_SYSINIT_ORDER_MIDDLE);
#endif /* CONFIGURE_SHELL_COMMANDS_MEM */
