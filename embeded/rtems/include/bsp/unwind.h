/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * arch/arm/include/asm/unwind.h
 *
 * Copyright (C) 2008 ARM Limited
 */

/*
 * CopyRight (C) 2022 wtcat
 */

#ifndef __ASM_UNWIND_H
#define __ASM_UNWIND_H

#include <rtems/score/cpu.h>
#include <rtems/printer.h>
#ifdef __cplusplus
extern "C"{
#endif

typedef struct backtrace_frame
{
	uint32_t fp;
	uint32_t sp;
	uint32_t lr;
	uint32_t pc;
	uint32_t top;
} backtrace_frame_t;

typedef struct backtrace
{
	void *function;
	void *address;
	const char *name;
} backtrace_t;

typedef struct unwind_control_block
{
	uint32_t vrs[16];
	const uint32_t *current;
	int remaining;
	int byte;
} unwind_control_block_t;

typedef struct unwind_index
{
	uint32_t addr_offset;
	uint32_t insn;
} unwind_index_t;

const char *unwind_kernel_symbols(unsigned long pc);
void __unwind_backtrace(rtems_printer *printer, 
	CPU_Exception_frame *regs, struct _Thread_Control *tsk);

#define unwind_backtrace(printer) __unwind_backtrace(printer, NULL, NULL)
	
#ifdef __cplusplus
}
#endif
#endif	/* __ASM_UNWIND_H */
