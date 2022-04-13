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
#include <rtems/print.h>

#ifdef __cplusplus
extern "C"{
#endif

#ifdef CONFIG_ARM_UNWIND
#define UNWIND(code...)		code
#else
#define UNWIND(code...)
#endif

struct _Thread_Control;

/* Unwind reason code according the the ARM EABI documents */
enum unwind_reason_code {
	URC_OK = 0,			/* operation completed successfully */
	URC_CONTINUE_UNWIND = 8,
	URC_FAILURE = 9			/* unspecified failure of some kind */
};

struct unwind_idx {
	unsigned long addr_offset;
	unsigned long insn;
};

struct unwind_table {
//	struct list_head list;
	const struct unwind_idx *start;
	const struct unwind_idx *origin;
	const struct unwind_idx *stop;
	unsigned long begin_addr;
	unsigned long end_addr;
};

void unwind_backtrace(rtems_printer *printer, CPU_Exception_frame *regs,
	struct _Thread_Control *tsk);

#ifdef __cplusplus
}
#endif
#endif	/* __ASM_UNWIND_H */
