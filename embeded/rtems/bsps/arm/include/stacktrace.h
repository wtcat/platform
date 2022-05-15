/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_STACKTRACE_H
#define __ASM_STACKTRACE_H

#include <rtems/score/cpu.h>
#include <rtems/printer.h>

#include "bsp/asm/unwind.h"
#ifdef __cplusplus
extern "C"{
#endif

struct stackframe {
	unsigned long fp;
	unsigned long sp;
	unsigned long lr;
	unsigned long pc;
	unsigned long top;
};

#if defined(__thumb2__) || defined(__thumb__)
#define frame_pointer(regs) (regs)->register_r7
#else
#define frame_pointer(regs) (regs)->register_r11
#endif

#define stack_backtrace(printer) __stack_backtrace(printer, NULL, NULL)

int unwind_frame(struct stackframe *frame);
void walk_stackframe(struct stackframe *frame,
			    int (*fn)(struct stackframe *, void *), void *data);
void __stack_backtrace(rtems_printer *printer, CPU_Exception_frame *regs,
    struct _Thread_Control *tsk);

#ifdef __cplusplus
}
#endif
#endif	/* __ASM_STACKTRACE_H */
