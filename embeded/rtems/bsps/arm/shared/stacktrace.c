// SPDX-License-Identifier: GPL-2.0-only

/*
 * Copyright 2022 wtcat
 */
#include <errno.h>

#include <rtems/score/cpu.h>
#include <rtems/score/percpu.h>
#include <rtems/score/thread.h>
#include <rtems/printer.h>

#include "bsp/asm/stacktrace.h"

struct trace_data {
    const rtems_printer *printer;
    int level;
};

#define thread_saved_fp(tsk) \
	(unsigned long)(tsk)->Registers.register_fp
#define thread_saved_sp(tsk) \
	(unsigned long)(tsk)->Registers.register_sp
#define thread_saved_pc(tsk) \
	(unsigned long)(tsk)->Registers.register_lr

static inline unsigned long current_stack_pointer(void) {
	unsigned long stkptr;
	__asm__ volatile ("mov %[stkptr], sp\n"
		: [stkptr] "=r" (stkptr)
	);
	return stkptr;
}

static unsigned long thread_stack_end(struct _Thread_Control *tsk) {
	Stack_Control *stk = &tsk->Start.Initial_stack;
	return (unsigned long)stk->area + stk->size;
}

static bool kernel_text_address(unsigned long addr) {
extern char bsp_section_text_begin[];
extern char bsp_section_text_end[];
	if (addr >= (unsigned long)bsp_section_text_begin &&
		addr <= (unsigned long)bsp_section_text_end)
		return true;
	return false;
}

/*
 * Unwind the current stack frame and store the new register values in the
 * structure passed as argument. Unwinding is equivalent to a function return,
 * hence the new PC value rather than LR should be used for backtrace.
 *
 * With framepointer enabled, a simple function prologue looks like this:
 *	mov	ip, sp
 *	stmdb	sp!, {fp, ip, lr, pc}
 *	sub	fp, ip, #4
 *
 * A simple function epilogue looks like this:
 *	ldm	sp, {fp, sp, pc}
 *
 * When compiled with clang, pc and sp are not pushed. A simple function
 * prologue looks like this when built with clang:
 *
 *	stmdb	{..., fp, lr}
 *	add	fp, sp, #x
 *	sub	sp, sp, #y
 *
 * A simple function epilogue looks like this when built with clang:
 *
 *	sub	sp, fp, #x
 *	ldm	{..., fp, pc}
 *
 *
 * Note that with framepointer enabled, even the leaf functions have the same
 * prologue and epilogue, therefore we can ignore the LR value in this case.
 */
int unwind_frame(struct stackframe *frame)
{
	unsigned long high, low;
	unsigned long fp = frame->fp;

	/* only go to a higher address on the stack */
	low = frame->sp;
	high = frame->top;

#ifdef CONFIG_CC_IS_CLANG
	/* check current frame pointer is within bounds */
	if (fp < low + 4 || fp > high - 4)
		return -EINVAL;

	frame->sp = frame->fp;
	frame->fp = *(unsigned long *)(fp);
	frame->pc = frame->lr;
	frame->lr = *(unsigned long *)(fp + 4);
#else
	/* check current frame pointer is within bounds */
	if (fp < low + 12 || fp > high - 4)
		return -EINVAL;

	/* restore the registers from the stack frame */
	frame->fp = *(unsigned long *)(fp - 12);
	frame->sp = *(unsigned long *)(fp - 8);
	frame->pc = *(unsigned long *)(fp - 4);
#endif

	return 0;
}

void walk_stackframe(struct stackframe *frame,
    int (*fn)(struct stackframe *, void *), void *data) {
	while (1) {
		if (fn(frame, data))
			break;
		int ret = unwind_frame(frame);
		if (ret < 0)
			break;
	}
}

static int dump_backtrace(struct stackframe *frame, void *data) {
	if (kernel_text_address(frame->pc)) {
		struct trace_data *ctx = (struct trace_data *)data;
	//	const char *sym = kernel_symbols(fnaddr);
		rtems_printf(ctx->printer, " [%2d] - <0x%lx>@ %s\n", ctx->level, 
			frame->pc, " ");
		ctx->level++;
		return 0;
	}
	return -EINVAL;
}

void __stack_backtrace(rtems_printer *printer, CPU_Exception_frame *regs,
    struct _Thread_Control *tsk) {
	struct stackframe frame;
    struct trace_data data;
	if (tsk == NULL)
		tsk = _Thread_Executing;
	if (regs) {
		frame.fp = frame_pointer(regs);
		frame.sp = (unsigned long)regs->register_sp;
		frame.lr = (unsigned long)regs->register_lr;
		frame.pc = (unsigned long)regs->register_pc;
		/* PC might be corrupted, use LR in that case. */
		if (!kernel_text_address((unsigned long)regs->register_pc))
			frame.pc = (unsigned long)regs->register_lr;            
	} else if (tsk == _Thread_Executing) {
		frame.fp = thread_saved_fp(tsk);
		frame.sp = thread_saved_sp(tsk);
		frame.lr = 0;		/* recovered from the stack */
		frame.pc = thread_saved_pc(tsk);
	} else {
		frame.fp = (unsigned long)__builtin_frame_address(0);
		frame.sp = current_stack_pointer();
		frame.lr = (unsigned long)__builtin_return_address(0);
		frame.pc = (unsigned long)__stack_backtrace;
	}
    frame.top = thread_stack_end(tsk);
    data.printer = printer;
    data.level = 0;
	walk_stackframe(&frame, dump_backtrace, &data);
}
