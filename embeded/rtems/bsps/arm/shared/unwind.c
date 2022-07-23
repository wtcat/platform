// SPDX-License-Identifier: GPL-2.0-only
/*
 * arch/arm/kernel/unwind.c
 *
 * Copyright (C) 2008 ARM Limited
 *
 * Stack unwinding support for ARM
 *
 * An ARM EABI version of gcc is required to generate the unwind
 * tables. For information about the structure of the unwind tables,
 * see "Exception Handling ABI for the ARM Architecture" at:
 *
 * http://infocenter.arm.com/help/topic/com.arm.doc.subset.swdev.abi/index.html
 */

/*
 * CopyRight (C) 2022 wtcat
 */

#include <rtems/score/cpu.h>
#include <rtems/score/percpu.h>
#include <rtems/score/thread.h>
#include <rtems/printer.h>
#include <rtems/bspIo.h>

#include "component/compiler.h"
#include "bsp/asm/unwind.h"


//#define DEBUG_ON

enum regs {
#if defined(__thumb2__) || defined(__thumb__)
	FP = 7,
#else
	FP = 11,
#endif
	SP = 13,
	LR = 14,
	PC = 15
};

#define prel31_to_addr(ptr)	({\
	long offset = (((long)*(ptr)) << 1) >> 1;	\
	(unsigned long)(ptr) + offset;			\
})

#ifdef DEBUG_ON
#define pr_debug(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define pr_debug(...)
#endif
#define pr_warn(fmt, ...) printk(fmt, ##__VA_ARGS__)

#ifndef likely
#define likely(x) RTEMS_PREDICT_TRUE(x)
#endif
#ifndef unlikely
#define unlikely(x) RTEMS_PREDICT_FALSE(x)
#endif

#define THREAD_SIZE 4096
#ifndef ALIGN
#define ALIGN(v, s) (((v) + ((s) - 1)) & ~(s))
#endif

extern char bsp_section_text_begin[];
extern char bsp_section_text_end[];

extern const unwind_index_t __exidx_start[];
extern const unwind_index_t __exidx_end[];

#define thread_saved_fp(tsk) \
	(unsigned long)(tsk)->Registers.register_fp
#define thread_saved_sp(tsk) \
	(unsigned long)(tsk)->Registers.register_sp
#define thread_saved_pc(tsk) \
	(unsigned long)(tsk)->Registers.register_lr

static inline unsigned long __notrace current_stack_pointer(void) {
	unsigned long stkptr;
	__asm__ volatile ("mov %[stkptr], sp\n"
		: [stkptr] "=r" (stkptr)
	);
	return stkptr;
}

static unsigned long __notrace thread_stack_end(struct _Thread_Control *tsk) {
	Stack_Control *stk = &tsk->Start.Initial_stack;
	return (unsigned long)stk->area + stk->size;
}

/* Dummy functions to avoid linker complaints */
//void __aeabi_unwind_cpp_pr0(void) {}
void __notrace __aeabi_unwind_cpp_pr1(void) {}
void __notrace __aeabi_unwind_cpp_pr2(void) {}

static __notrace bool kernel_text_address(unsigned long addr) {
	if (addr >= (unsigned long)bsp_section_text_begin &&
		addr <= (unsigned long)bsp_section_text_end)
		return true;
	return false;
}

static const char *__notrace kernel_symbols(unsigned long addr) {
#define SYMBOL_MAGIC 0xFF000000ul
	unsigned long *ptr= (unsigned long *)(addr - 4);
	if ((*ptr & SYMBOL_MAGIC) == SYMBOL_MAGIC) {
		unsigned long ofs = *ptr & ~SYMBOL_MAGIC;
		return (char *)ptr - ofs;
	}
	return "Unknown";
}

static const struct unwind_index *
__notrace unwind_search_index(const unwind_index_t *start, 
	const unwind_index_t *end, uint32_t ip) {
	const struct unwind_index *middle;

	/* Perform a binary search of the unwind index */
	while (start < end - 1) {
		middle = start + ((end - start + 1) >> 1);
		if (ip < prel31_to_addr(&middle->addr_offset))
			end = middle;
		else
			start = middle;
	}
	return start;
}

static int __notrace unwind_get_next_byte(unwind_control_block_t *ucb) {
	int instruction;

	/* Are there more instructions */
	if (ucb->remaining == 0)
		return -1;

	/* Extract the current instruction */
	instruction = ((*ucb->current) >> (ucb->byte << 3)) & 0xff;

	/* Move the next byte */
	--ucb->byte;
	if (ucb->byte < 0) {
		++ucb->current;
		ucb->byte = 3;
	}
	--ucb->remaining;
	return instruction;
}

static int __notrace unwind_control_block_init(unwind_control_block_t *ucb, 
	const uint32_t *instructions, const backtrace_frame_t *frame) {
	/* Initialize control block */
	memset(ucb, 0, sizeof(unwind_control_block_t));
	ucb->current = instructions;

	/* Is the a short unwind description */
	if ((*instructions & 0xff000000) == 0x80000000) {
		ucb->remaining = 3;
		ucb->byte = 2;
	/* Is the a long unwind description */
	} else if ((*instructions & 0xff000000) == 0x81000000) {
		ucb->remaining = ((*instructions & 0x00ff0000) >> 14) + 2;
		ucb->byte = 1;
	} else
		return -1;

	/* Initialize the virtual register set */
	if (frame) {
		ucb->vrs[7] = frame->fp;
		ucb->vrs[13] = frame->sp;
		ucb->vrs[14] = frame->lr;
		ucb->vrs[15] = 0;
	}
	return 0;
}

static int __notrace unwind_execute_instruction(unwind_control_block_t *ucb) {
	int instruction;
	uint32_t mask;
	uint32_t reg;
	uint32_t *vsp;

	/* Consume all instruction byte */
	while ((instruction = unwind_get_next_byte(ucb)) != -1) {
		if ((instruction & 0xc0) == 0x00) {
			/* vsp = vsp + (xxxxxx << 2) + 4 */
			ucb->vrs[13] += ((instruction & 0x3f) << 2) + 4;
		} else if ((instruction & 0xc0) == 0x40) {
			/* vsp = vsp - (xxxxxx << 2) - 4 */
			ucb->vrs[13] -= ((instruction & 0x3f) << 2) - 4;
		} else if ((instruction & 0xf0) == 0x80) {
			/* pop under mask {r15-r12},{r11-r4} or refuse to unwind */
			instruction = instruction << 8 | unwind_get_next_byte(ucb);

			/* Check for refuse to unwind */
			if (instruction == 0x8000)
				return 0;

			/* Pop registers using mask */
			vsp = (uint32_t *)ucb->vrs[13];
			mask = instruction & 0xfff;
			reg = 4;
			while (mask != 0) {
				if ((mask & 0x001) != 0)
					ucb->vrs[reg] = *vsp++;
				mask = mask >> 1;
				++reg;
			}

			/* Update the vrs sp as usual if r13 (sp) was not in the mask,
			 * otherwise leave the popped r13 as is. */
			if ((mask & (1 << (13 - 4))) == 0)
				ucb->vrs[13] = (uint32_t)vsp;
		} else if ((instruction & 0xf0) == 0x90 && instruction != 0x9d && instruction != 0x9f) {
			/* vsp = r[nnnn] */
			ucb->vrs[13] = ucb->vrs[instruction & 0x0f];
		} else if ((instruction & 0xf0) == 0xa0) {
			/* pop r4-r[4+nnn] or pop r4-r[4+nnn], r14*/
			vsp = (uint32_t *)ucb->vrs[13];
			for (reg = 4; reg <= (uint32_t)(instruction & 0x07) + 4; ++reg)
				ucb->vrs[reg] = *vsp++;
			if (instruction & 0x08)
				ucb->vrs[14] = *vsp++;
			ucb->vrs[13] = (uint32_t)vsp;
		} else if (instruction == 0xb0) {
			/* finished */
			if (ucb->vrs[15] == 0)
				ucb->vrs[15] = ucb->vrs[14];
			return 0;
		} else if (instruction == 0xb1) {
			/* pop register under mask {r3,r2,r1,r0} */
			vsp = (uint32_t *)ucb->vrs[13];
			mask = unwind_get_next_byte(ucb);

			reg = 0;
			while (mask != 0) {
				if ((mask & 0x01) != 0)
					ucb->vrs[reg] = *vsp++;
				mask = mask >> 1;
				++reg;
			}
			ucb->vrs[13] = (uint32_t)vsp;

		} else if (instruction == 0xb2) {
			/* vps = vsp + 0x204 + (uleb128 << 2) */
			ucb->vrs[13] += 0x204 + (unwind_get_next_byte(ucb) << 2);
		} else if (instruction == 0xb3 || instruction == 0xc8 || instruction == 0xc9) {
			/* pop VFP double-precision registers */
			vsp = (uint32_t *)ucb->vrs[13];

			/* D[ssss]-D[ssss+cccc] or D[16+sssss]-D[16+ssss+cccc] as pushed by VPUSH or FSTMFDX */
			vsp += 2 * ((unwind_get_next_byte(ucb) & 0x0f) + 1);
			if (instruction == 0xb3) {
				/* as pushed by FSTMFDX */
				vsp++;
			}
			ucb->vrs[13] = (uint32_t)vsp;
		} else if ((instruction & 0xf8) == 0xb8 || (instruction & 0xf8) == 0xd0) {
			/* pop VFP double-precision registers */
			vsp = (uint32_t *)ucb->vrs[13];

			/* D[8]-D[8+nnn] as pushed by VPUSH or FSTMFDX */
			vsp += 2 * ((instruction & 0x07) + 1);
			if ((instruction & 0xf8) == 0xb8) {
				/* as pushed by FSTMFDX */
				vsp++;
			}
			ucb->vrs[13] = (uint32_t)vsp;
		} else
			return -1;
	}
	return instruction != -1;
}

static int __notrace unwind_frame(backtrace_frame_t *frame) {
	unwind_control_block_t ucb;
	const unwind_index_t *index;
	const uint32_t *instructions;
	unsigned long high, low;
	int result;

	pr_debug("%s(pc = %08lx lr = %08lx sp = %08lx)\n", __func__,
		 frame->pc, frame->lr, frame->sp);
	low = frame->sp;
	high = frame->top;
	/* Search the unwind index for the matching unwind table */
	index = unwind_search_index(__exidx_start, __exidx_end, frame->pc);
	if (index == NULL) {
		pr_debug("unwind: Index not found %08lx\n", frame->pc);
		return -1;
	}

	/* Make sure we can unwind this frame */
	if (index->insn == 0x00000001) {
		pr_debug("Invalid instruction {offset=0x%x, insn = 0x%x}\n", 
			index->addr_offset,index->insn);
		return 0;
	}

	/* Get the pointer to the first unwind instruction */
	if (index->insn & 0x80000000)
		instructions = &index->insn;
	else
		instructions = (uint32_t *)prel31_to_addr(&index->insn);

	/* Initialize the unwind control block */
	if (unwind_control_block_init(&ucb, instructions, frame) < 0)
		return -1;

	/* Execute the unwind instructions TODO range check the stack pointer */
	while ((result = unwind_execute_instruction(&ucb)) > 0) {
		if (ucb.vrs[SP] < low || ucb.vrs[SP] >= high) {
			pr_debug("Stack limit reached (ucb.vrs[SP] = 0x%x)\n", ucb.vrs[SP]);
			return -1;
		}
	}
	if (result == -1) {
		pr_debug("unwind_execute_instruction: %d\n", result);
		return -1;
	}

	/* Set the virtual pc to the virtual lr if this is the first unwind */
	if (ucb.vrs[15] == 0)
		ucb.vrs[15] = ucb.vrs[14];

	/* We are done if current frame pc is equal to the virtual pc, prevent infinite loop */
	if (frame->pc == ucb.vrs[15])
		return 0;

	/* Update the frame */
	frame->fp = ucb.vrs[7];
	frame->sp = ucb.vrs[13];
	frame->lr = ucb.vrs[14];
	frame->pc = ucb.vrs[15];
	return 1;
}

static void __notrace backtrace_info(rtems_printer *printer) {
	rtems_printf(printer, "\n[Backtrace Dump] =>\n");
}

static void __notrace dump_stack(rtems_printer *printer, unsigned long sp, 
	struct _Thread_Control *tsk) {
	unsigned long top = sp + 0x100;
	if (tsk) {
		if (top > thread_stack_end(tsk))
			top = thread_stack_end(tsk);
	}
	rtems_printf(printer, "\n[Stack Dump] =>\n");
	while (sp < top) {
		rtems_printf(printer, "0x%08lx: 0x%08lx\n", sp, *(unsigned long *)sp);
		sp += sizeof(void *);
	}
}

static void __notrace dump_backtrace(rtems_printer *printer, unsigned long pc,
	unsigned long fnaddr, int level) {
	const char *sym = kernel_symbols(fnaddr);
	rtems_printf(printer, " [%2d] - <0x%lx>@ %s\n", level, pc, sym);
}

const char *__notrace unwind_kernel_symbols(unsigned long pc) {
	const unwind_index_t *index = unwind_search_index(__exidx_start, 
		__exidx_end, pc);
	if (index)
		return kernel_symbols(prel31_to_addr(&index->addr_offset));
	return "Unknown"; 
}

void __notrace __stack_backtrace(rtems_printer *printer, CPU_Exception_frame *regs,
	struct _Thread_Control *tsk) {
	const unwind_index_t *index;
	backtrace_frame_t frame;
	int level = 0;
	pr_debug("%s(regs = %p tsk = %p)\n", __func__, regs, tsk);
	if (tsk == NULL)
		tsk = _Thread_Executing;
	if (regs) {
		frame.fp = (unsigned long)regs->register_r11;
		frame.sp = (unsigned long)regs->register_sp;
		frame.lr = (unsigned long)regs->register_lr;
		frame.pc = (unsigned long)regs->register_pc;
		/* PC might be corrupted, use LR in that case. */
		if (!kernel_text_address((unsigned long)regs->register_pc))
			frame.pc = (unsigned long)regs->register_lr;
	} else if (tsk == _Thread_Executing) {
		frame.fp = (unsigned long)__builtin_frame_address(0);
		frame.sp = current_stack_pointer();
		frame.lr = (unsigned long)__builtin_return_address(0);
		frame.pc = (unsigned long)&__stack_backtrace;
	} else {
		/* task blocked in __switch_to */
		frame.fp = thread_saved_fp(tsk);
		frame.sp = thread_saved_sp(tsk);
		frame.lr = 0;
		frame.pc = thread_saved_pc(tsk);
	}
	backtrace_info(printer);
	frame.top = thread_stack_end(tsk);
	do {
		if (frame.pc == 0 || frame.pc == 0x00000001) {
			rtems_printf(printer, "<reached end of unwind table>\n");
			break;
		}
		index = unwind_search_index(__exidx_start, __exidx_end, frame.pc);
		frame.pc = (frame.pc >> 1) << 1;
		dump_backtrace(printer, frame.pc, prel31_to_addr(&index->addr_offset), level);
		level++;
	} while (unwind_frame(&frame));
	// if (regs)
	// 	dump_stack(printer, regs->register_sp, tsk);
}

