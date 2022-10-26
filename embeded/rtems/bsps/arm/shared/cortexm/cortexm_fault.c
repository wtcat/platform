/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <stdbool.h>

#include <rtems/score/armv7m.h>
#include <rtems/fatal.h>
#include <rtems/bspIo.h>

#include "asm/cortexm_trace.h"
#include "asm/cmsis_cortexm.h"


struct cortexm_cpu_exception {
	CPU_Exception_frame base;
	uint32_t msp;
	uint32_t psp;
	uint32_t exec_return;
};

#define STORE_xFAR(reg_var, reg) uint32_t reg_var = (uint32_t)reg
enum {
    K_ERR_CPU_EXCEPTION = 1
};

static inline bool z_arm_is_synchronous_svc(const rtems_exception_frame *frame) {
	uint16_t *ret_addr = (uint16_t *)frame->register_pc;
    uint16_t fault_insn;
	/* SVC is a 16-bit instruction. On a synchronous SVC
	 * escalated to Hard Fault, the return address is the
	 * next instruction, i.e. after the SVC.
	 */
#define _SVC_OPCODE 0xDF00

	/* We are about to de-reference the program counter at the
	 * time of fault to determine if it was a SVC
	 * instruction. However, we don't know if the pc itself is
	 * valid -- we could have faulted due to trying to execute a
	 * corrupted function pointer.
	 *
	 * We will temporarily ignore BusFault's so a bad program
	 * counter does not trigger ARM lockup condition.
	 */
	SCB->CCR |= SCB_CCR_BFHFNMIGN_Msk;
	__DSB();
	__ISB();

    fault_insn = *(ret_addr - 1);
	SCB->CCR &= ~SCB_CCR_BFHFNMIGN_Msk;
	__DSB();
	__ISB();

	if (((fault_insn & 0xff00) == _SVC_OPCODE))
		return true;
#undef _SVC_OPCODE
	return false;
}

static void reserved_exception(const rtems_exception_frame *frame, int fault) {
    (void)frame;
	printk("***** %s %d) *****",
	       fault < 16 ? "Reserved Exception (\n" : "Spurious interrupt (IRQ \n",
	       fault - 16);
}

static void fault_show(const rtems_exception_frame *frame, int fault) {
    (void)frame;
	printk("Fault! EXC #%d\n", fault);
	// printk("MMFSR: 0x%x, BFSR: 0x%x, UFSR: 0x%x\n", SCB_CFSR_MEMFAULTSR,
	//        SCB_CFSR_BUSFAULTSR, SCB_CFSR_USGFAULTSR);
#if defined(CONFIG_ARM_SECURE_FIRMWARE)
	printk("SFSR: 0x%x", SAU->SFSR);
#endif /* CONFIG_ARM_SECURE_FIRMWARE */

}

static void debug_monitor(const rtems_exception_frame *frame) {
    (void) frame;
	printk("***** Debug monitor exception *****\n");
}

static uint32_t usage_fault(const rtems_exception_frame *frame) {
	uint32_t reason = K_ERR_CPU_EXCEPTION;
    (void) frame;

	printk("***** USAGE FAULT *****\n");
	/* bits are sticky: they stack and must be reset */
	if ((SCB->CFSR & SCB_CFSR_DIVBYZERO_Msk) != 0)
		printk("  Division by zero\n");
	if ((SCB->CFSR & SCB_CFSR_UNALIGNED_Msk) != 0)
		printk("  Unaligned memory access\n");
	if ((SCB->CFSR & SCB_CFSR_NOCP_Msk) != 0)
		printk("  No coprocessor instructions\n");
	if ((SCB->CFSR & SCB_CFSR_INVPC_Msk) != 0)
		printk("  Illegal load of EXC_RETURN into PC\n");
	if ((SCB->CFSR & SCB_CFSR_INVSTATE_Msk) != 0)
		printk("  Illegal use of the EPSR\n");
	if ((SCB->CFSR & SCB_CFSR_UNDEFINSTR_Msk) != 0)
		printk("  Attempt to execute undefined instruction\n");
	/* clear UFSR sticky bits */
	SCB->CFSR |= SCB_CFSR_USGFAULTSR_Msk;
	return reason;
}

static int bus_fault(const rtems_exception_frame *frame, int from_hard_fault) {
	uint32_t reason = K_ERR_CPU_EXCEPTION;
    (void) frame;

	printk("***** BUS FAULT *****\n");
	if (SCB->CFSR & SCB_CFSR_STKERR_Msk)
		printk("  Stacking error\n");
	if (SCB->CFSR & SCB_CFSR_UNSTKERR_Msk)
		printk("  Unstacking error\n");
	if (SCB->CFSR & SCB_CFSR_PRECISERR_Msk) {
		printk("  Precise data bus error\n");
		/* In a fault handler, to determine the true faulting address:
		 * 1. Read and save the BFAR value.
		 * 2. Read the BFARVALID bit in the BFSR.
		 * The BFAR address is valid only if this bit is 1.
		 *
		 * Software must follow this sequence because another
		 * higher priority exception might change the BFAR value.
		 */
		STORE_xFAR(bfar, SCB->BFAR);
		if ((SCB->CFSR & SCB_CFSR_BFARVALID_Msk) != 0) {
			printk("  BFAR Address: 0x%x\n", bfar);
			if (from_hard_fault != 0) {
				/* clear SCB_CFSR_BFAR[VALID] to reset */
				SCB->CFSR &= ~SCB_CFSR_BFARVALID_Msk;
			}
		}
	}
	if (SCB->CFSR & SCB_CFSR_IMPRECISERR_Msk)
		printk("  Imprecise data bus error\n");
	if ((SCB->CFSR & SCB_CFSR_IBUSERR_Msk) != 0)
		printk("  Instruction bus error\n");
	else if (SCB->CFSR & SCB_CFSR_LSPERR_Msk)
		printk("  Floating-point lazy state preservation error\n");
	/* clear BFSR sticky bits */
	SCB->CFSR |= SCB_CFSR_BUSFAULTSR_Msk;
	return reason;
}

static uint32_t mem_manage_fault(const rtems_exception_frame *frame, 
    int from_hard_fault) {
	uint32_t reason = K_ERR_CPU_EXCEPTION;
    (void) frame;

	printk("***** MPU FAULT *****\n");
	if ((SCB->CFSR & SCB_CFSR_MSTKERR_Msk) != 0)
		printk("  Stacking error (context area might be not valid)\n");
	if ((SCB->CFSR & SCB_CFSR_MUNSTKERR_Msk) != 0)
		printk("  Unstacking error\n");
	if ((SCB->CFSR & SCB_CFSR_DACCVIOL_Msk) != 0) {
		printk("  Data Access Violation\n");
		/* In a fault handler, to determine the true faulting address:
		 * 1. Read and save the MMFAR value.
		 * 2. Read the MMARVALID bit in the MMFSR.
		 * The MMFAR address is valid only if this bit is 1.
		 *
		 * Software must follow this sequence because another higher
		 * priority exception might change the MMFAR value.
		 */
		uint32_t temp = SCB->MMFAR;

		if ((SCB->CFSR & SCB_CFSR_MMARVALID_Msk) != 0) {
			printk("  MMFAR Address: 0x%x\n", temp);
			if (from_hard_fault != 0) {
				/* clear SCB_MMAR[VALID] to reset */
				SCB->CFSR &= ~SCB_CFSR_MMARVALID_Msk;
			}
		}
	}
	if ((SCB->CFSR & SCB_CFSR_IACCVIOL_Msk) != 0)
		printk("  Instruction Access Violation\n");
	if ((SCB->CFSR & SCB_CFSR_MLSPERR_Msk) != 0)
		printk("  Floating-point lazy state preservation error\n");

	/* When stack protection is enabled, we need to assess
	 * if the memory violation error is a stack corruption.
	 *
	 * By design, being a Stacking MemManage fault is a necessary
	 * and sufficient condition for a thread stack corruption.
	 * [Cortex-M process stack pointer is always descending and
	 * is never modified by code (except for the context-switch
	 * routine), therefore, a stacking error implies the PSP has
	 * crossed into an area beyond the thread stack.]
	 *
	 * Data Access Violation errors may or may not be caused by
	 * thread stack overflows.
	 */
	if ((SCB->CFSR & SCB_CFSR_MSTKERR_Msk) ||
		(SCB->CFSR & SCB_CFSR_DACCVIOL_Msk)) {
        if (SCB->CFSR & SCB_CFSR_MSTKERR_Msk)
            printk("Stacking or Data Access Violation error "
                    "without stack guard, user-mode or null-pointer detection\n");
	}
	/* When we were handling this fault, we may have triggered a fp
	 * lazy stacking Memory Manage fault. At the time of writing, this
	 * can happen when printing.  If that's true, we should clear the
	 * pending flag in addition to the clearing the reason for the fault
	 */
	if ((SCB->CFSR & SCB_CFSR_MLSPERR_Msk) != 0) {
		SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTPENDED_Msk;
	}
	/* clear MMFSR sticky bits */
	SCB->CFSR |= SCB_CFSR_MEMFAULTSR_Msk;
	return reason;
}

static uint32_t hard_fault(const rtems_exception_frame *frame) {
	uint32_t reason = K_ERR_CPU_EXCEPTION;

	printk("***** HARD FAULT *****\n");
	if ((SCB->HFSR & SCB_HFSR_VECTTBL_Msk) != 0) {
		printk("  Bus fault on vector table read\n");
	} else if ((SCB->HFSR & SCB_HFSR_DEBUGEVT_Msk) != 0) {
		printk("  Debug event\n");
	} else if ((SCB->HFSR & SCB_HFSR_FORCED_Msk) != 0) {
		printk("  Fault escalation (see below)\n");
		if (z_arm_is_synchronous_svc(frame)) {
			printk("ARCH_EXCEPT with reason %x\n", frame->register_r0);
			reason = frame->register_r0;
		} else if ((SCB->CFSR & SCB_CFSR_MEMFAULTSR_Msk) != 0) {
			reason = mem_manage_fault(frame, 1);
		} else if ((SCB->CFSR & SCB_CFSR_BUSFAULTSR_Msk) != 0) {
			reason = bus_fault(frame, 1);
		} else if ((SCB->CFSR & SCB_CFSR_USGFAULTSR_Msk) != 0) {
			reason = usage_fault(frame);
#if defined(CONFIG_ARM_SECURE_FIRMWARE)
		} else if (SAU->SFSR != 0) {
			secure_fault(esf);
#endif /* CONFIG_ARM_SECURE_FIRMWARE */
		} else {
			printk("Fault escalation without FSR info\n");
		}
	} else {
		printk("HardFault without HFSR info"
		" Shall never occur\n");
	}
	return reason;
}

/* Handler function for ARM fault conditions. */
static uint32_t fault_handle(const rtems_exception_frame *frame, int fault) {
	uint32_t reason = K_ERR_CPU_EXCEPTION;
	switch (fault) {
	case 3:
		reason = hard_fault(frame);
		break;
	case 4:
		reason = mem_manage_fault(frame, 0);
		break;
	case 5:
		reason = bus_fault(frame, 0);
		break;
	case 6:
		reason = usage_fault(frame);
		break;
#if defined(CONFIG_ARM_SECURE_FIRMWARE)
	case 7:
		secure_fault(esf);
		break;
#endif /* CONFIG_ARM_SECURE_FIRMWARE */
	case 12:
		debug_monitor(frame);
		break;
	default:
		reserved_exception(frame, fault);
		break;
	}
    /* Dump generic information about the fault. */
    fault_show(frame, fault);
	return reason;
}

void bsp_cortexm_fault(const rtems_exception_frame *frame) {
	const struct cortexm_cpu_exception *cce = (void *)frame;
    int fault = SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk;
	uint32_t sp;

    fault_handle(frame, fault);
	printk("** exec_return: 0x%x msp = 0x%x, psp = 0x%x\n", cce->exec_return, cce->msp, cce->psp);
	sp = (cce->exec_return & 0x4)? cce->msp: cce->psp;
	cm_backtrace_fault(cce->exec_return, sp);
}

void __attribute__((naked)) _cortexm_exception_default(void) {

    /* On exception entry, ARMv7M saves context state onto a stack pointed to
     * by either MSP or PSP. The value stored in LR indicates whether we were
     * in Thread or Handler mode, whether we were using the FPU (if any),
     * and which stack pointer we were using.
     * In particular, bit 2 of LR will be 0 if we were using MSP.
     *
     * For a more detailed explanation, see the Exception Entry Behavior
     * section of the ARMv7M Architecture Reference Manual.
     */

    /* As we're in Handler mode here, we'll always operate on MSP.
     * However, we need to store the right SP in our CPU_Exception_frame.
     */
  __asm__ volatile (
    "sub sp, %[cpufsz]\n"   /* Allocate space for a CPU_Exception_frame. */
    "stm sp, {r0-r12}\n"
	"add r3, sp, %[cpumsp]\n" 
	"mrs r4, msp\n"
	"add r4, %[cpufsz]\n"
	"mrs r5, psp\n"
	"mov r6, lr\n"
	"stm r3, {r4-r6}\n"     /* Save msp, psp, exec_return */
    "tst lr, #4\n"          /* Check if bit 2 of LR is zero. If so, PSR.Z = 1 */
    "itte eq\n"             /* IF bit 2 of LR is zero... (PSR.Z == 1) */
    "mrseq r3, msp\n"       /* THEN we were using MSP */
    "addeq r3, %[cpufsz]\n" /* THEN, set r3 = old MSP value */
    "mrsne r3, psp\n"       /* ELSE it is not zero; we were using PSP */
    "add r2, r3, %[v7mlroff]\n"
    "add r1, sp, %[cpuspoff]\n"
    "ldm r2, {r4-r6}\n"     /* Grab LR, PC and xPSR from the stack */
    "tst lr, #16\n"         /* Check if we have an FP state on the frame */
    "ite eq\n"
    "addeq r3, #104\n"      /* Back to previous SP with FP state */
    "addne r3, #32\n"       /* Back to previous SP without FP state */
    "tst r6, #512\n"        /* Check xPSR if the SP was aligned */
    "it ne\n"
    "addne r3, #4\n"        /* Undo alignment */
    "stm r1, {r3-r6}\n"     /* Store to CPU_Exception_frame */
    "mrs r1, ipsr\n"
    "str r1, [sp, %[cpuvecoff]]\n"

    /* Argument for high level handler */
    "mov r0, sp\n"

    /* Clear VFP context pointer */
    "add r3, sp, %[cpuvfpoff]\n"
    "mov r1, #0\n"
    "str r1, [r3]\n"

#ifdef ARM_MULTILIB_VFP
    /* Ensure that the FPU is enabled */
    "ldr r4, =%[cpacr]\n"
    "tst r4, #(0xf << 20)\n"
    "bne 1f\n"

    /* Save VFP context */
    "sub sp, %[vfpsz]\n"
    "add r4, sp, #4\n"
    "bic r4, r4, #7\n"
    "str r4, [r3]\n"
    "vmrs r2, FPSCR\n"
    "stmia r4!, {r1-r2}\n"
    "vstmia r4!, {d0-d15}\n"
    "mov r1, #0\n"
    "mov r2, #0\n"
    "adds r3, r4, #128\n"
    "2:\n"
    "stmia r4!, {r1-r2}\n"
    "cmp r4, r3\n"
    "bne 2b\n"
    "1:\n"
#endif

    "b _ARM_Exception_default\n"
    :
    : [cpufsz] "i" (sizeof(struct cortexm_cpu_exception)),
	  [cpumsp] "i" (offsetof(struct cortexm_cpu_exception, msp)),
      [cpuspoff] "i" (offsetof(CPU_Exception_frame, register_sp)),
      [v7mlroff] "i" (offsetof(ARMV7M_Exception_frame, register_lr)),
      [cpuvecoff] "J" (offsetof(CPU_Exception_frame, vector)),
      [cpuvfpoff] "i" (ARM_EXCEPTION_FRAME_VFP_CONTEXT_OFFSET),
      [cpacr] "i" (ARMV7M_CPACR),
      [vfpsz] "i" (ARM_VFP_CONTEXT_SIZE)
  );
}