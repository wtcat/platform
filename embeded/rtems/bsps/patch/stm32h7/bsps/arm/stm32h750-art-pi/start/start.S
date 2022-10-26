/**
 * @file
 *
 * @brief Boot and system start code.
 */

/*
 * Copyright (c) 2008, 2020 embedded brains GmbH.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Dornierstr. 4
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * Copyright (c) 2016 Pavel Pisa <pisa@cmp.felk.cvut.cz>
 *
 *  Czech Technical University in Prague
 *  Zikova 1903/4
 *  166 36 Praha 6
 *  Czech Republic
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <rtems/asm.h>
#include <rtems/score/percpu.h>
#include <rtems/score/armv7m.h>
#include <bspopts.h>
#include <bsp/irq.h>

	/* Global symbols */
	.globl	_start
	.globl	bsp_start_hook_0_done
	.globl	bsp_start_vector_table_begin
	.globl	bsp_start_vector_table_end
	.globl	bsp_start_vector_table_size
	.globl	bsp_vector_table_size
    .extern _cortexm_exception_default

	.section	".bsp_start_text", "ax"
	.syntax	unified
	.thumb

bsp_start_vector_table_begin:

	.word	_ISR_Stack_area_end
	.word	_start /* Reset */
	.word	_cortexm_exception_default /* NMI */
	.word	_cortexm_exception_default /* Hard Fault */
	.word	_cortexm_exception_default /* MPU Fault */
	.word	_cortexm_exception_default /* Bus Fault */
	.word	_cortexm_exception_default /* Usage Fault */
	.word	_cortexm_exception_default /* Reserved */
	.word	_cortexm_exception_default /* Reserved */
	.word	_cortexm_exception_default /* Reserved */
	.word	_cortexm_exception_default /* Reserved */
	.word	_ARMV7M_Supervisor_call /* SVC */
	.word	_cortexm_exception_default /* Debug Monitor */
	.word	_cortexm_exception_default /* Reserved */
	.word	_ARMV7M_Pendable_service_call /* PendSV */
	.word	_ARMV7M_Clock_handler /* SysTick */
	.rept	BSP_INTERRUPT_VECTOR_COUNT
	.word	_ARMV7M_NVIC_Interrupt_dispatch /* IRQ */
	.endr

bsp_start_vector_table_end:

	.thumb_func

_start:

#if defined(ARM_MULTILIB_VFP) && defined(ARM_MULTILIB_HAS_CPACR)
	/*
	 * Enable CP10 and CP11 coprocessors for privileged and user mode in
	 * CPACR (bits 20-23).  Ensure that write to register completes.
	 */
	ldr	r0, =ARMV7M_CPACR
	ldr	r1, [r0]
	orr	r1, r1, #(0xf << 20)
	str	r1, [r0]
	dsb
	isb
#endif

	ldr	sp, =_ISR_Stack_area_end

	/*
	 * Set the Main Stack Pointer (MSP) to the ISR stack area end just in
	 * case we run using the Process Stack Pointer (PSP).
	 */
	mov	r0, sp
	msr	msp, r0

	ldr	lr, =bsp_start_hook_0_done + 1
	b	bsp_start_hook_0

	/* Allow bsp_start_hook_0() hooks to jump to this label */
bsp_start_hook_0_done:

	bl	bsp_start_hook_1
	movs	r0, #0
	bl	boot_card

	.set	bsp_start_vector_table_size, bsp_start_vector_table_end - bsp_start_vector_table_begin
	.set	bsp_vector_table_size, bsp_start_vector_table_size
