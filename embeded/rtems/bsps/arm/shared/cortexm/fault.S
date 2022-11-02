/*
 * Copyright 2022 wtcat
 *
 * Cortex-M excepetion entry
 */
    .section ".text.cortexm.fault", "ax"
    .global _cortexm_exception_default
    .extern _cortexm_fault
    .thumb

_cortexm_exception_default:
	mrs r0, MSP
	mrs r1, PSP

	push {r0, lr}
    push {r1, r2}

    /* push {r4-r11} */
	mov  r3, r11
	mov  r2, r10
	push {r2, r3}
	mov  r3, r9
	mov  r2, r8
	push {r2, r3}
	push {r4-r7}

    mov r3, sp /* pointer to struct cortexm_exception */
    mov r2, lr /* EXC_RETURN */
    bl _cortexm_fault

    add sp, #40
    pop {r0, pc}

    .end