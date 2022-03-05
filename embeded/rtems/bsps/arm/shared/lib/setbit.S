/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  linux/arch/arm/lib/setbit.S
 *
 *  Copyright (C) 1995-1996 Russell King
 */
#include <linux/linkage.h>
#include <asm/assembler.h>
#include "bitops.h"
		.text

/* bitop	_set_bit, orr */

#if __LINUX_ARM_ARCH__ >= 6
ENTRY( _set_bit		)
UNWIND(	.fnstart	)
	ands	ip, r1, #3
	strbne	r1, [ip]		@ assert word-aligned
	mov	r2, #1
	and	r3, r0, #31		@ Get bit offset
	mov	r0, r0, lsr #5
	add	r1, r1, r0, lsl #2	@ Get word offset
#if __LINUX_ARM_ARCH__ >= 7 && defined(CONFIG_SMP)
	.arch_extension	mp
	ALT_SMP(W(pldw)	[r1])
	ALT_UP(W(nop))
#endif
	mov	r3, r2, lsl r3
1:	ldrex	r2, [r1]
	orr	r2, r2, r3
	strex	r0, r2, [r1]
	cmp	r0, #0
	bne	1b
	bx	lr
UNWIND(	.fnend		)
ENDPROC(_set_bit		)

#else /* __LINUX_ARM_ARCH__ < 6 */
ENTRY(	_set_bit		)
UNWIND(	.fnstart	)
	ands	ip, r1, #3
	strbne	r1, [ip]		@ assert word-aligned
	and	r2, r0, #31
	mov	r0, r0, lsr #5
	mov	r3, #1
	mov	r3, r3, lsl r2
	save_and_disable_irqs ip
	ldr	r2, [r1, r0, lsl #2]
	orr	r2, r2, r3
	str	r2, [r1, r0, lsl #2]
	restore_irqs ip
	ret	lr
UNWIND(	.fnend		)
ENDPROC(_set_bit		)
#endif /* __LINUX_ARM_ARCH__ */

