/*
 * Copyright 2022 wtcat
 */
    .section ".rodata.dtb", "a"
    .align 8

__bsp_dtb_start:
    .incbin "stm32h750-art-pi.dtb"
__bsp_dtb_end:
    .align 8

    .section ".text.bsp_fdt_get", "ax"
    .align 8
    .global __bsp_dtb_size
    .global bsp_fdt_get
    .type bsp_fdt_get, %function
    .thumb

bsp_fdt_get:
    ldr r0, =__bsp_dtb_start
    bx lr
    .set __bsp_dtb_size, __bsp_dtb_end - __bsp_dtb_start
    .end