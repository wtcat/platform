/*
 * Copyright 2022 wtcat
 */
#include <bsp.h>
#include <bsp/start.h>
#include <bsp/linker-symbols.h>
#include <bsp/irq-generic.h>
#include <bsp/irq.h>

#include <rtems/bspIo.h>
#include <rtems/sysinit.h>
#include <rtems/score/armv7m.h>

#include "base/compiler.h"
#include "base/sections.h"

#include "stm32/stm32_sdram.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_usart.h"


extern char stm32h7_memory_null_begin[];
extern char stm32h7_memory_null_end[];

extern char stm32h7_memory_sram_axi_begin[];
extern char stm32h7_memory_sram_axi_end[];

extern char stm32h7_memory_sdram_1_begin[];
extern char stm32h7_memory_sdram_1_end[];

extern char stm32h7_memory_sdram_2_begin[];
extern char stm32h7_memory_sdram_2_end[];
extern char stm32h7_memory_sdram_2_size[];

extern void  _cortexm_exception_default(void);
#define EARLY_UART UART4

static const ARMV7M_MPU_Region_config stm32h7_mpu_map[] __unused = {
    {
      .begin = stm32h7_memory_sram_axi_begin,
      .end = stm32h7_memory_sram_axi_end,
      .rasr = ARMV7M_MPU_RASR_XN
        | ARMV7M_MPU_RASR_AP(0x3)
        | ARMV7M_MPU_RASR_TEX(0x1) | ARMV7M_MPU_RASR_C | ARMV7M_MPU_RASR_B
        | ARMV7M_MPU_RASR_ENABLE,
    }, {
      .begin = stm32h7_memory_sdram_1_begin,
      .end = stm32h7_memory_sdram_1_end,
      .rasr = ARMV7M_MPU_RASR_XN
        | ARMV7M_MPU_RASR_AP(0x3)
        | ARMV7M_MPU_RASR_TEX(0x1) | ARMV7M_MPU_RASR_C | ARMV7M_MPU_RASR_B
        | ARMV7M_MPU_RASR_ENABLE,
    }, {
      .begin = stm32h7_memory_sdram_2_begin,
      .end = stm32h7_memory_sdram_2_end,
      .rasr = ARMV7M_MPU_RASR_XN
        | ARMV7M_MPU_RASR_AP(0x3)
        | ARMV7M_MPU_RASR_TEX(0x1) | ARMV7M_MPU_RASR_C | ARMV7M_MPU_RASR_B
        | ARMV7M_MPU_RASR_ENABLE,
    }, {
      .begin = bsp_section_start_begin,
      .end = bsp_section_text_end,
      .rasr = ARMV7M_MPU_RASR_AP(0x5)
        | ARMV7M_MPU_RASR_TEX(0x1) | ARMV7M_MPU_RASR_C | ARMV7M_MPU_RASR_B
        | ARMV7M_MPU_RASR_ENABLE,
    }, {
      .begin = bsp_section_rodata_begin,
      .end = bsp_section_rodata_end,
      .rasr = ARMV7M_MPU_RASR_XN
        | ARMV7M_MPU_RASR_AP(0x5)
        | ARMV7M_MPU_RASR_TEX(0x1) | ARMV7M_MPU_RASR_C | ARMV7M_MPU_RASR_B
        | ARMV7M_MPU_RASR_ENABLE,
    }, {
      .begin = bsp_section_nocache_begin,
      .end = bsp_section_nocachenoload_end,
      .rasr = ARMV7M_MPU_RASR_XN
        | ARMV7M_MPU_RASR_AP(0x3)
        | ARMV7M_MPU_RASR_TEX(0x2)
        | ARMV7M_MPU_RASR_ENABLE,
    }, {
      .begin = stm32h7_memory_null_begin,
      .end = stm32h7_memory_null_end,
      .rasr = ARMV7M_MPU_RASR_XN | ARMV7M_MPU_RASR_ENABLE,
    }
};

static void __notrace stm32h7_early_uart_putc(char c) {
    while (!(EARLY_UART->ISR & USART_ISR_TXE_TXFNF));
    EARLY_UART->TDR = c;
}

static void __fastcode cortexm_interrupt_dispatch(void) {
  rtems_vector_number vector =
    ARMV7M_SCB_ICSR_VECTACTIVE_GET(_ARMV7M_SCB->icsr);

  _ARMV7M_Interrupt_service_enter();
  bsp_interrupt_handler_dispatch(ARMV7M_IRQ_OF_VECTOR(vector));
  _ARMV7M_Interrupt_service_leave();
}

static void __notrace bsp_vector_setup(void) {
  _ARMV7M_Set_exception_handler(ARMV7M_VECTOR_HARD_FAULT, 
    _cortexm_exception_default);
  _ARMV7M_Set_exception_handler(ARMV7M_VECTOR_MEM_MANAGE, 
    _cortexm_exception_default);
  _ARMV7M_Set_exception_handler(ARMV7M_VECTOR_BUS_FAULT , 
    _cortexm_exception_default);
  _ARMV7M_Set_exception_handler(ARMV7M_VECTOR_USAGE_FAULT, 
    _cortexm_exception_default);
  _ARMV7M_Set_exception_handler(ARMV7M_VECTOR_DEBUG_MONITOR, 
    _cortexm_exception_default);

  for (int i = 0; i < BSP_INTERRUPT_VECTOR_COUNT; i++) {
    _ARMV7M_Set_exception_handler(ARMV7M_VECTOR_IRQ(i), 
      cortexm_interrupt_dispatch);
  }
}

void __notrace bsp_start(void) {
    bsp_interrupt_initialize();
    bsp_vector_setup();
    rtems_cache_coherent_add_area(bsp_section_nocacheheap_begin,
        (uintptr_t)bsp_section_nocacheheap_size);
    rtems_cache_coherent_add_area(stm32h7_memory_sdram_2_begin,
      (uintptr_t)stm32h7_memory_sdram_2_size);
}

void __notrace bsp_start_hook_0(void) {
  BSP_output_char = stm32h7_early_uart_putc;
  if (!(RCC->AHB3ENR & RCC_AHB3ENR_FMCEN)) {
    stm32h7_sdram_init();
    printk("** BSP SDRAM setup OK**\n");
  }

  if ((SCB->CCR & SCB_CCR_IC_Msk) == 0) 
    SCB_EnableICache();
  if ((SCB->CCR & SCB_CCR_DC_Msk) == 0) 
    SCB_EnableDCache();
  // _ARMV7M_MPU_Setup(stm32h7_mpu_map, RTEMS_ARRAY_SIZE(stm32h7_mpu_map));
}

void __notrace bsp_start_hook_1(void) {
  bsp_start_copy_sections_compact();
  SCB_CleanDCache();
  SCB_InvalidateICache();
  bsp_start_clear_bss();
  BSP_output_char = stm32h7_early_uart_putc;
}

/* For stm32_hal library */
uint32_t HAL_GetTick(void) {
  return rtems_clock_get_ticks_since_boot() *
    rtems_configuration_get_milliseconds_per_tick();
}

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
}

uint32_t stm32h7_systick_frequency(void) {
  LL_RCC_ClocksTypeDef rcc_clks;
  LL_RCC_GetSystemClocksFreq(&rcc_clks);
  return rcc_clks.SYSCLK_Frequency;
}

uint32_t bsp_fdt_map_intr(const uint32_t *intr, 
	size_t icells) {
    (void) icells;
    return intr[0];
}