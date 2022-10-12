/*
 * Copyright 2022 wtcat
 */
#include <bsp.h>
#include <bsp/start.h>
#include <bsp/linker-symbols.h>
#include <bsp/irq-generic.h>

#include <rtems/bspIo.h>
#include <rtems/sysinit.h>
#include <rtems/score/armv7m.h>

#include "base/compiler.h"
#include "stm32h7xx.h"


extern char stm32h7_memory_null_begin[];
extern char stm32h7_memory_null_end[];
extern char stm32h7_memory_null_size[];

extern char stm32h7_memory_itcm_begin[];
extern char stm32h7_memory_itcm_end[];
extern char stm32h7_memory_itcm_size[];

extern char stm32h7_memory_flash_begin[];
extern char stm32h7_memory_flash_end[];
extern char stm32h7_memory_flash_size[];

extern char stm32h7_memory_dtcm_begin[];
extern char stm32h7_memory_dtcm_end[];
extern char stm32h7_memory_dtcm_size[];

extern char stm32h7_memory_sram_axi_begin[];
extern char stm32h7_memory_sram_axi_end[];
extern char stm32h7_memory_sram_axi_size[];

extern char stm32h7_memory_sram_1_begin[];
extern char stm32h7_memory_sram_1_end[];
extern char stm32h7_memory_sram_1_size[];

extern char stm32h7_memory_sram_2_begin[];
extern char stm32h7_memory_sram_2_end[];
extern char stm32h7_memory_sram_2_size[];

extern char stm32h7_memory_sram_3_begin[];
extern char stm32h7_memory_sram_3_end[];
extern char stm32h7_memory_sram_3_size[];

extern char stm32h7_memory_sram_4_begin[];
extern char stm32h7_memory_sram_4_end[];
extern char stm32h7_memory_sram_4_size[];

extern char stm32h7_memory_sram_backup_begin[];
extern char stm32h7_memory_sram_backup_end[];
extern char stm32h7_memory_sram_backup_size[];

extern char stm32h7_memory_peripheral_begin[];
extern char stm32h7_memory_peripheral_end[];
extern char stm32h7_memory_peripheral_size[];

extern char stm32h7_memory_sdram_1_begin[];
extern char stm32h7_memory_sdram_1_end[];
extern char stm32h7_memory_sdram_1_size[];

extern char stm32h7_memory_quadspi_begin[];
extern char stm32h7_memory_quadspi_end[];
extern char stm32h7_memory_quadspi_size[];

extern char stm32h7_memory_sdram_2_begin[];
extern char stm32h7_memory_sdram_2_end[];
extern char stm32h7_memory_sdram_2_size[];


static const ARMV7M_MPU_Region_config stm32h7_mpu_map[] = {
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

void __notrace bsp_start(void) {
    bsp_interrupt_initialize();
    rtems_cache_coherent_add_area(bsp_section_nocacheheap_begin,
        (uintptr_t)bsp_section_nocacheheap_size);
}

void __notrace bsp_start_hook_0(void) {
  // if ((RCC->AHB3ENR & RCC_AHB3ENR_FMCEN) == 0) {
  //   /*
  //    * Only perform the low-level initialization if necessary.  An initialized
  //    * FMC indicates that a boot loader already performed the low-level
  //    * initialization.
  //    */
  //   SystemInit();
  //   stm32h7_init_power();
  //   stm32h7_init_oscillator();
  //   stm32h7_init_clocks();
  //   stm32h7_init_peripheral_clocks();
  //   HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
  //   HAL_Init();
  //   SystemInit_ExtMemCtl();
  // }

  if ((SCB->CCR & SCB_CCR_IC_Msk) == 0) 
    SCB_EnableICache();
  if ((SCB->CCR & SCB_CCR_DC_Msk) == 0) 
    SCB_EnableDCache();
  _ARMV7M_MPU_Setup(stm32h7_mpu_map, RTEMS_ARRAY_SIZE(stm32h7_mpu_map));
}

void __notrace bsp_start_hook_1(void) {
  bsp_start_copy_sections_compact();
  SCB_CleanDCache();
  SCB_InvalidateICache();
  bsp_start_clear_bss();
}

/* For stm32_hal library */
uint32_t HAL_GetTick(void) {
  return rtems_clock_get_ticks_since_boot() *
    rtems_configuration_get_milliseconds_per_tick();
}

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
}

const void *bsp_fdt_get(void) {
  return NULL;
}