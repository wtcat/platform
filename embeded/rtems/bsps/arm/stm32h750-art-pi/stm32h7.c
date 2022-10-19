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
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_fmc.h"
#include "stm32h7xx_ll_gpio.h"

/* memory mode register */
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

extern char stm32h7_memory_null_begin[];
extern char stm32h7_memory_null_end[];

extern char stm32h7_memory_sram_axi_begin[];
extern char stm32h7_memory_sram_axi_end[];

extern char stm32h7_memory_sdram_1_begin[];
extern char stm32h7_memory_sdram_1_end[];

extern char stm32h7_memory_sdram_2_begin[];
extern char stm32h7_memory_sdram_2_end[];


#define EARLY_UART UART4

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

static void __notrace stm32h7_sdram_setup_pin(GPIO_TypeDef *GPIOx, uint32_t Pin) {
  LL_GPIO_SetPinMode(GPIOx, Pin, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinOutputType(GPIOx, Pin, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(GPIOx, Pin, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetPinPull(GPIOx, Pin, LL_GPIO_PULL_UP);
  if (Pin < 8)
    LL_GPIO_SetAFPin_0_7(GPIOx, Pin, LL_GPIO_AF_12);
  else
    LL_GPIO_SetAFPin_8_15(GPIOx, Pin, LL_GPIO_AF_12);
}

static void __notrace stm32h7_sdram_setup_pins(void) {
  LL_AHB4_GRP1_EnableClock(
    LL_AHB4_GRP1_PERIPH_GPIOC | 
    LL_AHB4_GRP1_PERIPH_GPIOD |
    LL_AHB4_GRP1_PERIPH_GPIOE |
    LL_AHB4_GRP1_PERIPH_GPIOF |
    LL_AHB4_GRP1_PERIPH_GPIOG);
    
  /* GPIOC */
  stm32h7_sdram_setup_pin(GPIOC, LL_GPIO_PIN_2);
  stm32h7_sdram_setup_pin(GPIOC, LL_GPIO_PIN_3);

  /* GPIOD */
  stm32h7_sdram_setup_pin(GPIOD, LL_GPIO_PIN_0);
  stm32h7_sdram_setup_pin(GPIOD, LL_GPIO_PIN_1);
  stm32h7_sdram_setup_pin(GPIOD, LL_GPIO_PIN_8);
  stm32h7_sdram_setup_pin(GPIOD, LL_GPIO_PIN_9);
  stm32h7_sdram_setup_pin(GPIOD, LL_GPIO_PIN_10);
  stm32h7_sdram_setup_pin(GPIOD, LL_GPIO_PIN_14);
  stm32h7_sdram_setup_pin(GPIOD, LL_GPIO_PIN_15);

  /* GPIOE */
  stm32h7_sdram_setup_pin(GPIOE, LL_GPIO_PIN_0);
  stm32h7_sdram_setup_pin(GPIOE, LL_GPIO_PIN_1);
  stm32h7_sdram_setup_pin(GPIOE, LL_GPIO_PIN_7);
  stm32h7_sdram_setup_pin(GPIOE, LL_GPIO_PIN_8);
  stm32h7_sdram_setup_pin(GPIOE, LL_GPIO_PIN_9);
  stm32h7_sdram_setup_pin(GPIOE, LL_GPIO_PIN_10);
  stm32h7_sdram_setup_pin(GPIOE, LL_GPIO_PIN_11);
  stm32h7_sdram_setup_pin(GPIOE, LL_GPIO_PIN_12);
  stm32h7_sdram_setup_pin(GPIOE, LL_GPIO_PIN_13);
  stm32h7_sdram_setup_pin(GPIOE, LL_GPIO_PIN_14);
  stm32h7_sdram_setup_pin(GPIOE, LL_GPIO_PIN_15);

  /* GPIOF */
  stm32h7_sdram_setup_pin(GPIOF, LL_GPIO_PIN_0);
  stm32h7_sdram_setup_pin(GPIOF, LL_GPIO_PIN_1);
  stm32h7_sdram_setup_pin(GPIOF, LL_GPIO_PIN_2);
  stm32h7_sdram_setup_pin(GPIOF, LL_GPIO_PIN_3);
  stm32h7_sdram_setup_pin(GPIOF, LL_GPIO_PIN_4);
  stm32h7_sdram_setup_pin(GPIOF, LL_GPIO_PIN_5);
  stm32h7_sdram_setup_pin(GPIOF, LL_GPIO_PIN_11);
  stm32h7_sdram_setup_pin(GPIOF, LL_GPIO_PIN_12);
  stm32h7_sdram_setup_pin(GPIOF, LL_GPIO_PIN_13);
  stm32h7_sdram_setup_pin(GPIOF, LL_GPIO_PIN_14);
  stm32h7_sdram_setup_pin(GPIOF, LL_GPIO_PIN_15);

  /* GPIOG */
  stm32h7_sdram_setup_pin(GPIOG, LL_GPIO_PIN_0);
  stm32h7_sdram_setup_pin(GPIOG, LL_GPIO_PIN_1);
  stm32h7_sdram_setup_pin(GPIOG, LL_GPIO_PIN_2);
  stm32h7_sdram_setup_pin(GPIOG, LL_GPIO_PIN_4);
  stm32h7_sdram_setup_pin(GPIOG, LL_GPIO_PIN_5);
  stm32h7_sdram_setup_pin(GPIOG, LL_GPIO_PIN_8);
  stm32h7_sdram_setup_pin(GPIOG, LL_GPIO_PIN_15);

  /* GPIOH */
  stm32h7_sdram_setup_pin(GPIOH, LL_GPIO_PIN_5);
}

static void __notrace stm32h7_sdram_init(void) {
#define FMC_DEVICE FMC_SDRAM_DEVICE
  /* Timing configuration for W9825G6KH-6 */
  /* 100 MHz of HCKL3 clock frequency */
  FMC_SDRAM_TimingTypeDef sdram_timing = {
    .LoadToActiveDelay = 2, /* TMRD: 2 Clock cycles */
    .ExitSelfRefreshDelay = 8, /* TXSR: 8x10ns */
    .SelfRefreshTime = 6, /* TRAS: 5x10ns */
    .RowCycleDelay = 6, /* TRC:  7x10ns */
    .WriteRecoveryTime = 2, /* TWR:  2 Clock cycles */
    .RPDelay = 2, /* TRP:  2x10ns */
    .RCDDelay = 2 /* TRCD: 2x10ns */
  };
  FMC_SDRAM_InitTypeDef sdram_fmc = {
    .SDBank = FMC_SDRAM_BANK1,
    .InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4,
    .ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9,
    .RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13,
    .CASLatency = FMC_SDRAM_CAS_LATENCY_2,
    .MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16,
    .ReadBurst = FMC_SDRAM_RBURST_ENABLE,
    .ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0,
    .SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2,
    .WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE
  };
  FMC_SDRAM_CommandTypeDef sdram_cmd;
  volatile int temp;

  /* Enable the FMC interface clock */
  RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;

  stm32h7_sdram_setup_pins();
  FMC_SDRAM_DeInit(FMC_DEVICE, sdram_fmc.SDBank);
  FMC_SDRAM_Init(FMC_DEVICE, &sdram_fmc);
  FMC_SDRAM_Timing_Init(FMC_DEVICE, &sdram_timing, sdram_fmc.SDBank);

  /* Configure a clock configuration enable command */
  sdram_cmd.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
  sdram_cmd.CommandTarget = sdram_fmc.SDBank;
  sdram_cmd.AutoRefreshNumber = 1;
  sdram_cmd.ModeRegisterDefinition = 0;
  FMC_SDRAM_SendCommand(FMC_DEVICE, &sdram_cmd, 0x1000);
  for (temp = 0; temp < 0xffff; temp ++);

  /* Configure a PALL (precharge all) command */
  sdram_cmd.CommandMode = FMC_SDRAM_CMD_PALL;
  FMC_SDRAM_SendCommand(FMC_DEVICE, &sdram_cmd, 0x1000);

  /* Configure a Auto-Refresh command */
  sdram_cmd.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  sdram_cmd.AutoRefreshNumber = 8;
  FMC_SDRAM_SendCommand(FMC_DEVICE, &sdram_cmd, 0x1000);

  /* Program the external memory mode register */
  if (sdram_fmc.MemoryDataWidth == FMC_SDRAM_MEM_BUS_WIDTH_8)
    temp = SDRAM_MODEREG_BURST_LENGTH_1;
  else if (sdram_fmc.MemoryDataWidth == FMC_SDRAM_MEM_BUS_WIDTH_16)
    temp = SDRAM_MODEREG_BURST_LENGTH_2;
  else
    temp = SDRAM_MODEREG_BURST_LENGTH_4;
  if (sdram_fmc.CASLatency == FMC_SDRAM_CAS_LATENCY_3)
    temp |= SDRAM_MODEREG_CAS_LATENCY_3;
  else
    temp |= SDRAM_MODEREG_CAS_LATENCY_2;
  temp |= SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL | 
        SDRAM_MODEREG_OPERATING_MODE_STANDARD | 
        SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
  sdram_cmd.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  sdram_cmd.AutoRefreshNumber = 1;
  sdram_cmd.ModeRegisterDefinition = temp;
  FMC_SDRAM_SendCommand(FMC_DEVICE, &sdram_cmd, 0x1000);
  FMC_SDRAM_ProgramRefreshRate(FMC_DEVICE, 0x02A5);
}

static void __notrace stm32h7_early_uart_putc(char c) {
    while (!(EARLY_UART->ISR & USART_ISR_TXE_TXFNF));
    EARLY_UART->TDR = c;
}

void __notrace bsp_start(void) {
    bsp_interrupt_initialize();
    rtems_cache_coherent_add_area(bsp_section_nocacheheap_begin,
        (uintptr_t)bsp_section_nocacheheap_size);
}

void __notrace bsp_start_hook_0(void) {
  BSP_output_char = stm32h7_early_uart_putc;
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

const void *bsp_fdt_get(void) {
extern const unsigned char stm32h7_art_pi_dts[];
  return stm32h7_art_pi_dts;
}

uint32_t stm32h7_systick_frequency(void) {
  LL_RCC_ClocksTypeDef rcc_clks;
  LL_RCC_GetSystemClocksFreq(&rcc_clks);
  return rcc_clks.SYSCLK_Frequency;
}
