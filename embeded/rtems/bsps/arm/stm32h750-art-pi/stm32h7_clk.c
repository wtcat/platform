/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <rtems/malloc.h>
#include <rtems/rtems/intr.h>

#include "drivers/ofw_platform_bus.h" 
#include "drivers/clock.h"

#include "dt-bindings/clock/stm32h7-clks.h"
#include "dt-bindings/mfd/stm32h7-rcc.h"
#include "stm32h7xx_ll_bus.h"

static rtems_interrupt_lock lock RTEMS_UNUSED;

static int stm32h7_clk_enable(struct drvmgr_dev *dev, void *clk) {
	(void) dev;
	rtems_interrupt_lock_context context;
	int clkno = *(int *)clk;

	rtems_interrupt_lock_acquire(&lock, &context);
	switch (clkno) {
	case FLITF_CK:
		goto _notsup;
	case JPGDEC_CK:
		LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_JPGDEC);
		break;
	case DMA2D_CK:
		LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_DMA2D);
		break;
	case MDMA_CK:
		LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_MDMA);
		break;
	case USB2ULPI_CK:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_USB2OTGHSULPI);
		break;
	case USB1ULPI_CK:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_USB1OTGHSULPI);
		break;
	case ETH1RX_CK:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ETH1RX);
		break;
	case ETH1TX_CK:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ETH1TX);
		break;
	case ETH1MAC_CK:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ETH1MAC);
		break;
#if defined(DUAL_CORE)
	case ART_CK:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ART);
		break;
#endif
	case DMA2_CK:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
		break;
	case DMA1_CK:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
		break;
	case HASH_CK:
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_HASH);
		break;
	case CRYPT_CK:
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_CRYP);
		break;
	case CAMITF_CK:
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DCMI);
		break;
	case HSEM_CK:
		LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_HSEM);
		break;
	case BDMA_CK:
		LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_BDMA);
		break;
	case CRC_CK:
		LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_CRC);
		break;
	case GPIOK_CK:
		LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOK);
		break;
	case GPIOJ_CK:
		LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOJ);
		break;
	case GPIOH_CK:
		LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOH);
		break;
	case GPIOG_CK:
		LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG);
		break;
	case GPIOF_CK:
		LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOF);
		break;
	case GPIOE_CK:
		LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
		break;
	case GPIOD_CK:
		LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
		break;
	case GPIOC_CK:
		LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
		break;
	case GPIOB_CK:
		LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
		break;
	case GPIOA_CK:
		LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
		break;
	case WWDG1_CK:
		LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_WWDG1);
		break;
	case DAC12_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC12);
		break;
#if defined(DUAL_CORE)
	case WWDG2_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG2);
		break;
#endif
	case TIM14_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);
		break;
	case TIM13_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM13);
		break;
	case TIM12_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM12);
		break;
	case TIM7_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);
		break;
	case TIM6_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);
		break;
	case TIM5_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
		break;
	case TIM4_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
		break;
	case TIM3_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
		break;
	case TIM2_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
		break;
	case MDIOS_CK:
		LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_MDIOS);
		break;
	case OPAMP_CK:
		LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_OPAMP);
		break;
	case CRS_CK:
		LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_CRS);
		break;
	case TIM17_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM17);
		break;
	case TIM16_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);
		break;
	case TIM15_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM15);
		break;
	case TIM8_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
		break;
	case TIM1_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
		break;
	case TMPSENS_CK:
		goto _notsup;
	case RTCAPB_CK:
		LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_RTCAPB);
		break;
	case VREF_CK:
		LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_VREF);
		break;
	case COMP12_CK:
		LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_COMP12);
		break;
	case SYSCFG_CK:
		LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);
		break;
	case SDMMC1_CK:
		LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_SDMMC1);
		break;
	case QUADSPI_CK:
		LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_QSPI);
		break;
	case USB2OTG_CK:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_USB2OTGHS);
		break;
	case USB1OTG_CK:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_USB1OTGHS);
		break;
	case ADC12_CK:
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);
		break;
	case SDMMC2_CK:
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_SDMMC2);
		break;
	case RNG_CK:
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_RNG);
		break;
	case ADC3_CK:
		LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_ADC3);
		break;
#if defined(DSI)
	case DSI_CK:
		LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_DSI);
		break;
#endif
	case LTDC_CK:
		LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_LTDC);
		break;
	case USART8_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART8);
		break;
	case USART7_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART7);
		break;
	case HDMICEC_CK:
		goto _notsup;
	case I2C3_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);
		break;
	case I2C2_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
		break;
	case I2C1_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
		break;
	case UART5_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
		break;
	case UART4_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
		break;
	case USART3_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
		break;
	case USART2_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
		break;
	case SPDIFRX_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPDIFRX);
		break;
	case SPI3_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
		break;
	case SPI2_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
		break;
	case LPTIM1_CK:
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);
		break;
	case FDCAN_CK:
		LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_FDCAN);
		break;
	case SWP_CK:
		goto _notsup;
	case HRTIM_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_HRTIM);
		break;
	case DFSDM1_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_DFSDM1);
		break;
	case SAI3_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SAI3);
		break;
	case SAI2_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SAI2);
		break;
	case SAI1_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SAI1);
		break;
	case SPI5_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI5);
		break;
	case SPI4_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI4);
		break;
	case SPI1_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
		break;
	case USART6_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);
		break;
	case USART1_CK:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
		break;
	case SAI4B_CK:
	case SAI4A_CK:
		goto _notsup;
	case LPTIM5_CK:
		LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_LPTIM5);
		break;
	case LPTIM4_CK:
		LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_LPTIM4);
		break;
	case LPTIM3_CK:
		LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_LPTIM3);
		break;
	case LPTIM2_CK:
		LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_LPTIM2);
		break;
	case I2C4_CK:
		LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_I2C4);
		break;
	case SPI6_CK:
		LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SPI6);
		break;
	case LPUART1_CK:
		LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_LPUART1);
		break;
	default:
		goto _notsup;
	}
	rtems_interrupt_lock_release(&lock, &context);
	return 0;
_notsup:
	rtems_interrupt_lock_release(&lock, &context);
	return -ENOTSUP;
}

static int stm32h7_clk_disable(struct drvmgr_dev *dev, void *clk) {
	(void) dev;
	rtems_interrupt_lock_context context;
	int clkno = *(int *)clk;

	rtems_interrupt_lock_acquire(&lock, &context);
	switch (clkno) {
	case FLITF_CK:
		goto _notsup;
	case JPGDEC_CK:
		LL_AHB3_GRP1_DisableClock(LL_AHB3_GRP1_PERIPH_JPGDEC);
		break;
	case DMA2D_CK:
		LL_AHB3_GRP1_DisableClock(LL_AHB3_GRP1_PERIPH_DMA2D);
		break;
	case MDMA_CK:
		LL_AHB3_GRP1_DisableClock(LL_AHB3_GRP1_PERIPH_MDMA);
		break;
	case USB2ULPI_CK:
		LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_USB2OTGHSULPI);
		break;
	case USB1ULPI_CK:
		LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_USB1OTGHSULPI);
		break;
	case ETH1RX_CK:
		LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_ETH1RX);
		break;
	case ETH1TX_CK:
		LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_ETH1TX);
		break;
	case ETH1MAC_CK:
		LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_ETH1MAC);
		break;
#if defined(DUAL_CORE)
	case ART_CK:
		LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_ART);
		break;
#endif
	case DMA2_CK:
		LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_DMA2);
		break;
	case DMA1_CK:
		LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_DMA1);
		break;
	case HASH_CK:
		LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_HASH);
		break;
	case CRYPT_CK:
		LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_CRYP);
		break;
	case CAMITF_CK:
		LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_DCMI);
		break;
	case HSEM_CK:
		LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_HSEM);
		break;
	case BDMA_CK:
		LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_BDMA);
		break;
	case CRC_CK:
		LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_CRC);
		break;
	case GPIOK_CK:
		LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOK);
		break;
	case GPIOJ_CK:
		LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOJ);
		break;
	case GPIOH_CK:
		LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOH);
		break;
	case GPIOG_CK:
		LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOG);
		break;
	case GPIOF_CK:
		LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOF);
		break;
	case GPIOE_CK:
		LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
		break;
	case GPIOD_CK:
		LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
		break;
	case GPIOC_CK:
		LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
		break;
	case GPIOB_CK:
		LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
		break;
	case GPIOA_CK:
		LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
		break;
	case WWDG1_CK:
		LL_APB3_GRP1_DisableClock(LL_APB3_GRP1_PERIPH_WWDG1);
		break;
	case DAC12_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_DAC12);
		break;
#if defined(DUAL_CORE)
	case WWDG2_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_WWDG2);
		break;
#endif
	case TIM14_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM14);
		break;
	case TIM13_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM13);
		break;
	case TIM12_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM12);
		break;
	case TIM7_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM7);
		break;
	case TIM6_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM6);
		break;
	case TIM5_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM5);
		break;
	case TIM4_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM4);
		break;
	case TIM3_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM3);
		break;
	case TIM2_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM2);
		break;
	case MDIOS_CK:
		LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_MDIOS);
		break;
	case OPAMP_CK:
		LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_OPAMP);
		break;
	case CRS_CK:
		LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_CRS);
		break;
	case TIM17_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_TIM17);
		break;
	case TIM16_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_TIM16);
		break;
	case TIM15_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_TIM15);
		break;
	case TIM8_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_TIM8);
		break;
	case TIM1_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_TIM1);
		break;
	case TMPSENS_CK:
		goto _notsup;
	case RTCAPB_CK:
		LL_APB4_GRP1_DisableClock(LL_APB4_GRP1_PERIPH_RTCAPB);
		break;
	case VREF_CK:
		LL_APB4_GRP1_DisableClock(LL_APB4_GRP1_PERIPH_VREF);
		break;
	case COMP12_CK:
		LL_APB4_GRP1_DisableClock(LL_APB4_GRP1_PERIPH_COMP12);
		break;
	case SYSCFG_CK:
		LL_APB4_GRP1_DisableClock(LL_APB4_GRP1_PERIPH_SYSCFG);
		break;
	case SDMMC1_CK:
		LL_AHB3_GRP1_DisableClock(LL_AHB3_GRP1_PERIPH_SDMMC1);
		break;
	case QUADSPI_CK:
		LL_AHB3_GRP1_DisableClock(LL_AHB3_GRP1_PERIPH_QSPI);
		break;
	case USB2OTG_CK:
		LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_USB2OTGHS);
		break;
	case USB1OTG_CK:
		LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_USB1OTGHS);
		break;
	case ADC12_CK:
		LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_ADC12);
		break;
	case SDMMC2_CK:
		LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_SDMMC2);
		break;
	case RNG_CK:
		LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_RNG);
		break;
	case ADC3_CK:
		LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_ADC3);
		break;
#if defined(DSI)
	case DSI_CK:
		LL_APB3_GRP1_DisableClock(LL_APB3_GRP1_PERIPH_DSI);
		break;
#endif
	case LTDC_CK:
		LL_APB3_GRP1_DisableClock(LL_APB3_GRP1_PERIPH_LTDC);
		break;
	case USART8_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_UART8);
		break;
	case USART7_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_UART7);
		break;
	case HDMICEC_CK:
		goto _notsup;
	case I2C3_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I2C3);
		break;
	case I2C2_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I2C2);
		break;
	case I2C1_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I2C1);
		break;
	case UART5_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_UART5);
		break;
	case UART4_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_UART4);
		break;
	case USART3_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART3);
		break;
	case USART2_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART2);
		break;
	case SPDIFRX_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_SPDIFRX);
		break;
	case SPI3_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_SPI3);
		break;
	case SPI2_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_SPI2);
		break;
	case LPTIM1_CK:
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_LPTIM1);
		break;
	case FDCAN_CK:
		LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_FDCAN);
		break;
	case SWP_CK:
		goto _notsup;
	case HRTIM_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_HRTIM);
		break;
	case DFSDM1_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_DFSDM1);
		break;
	case SAI3_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SAI3);
		break;
	case SAI2_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SAI2);
		break;
	case SAI1_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SAI1);
		break;
	case SPI5_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SPI5);
		break;
	case SPI4_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SPI4);
		break;
	case SPI1_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SPI1);
		break;
	case USART6_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_USART6);
		break;
	case USART1_CK:
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_USART1);
		break;
	case SAI4B_CK:
	case SAI4A_CK:
		goto _notsup;
	case LPTIM5_CK:
		LL_APB4_GRP1_DisableClock(LL_APB4_GRP1_PERIPH_LPTIM5);
		break;
	case LPTIM4_CK:
		LL_APB4_GRP1_DisableClock(LL_APB4_GRP1_PERIPH_LPTIM4);
		break;
	case LPTIM3_CK:
		LL_APB4_GRP1_DisableClock(LL_APB4_GRP1_PERIPH_LPTIM3);
		break;
	case LPTIM2_CK:
		LL_APB4_GRP1_DisableClock(LL_APB4_GRP1_PERIPH_LPTIM2);
		break;
	case I2C4_CK:
		LL_APB4_GRP1_DisableClock(LL_APB4_GRP1_PERIPH_I2C4);
		break;
	case SPI6_CK:
		LL_APB4_GRP1_DisableClock(LL_APB4_GRP1_PERIPH_SPI6);
		break;
	case LPUART1_CK:
		LL_APB4_GRP1_DisableClock(LL_APB4_GRP1_PERIPH_LPUART1);
		break;
	default:
		goto _notsup;
	}
	rtems_interrupt_lock_release(&lock, &context);
	return 0;
_notsup:
	rtems_interrupt_lock_release(&lock, &context);
	return -ENOTSUP;
}

static int stm32h7_clk_get_rate(struct drvmgr_dev *dev, void *clk, uint32_t *rate) {
	(void) dev;
	(void) clk;
	(void) rate;
	return -ENOTSUP;
}

static const struct clock_driver_api stm32h7_clk_ops = {
    .enable = stm32h7_clk_enable,
    .disable = stm32h7_clk_disable,
    .get_rate = stm32h7_clk_get_rate
};

static const struct dev_id id_table[] = {
    {.compatible = "st,stm32-rcc", NULL},
    {NULL, NULL}
};

static int stm32h7_clk_preprobe(struct drvmgr_dev *dev) {
	struct dev_private *priv = device_get_private(dev);
	rtems_interrupt_lock_initialize(&lock, "rcc");
	priv->devops = &stm32h7_clk_ops;
	return 0;
}

static struct drvmgr_drv_ops clk_driver = {
	.init = {
		stm32h7_clk_preprobe
	}
};
		
OFW_PLATFORM_DRIVER(stm32h7_clk) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "rcc",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &clk_driver
	},
	.ids = id_table
};