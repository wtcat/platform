/*
 * Copyright 2021-2022 wtcat
 */
#include <stm32_ll_bus.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_system.h>
#include <stm32_ll_utils.h>

#include "drivers/platform_bus.h"
#include "dt-bindings/pinctrl/stm32-pinctrl.h"
#include "stm32/stm32_clock.h"

#define nIRQ(n) (n)

PLATFORM_RESOURCE(rcc, "st,stm32-rcc",
  	TRN("REG0", DRVMGR_KT_INT, 0)
);

/*
 * UARTn
 */
static const struct stm32_pclken uart_clocks[] = {
	{STM32_CLOCK_BUS_APB2, LL_APB2_GRP1_PERIPH_USART1},
	{STM32_CLOCK_BUS_APB1, LL_APB1_GRP1_PERIPH_USART2},
	{STM32_CLOCK_BUS_APB1, LL_APB1_GRP1_PERIPH_USART3},
	{STM32_CLOCK_BUS_APB1, LL_APB1_GRP1_PERIPH_UART4},
	{STM32_CLOCK_BUS_APB1, LL_APB1_GRP1_PERIPH_UART5},
	{STM32_CLOCK_BUS_APB2, LL_APB2_GRP1_PERIPH_USART6},
	{STM32_CLOCK_BUS_APB1, LL_APB1_GRP1_PERIPH_UART7}
};
static const struct stm32_dma_channel uart1_dma_channel = {
	{1, 2, 4}, //UART1 RX
	{1, 7, 4}  //UART1 TX
};
PLATFORM_RESOURCE(ttyS0, "st,stm32-uart",
  	TRN("REG0", DRVMGR_KT_INT, 0x40011000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(37)),
	TRN("dmarx-channel", DRVMGR_KT_POINTER, (void *)&uart1_dma_channel[0]),
	TRN("dmatx-channel", DRVMGR_KT_POINTER, (void *)&uart1_dma_channel[1]),
	TRN("clocks", DRVMGR_KT_POINTER, &uart_clocks[0]),
	TRN("stdout", DRVMGR_KT_INT, 921600)
);


/*
 * GPIO
 */
static const struct stm32_pclken gpio_clocks[] = {
	{STM32_CLOCK_BUS_AHB1, LL_AHB1_GRP1_PERIPH_GPIOA},
	{STM32_CLOCK_BUS_AHB1, LL_AHB1_GRP1_PERIPH_GPIOB},
	{STM32_CLOCK_BUS_AHB1, LL_AHB1_GRP1_PERIPH_GPIOC},
	{STM32_CLOCK_BUS_AHB1, LL_AHB1_GRP1_PERIPH_GPIOD},
	{STM32_CLOCK_BUS_AHB1, LL_AHB1_GRP1_PERIPH_GPIOE},
	{STM32_CLOCK_BUS_AHB1, LL_AHB1_GRP1_PERIPH_GPIOF},
	{STM32_CLOCK_BUS_AHB1, LL_AHB1_GRP1_PERIPH_GPIOG}
};

PLATFORM_RESOURCE(gpio_a, "st,stm32-gpio",
  	TRN("REG0", DRVMGR_KT_INT, 0x40020000),
	TRN("clocks", DRVMGR_KT_POINTER, &gpio_clocks[0]),
);
PLATFORM_RESOURCE(gpio_b, "st,stm32-gpio",
  	TRN("REG0", DRVMGR_KT_INT, 0x40020400),
	TRN("clocks", DRVMGR_KT_POINTER, &gpio_clocks[1]),
);
PLATFORM_RESOURCE(gpio_c, "st,stm32-gpio",
  	TRN("REG0", DRVMGR_KT_INT, 0x40020800),
	TRN("clocks", DRVMGR_KT_POINTER, &gpio_clocks[2]),
);
PLATFORM_RESOURCE(gpio_d, "st,stm32-gpio",
  	TRN("REG0", DRVMGR_KT_INT, 0x40020c00),
	TRN("clocks", DRVMGR_KT_POINTER, &gpio_clocks[3]),
);
PLATFORM_RESOURCE(gpio_e, "st,stm32-gpio",
  	TRN("REG0", DRVMGR_KT_INT, 0x40021000),
	TRN("clocks", DRVMGR_KT_POINTER, &gpio_clocks[4]),
);
PLATFORM_RESOURCE(gpio_f, "st,stm32-gpio",
  	TRN("REG0", DRVMGR_KT_INT, 0x40021400),
	TRN("clocks", DRVMGR_KT_POINTER, &gpio_clocks[5]),
);
PLATFORM_RESOURCE(gpio_g, "st,stm32-gpio",
  	TRN("REG0", DRVMGR_KT_INT, 0x40021800),
	TRN("clocks", DRVMGR_KT_POINTER, &gpio_clocks[6]),
);

/*
 * User button
 */
TEMPLATE_RESOURCE(gpio_keys, "gpio-keys", DRVMGR_BUS_TYPE_GPIO, gpio4,
  	TRN("REG0", DRVMGR_KT_INT, 2),
	TRN("POLARITY", DRVMGR_KT_INT, 0),
	TRN("CODE", DRVMGR_KT_INT, 0x10)
);

/*
 * DMA
 */
static const struct stm32_pclken dma_clocks[] = {
	{STM32_CLOCK_BUS_AHB1, LL_AHB1_GRP1_PERIPH_DMA1},
	{STM32_CLOCK_BUS_AHB1, LL_AHB1_GRP1_PERIPH_DMA2}
};
PLATFORM_RESOURCE(dma_0, "st,stm32-dma",
  	TRN("REG0", DRVMGR_KT_INT, 0x40026000),
	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(11)),
	TRN("IRQ1", DRVMGR_KT_INT, nIRQ(12)),
	TRN("IRQ2", DRVMGR_KT_INT, nIRQ(13)),
	TRN("IRQ3", DRVMGR_KT_INT, nIRQ(14)),
	TRN("IRQ4", DRVMGR_KT_INT, nIRQ(15)),
	TRN("IRQ5", DRVMGR_KT_INT, nIRQ(16)),
	TRN("IRQ6", DRVMGR_KT_INT, nIRQ(17)),
	TRN("IRQ7", DRVMGR_KT_INT, nIRQ(47)),
	TRN("clocks", DRVMGR_KT_POINTER, &dma_clocks[0]) // STM32F4_AHB1_CLOCK(DMA1)
);

PLATFORM_RESOURCE(dma_1, "st,stm32-dma",
  	TRN("REG0", DRVMGR_KT_INT, 0x40026400),
	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(56)),
	TRN("IRQ1", DRVMGR_KT_INT, nIRQ(57)),
	TRN("IRQ2", DRVMGR_KT_INT, nIRQ(58)),
	TRN("IRQ3", DRVMGR_KT_INT, nIRQ(59)),
	TRN("IRQ4", DRVMGR_KT_INT, nIRQ(60)),
	TRN("IRQ5", DRVMGR_KT_INT, nIRQ(68)),
	TRN("IRQ6", DRVMGR_KT_INT, nIRQ(69)),
	TRN("IRQ7", DRVMGR_KT_INT, nIRQ(70)),
	TRN("clocks", DRVMGR_KT_POINTER, &dma_clocks[1]), //STM32F4_AHB1_CLOCK(DMA2)
	TRN("st,mem2mem", DRVMGR_KT_INT, 0)
);

/*
 * Pin-ctrl
 */
static uint32_t pinctrl_pad[] = {
	STM32_PINMUX('A', 9, STM32_AF7),

	PINCTRL_TERMINAL
};

PLATFORM_RESOURCE(pin_ctrl, "pinctrl-single,pins",
  	TRN("REG0", DRVMGR_KT_INT, CTRL_MODULE),
	TRN("pins", DRVMGR_KT_POINTER, am437x_pads)
);

TEMPLATE_RESOURCES_REGISTER(platform_resources,
	RN(rcc),
	RN(pin_ctrl),
	RN(dma_0), RN(dma_1),
    RN(ttyS0), RN(ttyS1), RN(ttyS2), RN(ttyS3), RN(ttyS4), 
	RN(gpio_a), RN(gpio_b), RN(gpio_c), RN(gpio_d), RN(gpio_e), RN(gpio_f), RN(gpio_g),
	NULL
);
