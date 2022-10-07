/*
 * Copyright 2022 wtcat
 */

#include <stm32_ll_pwr.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_system.h>
#include <stm32_ll_utils.h>

#include "drivers/clock.h"
#include "stm32/stm32_clock.h"


static int stm32_clock_enable(struct drvmgr_dev *dev, void *clk) {
    const struct stm32_pclken *pclken = (const struct stm32_pclken *)clk;
	switch (pclken->bus) {
	case STM32_CLOCK_BUS_AHB1:
		LL_AHB1_GRP1_EnableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_AHB2:
		LL_AHB2_GRP1_EnableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_AHB3:
		LL_AHB3_GRP1_EnableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_APB1:
		LL_APB1_GRP1_EnableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_APB2:
		LL_APB2_GRP1_EnableClock(pclken->enr);
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static int stm32_clock_disable(struct drvmgr_dev *dev, void *clk) {
    const struct stm32_pclken *pclken = (const struct stm32_pclken *)clk;
	switch (pclken->bus) {
	case STM32_CLOCK_BUS_AHB1:
		LL_AHB1_GRP1_DisableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_AHB2:
		LL_AHB2_GRP1_DisableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_AHB3:
		LL_AHB3_GRP1_EnableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_APB1:
		LL_APB1_GRP1_DisableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_APB2:
		LL_APB2_GRP1_DisableClock(pclken->enr);
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static const struct clock_driver_api *stm32_driver_ops = {
    .enable = stm32_clock_enable,
    .disable = stm32_clock_disable
};

static int stm32_clock_probe(struct drvmgr_dev *dev) {
    devp->devops = &stm32_driver_ops;
    return 0;
}

static const struct dev_id id_table[] = {
    {.compatible = "st,stm32-rcc", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops stm32_clk_driver_ops = {
	.init = {
		stm32_clock_probe,
	},
};
		
PLATFORM_DRIVER(stm32_clk) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "RCC",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &stm32_clk_driver_ops
	},
	.ids = id_table
};
