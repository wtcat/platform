/*
 * Copyright 2022 wtcat
 */

#ifndef STM32_CLOCK_H_
#define STM32_CLOCK_H_

#include <stdint.h>
#include "stm32h7xx_ll_bus.h"

#ifdef __cplusplus
extern "C"{
#endif

/* clock bus references */
#define STM32_CLOCK_BUS_AHB1    0
#define STM32_CLOCK_BUS_AHB2    1
#define STM32_CLOCK_BUS_APB1    2
#define STM32_CLOCK_BUS_APB2    3
#define STM32_CLOCK_BUS_IOP     5
#define STM32_CLOCK_BUS_AHB3    6
#define STM32_CLOCK_BUS_AHB4    7
#define STM32_CLOCK_BUS_AHB5    8
#define STM32_CLOCK_BUS_AHB6    9
#define STM32_CLOCK_BUS_APB3    10
#define STM32_CLOCK_BUS_APB4    11
#define STM32_CLOCK_BUS_APB5    12
#define STM32_CLOCK_BUS_AXI     13
#define STM32_CLOCK_BUS_MLAHB   14

struct stm32_pclken {
	uint32_t bus;
	uint32_t enr;
};

#ifdef __cplusplus
}
#endif
#endif /* STM32_CLOCK_H_ */
