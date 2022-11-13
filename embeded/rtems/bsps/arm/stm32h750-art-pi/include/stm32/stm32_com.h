/*
 * Copyright 2022 wtcat
 */

#ifndef STM32_COM_H_
#define STM32_COM_H_

#include <ofw/ofw.h>

#ifdef __cplusplus
extern "C"{
#endif
struct drvmgr_dev;

int stm32_ofw_get_clkdev(phandle_t np, struct drvmgr_dev **clkdev, int *clkid);
int stm32_pinctrl_set_np(phandle_t np);

#ifdef __cplusplus
}
#endif
#endif /* STM32_COM_H_ */
