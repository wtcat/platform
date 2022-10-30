/*
 * Copyright 2022 wtcat
 */
#include <stdlib.h>
#include <rtems/malloc.h>
#include <rtems/bspIo.h>
#include <rtems/sysinit.h>
#include <bsp/irq-generic.h>

#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_system.h"
#include "stm32/stm32_com.h"
#include "dt-bindings/pinctrl/stm32-pinctrl.h"

#undef GPIO_PULLUP
#undef GPIO_PULLDOWN
#include "drivers/ofw_platform_bus.h"
#include "drivers/gpio.h"
#include "drivers/clock.h"


#ifndef BIT
#define BIT(nr) (0x1ul << (nr))
#endif

struct exti_cb {
	void (*cb)(void *arg);
	void *arg;
};

struct stm32h7_exti {
#define EXTI_NUMS 16
	struct exti_cb cb[EXTI_NUMS];
    rtems_interrupt_lock lock;
};

struct stm32h7_gpio {
    rtems_interrupt_lock lock;
    struct stm32h7_exti *exti;
    GPIO_TypeDef *gpio;
    struct drvmgr_dev *clk;
    int clkid;
    int port;
};

static struct stm32h7_exti exti_controller __fastdata;
static const IRQn_Type exti_irq_table[] = {
	EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn,
	EXTI4_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn,
	EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn
};
RTEMS_STATIC_ASSERT(RTEMS_ARRAY_SIZE(exti_irq_table) == EXTI_NUMS, "");

static int stm32_configure_gpio_intr(struct drvmgr_dev *dev,
    int pin, unsigned int mode);

static int stm32_flags_to_conf(int flags, int *pincfg) {
	if ((flags & GPIO_OUTPUT) != 0) {
		*pincfg = STM32_MODER_OUTPUT_MODE;
		if ((flags & GPIO_OPENDRAIN)) {
			*pincfg |= STM32_OTYPER_OPEN_DRAIN;
		} else {
			*pincfg |= STM32_OTYPER_PUSH_PULL;
		}
		if ((flags & GPIO_PULLUP) != 0)
			*pincfg |= STM32_PUPDR_PULL_UP;
		else if ((flags & GPIO_PULLDOWN) != 0) 
			*pincfg |= STM32_PUPDR_PULL_DOWN;
		
	} else if  ((flags & GPIO_INPUT) != 0) {
		*pincfg = STM32_MODER_INPUT_MODE;
		if ((flags & GPIO_PULLUP) != 0)
			*pincfg |= STM32_PUPDR_PULL_UP;
		else if ((flags & GPIO_PULLDOWN) != 0)
			*pincfg |= STM32_PUPDR_PULL_DOWN;
		else
			*pincfg |= STM32_PUPDR_NO_PULL;
	} else {
		*pincfg = STM32_MODER_ANALOG_MODE;
	}
    *pincfg |= STM32_OSPEEDR_MEDIUM_SPEED;
	return 0;
}

int stm32_gpio_to_portno(struct drvmgr_dev *dev) {
    struct stm32h7_gpio *priv;
    if (dev == NULL || dev->priv)
        return -EINVAL;
    priv = dev->priv;
    return priv->port;
}

void stm32_gpio_setup(struct drvmgr_dev *dev, int pin, int conf, int altf) {
	struct stm32h7_gpio *priv = dev->priv;
	GPIO_TypeDef *gpio = priv->gpio;
	int pin_ll = BIT(pin);
	unsigned int mode = conf & (STM32_MODER_MASK << STM32_MODER_SHIFT);
	unsigned int otype = conf & (STM32_OTYPER_MASK << STM32_OTYPER_SHIFT);
	unsigned int ospeed = conf & (STM32_OSPEEDR_MASK << STM32_OSPEEDR_SHIFT);
	unsigned int pupd = conf & (STM32_PUPDR_MASK << STM32_PUPDR_SHIFT);

	LL_GPIO_SetPinOutputType(gpio, pin_ll, otype >> STM32_OTYPER_SHIFT);
	LL_GPIO_SetPinSpeed(gpio, pin_ll, ospeed >> STM32_OSPEEDR_SHIFT);
	LL_GPIO_SetPinPull(gpio, pin_ll, pupd >> STM32_PUPDR_SHIFT);
	if (mode == STM32_MODER_ALT_MODE) {
		if (pin < 8)
			LL_GPIO_SetAFPin_0_7(gpio, pin_ll, altf);
		else
			LL_GPIO_SetAFPin_8_15(gpio, pin_ll, altf);
	}
	LL_GPIO_SetPinMode(gpio, pin_ll, mode >> STM32_MODER_SHIFT);
}

static int stm32_gpio_getport(struct drvmgr_dev *dev, uint32_t mask, uint32_t *value) {
	struct stm32h7_gpio *priv = dev->priv;
	GPIO_TypeDef *gpio = priv->gpio;
    rtems_interrupt_lock_context lock_context;
    rtems_interrupt_lock_acquire(&priv->lock, &lock_context);
    uint32_t pv = LL_GPIO_ReadInputPort(gpio);
    rtems_interrupt_lock_release(&priv->lock, &lock_context);
	*value = pv & mask;
	return 0;
}

static int stm32_gpio_setport(struct drvmgr_dev *dev, uint32_t mask, uint32_t value) {
	const struct stm32h7_gpio *priv = dev->priv;
	GPIO_TypeDef *gpio = priv->gpio;
    rtems_interrupt_lock_context lock_context;
    rtems_interrupt_lock_acquire(&priv->lock, &lock_context);
	LL_GPIO_WriteOutputPort(gpio, mask & value);
    rtems_interrupt_lock_release(&priv->lock, &lock_context);
	return 0;
}

static int stm32_gpio_setpin(struct drvmgr_dev *dev, int pin, int value) {
	struct stm32h7_gpio *priv = dev->priv;
	GPIO_TypeDef *gpio = priv->gpio;
    uint32_t rv = 0x1u << ((!value << 4) + pin);
    rtems_interrupt_lock_context lock_context;
    rtems_interrupt_lock_acquire(&priv->lock, &lock_context);
	gpio->BSRR = rv;
    rtems_interrupt_lock_release(&priv->lock, &lock_context);
	return 0;
}

static int stm32_gpio_getpin(struct drvmgr_dev *dev, int pin) {
	struct stm32h7_gpio *priv = dev->priv;
	GPIO_TypeDef *gpio = priv->gpio;
    rtems_interrupt_lock_context lock_context;
    rtems_interrupt_lock_acquire(&priv->lock, &lock_context);
    int val = LL_GPIO_ReadInputPort(gpio) & BIT(pin);
    rtems_interrupt_lock_release(&priv->lock, &lock_context);
	return !!val;
}

static int stm32_gpio_toggle_pin(struct drvmgr_dev *dev, int pin) {
	struct stm32h7_gpio *priv = dev->priv;
	GPIO_TypeDef *gpio = priv->gpio;
    rtems_interrupt_lock_context lock_context;
	rtems_interrupt_lock_acquire(&priv->lock, &lock_context);
	gpio->ODR ^= pin;
    rtems_interrupt_lock_release(&priv->lock, &lock_context);
	return 0;
}

static int stm32_gpio_configure(struct drvmgr_dev *dev, int pin, uint32_t flags) {
	int pincfg = 0, err = 0;

	err = stm32_flags_to_conf(flags, &pincfg);
	if (err != 0) 
		goto exit;
	if ((flags & GPIO_OUTPUT) != 0) {
		if (flags & GPIO_OUTPUT_INIT_HIGH)
            stm32_gpio_setpin(dev, pin, 1);
		else if (flags & GPIO_OUTPUT_INIT_LOW)
            stm32_gpio_setpin(dev, pin, 0);
	}
	stm32_gpio_setup(dev, pin, pincfg, 0);
    if ((flags & GPIO_INPUT) && GPIO_INTR_MASK(flags))
        err = stm32_configure_gpio_intr(dev, pin, GPIO_INTR_MASK(flags));
exit:
	return err;
}

static inline uint32_t stm32_pin_to_exti_line(int pin) {
	return (0xF << ((pin % 4 * 4) + 16)) | (pin / 4);
}

static void stm32_set_exti_source(int port, int pin) {
	uint32_t line = stm32_pin_to_exti_line(pin);
	LL_SYSCFG_SetEXTISource(port, line);
}

static int stm32_enable_exit(int port, int pin) {
/* Enable SYSCFG clock */
#ifdef CONFIG_SOC_SERIES_STM32H7X
    LL_AHB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);
#else
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
#endif /* CONFIG_SOC_SERIES_STM32H7X */
	stm32_set_exti_source(port, pin);
	return 0;
}

static void stm32_exti_enable(int line) {
	/* Enable requested line interrupt */
#if defined(CONFIG_SOC_SERIES_STM32H7X) && defined(CONFIG_CPU_CORTEX_M4)
	LL_C2_EXTI_EnableIT_0_31(1 << line);
#else
	LL_EXTI_EnableIT_0_31(1 << line);
#endif
}

static void stm32_exti_disable(int line) {
	if (line < 32) {
#if defined(CONFIG_SOC_SERIES_STM32H7X) && defined(CONFIG_CPU_CORTEX_M4)
		LL_C2_EXTI_DisableIT_0_31(1 << line);
#else
		LL_EXTI_DisableIT_0_31(1 << line);
#endif
	}
}

static inline int stm32_exti_is_pending(int line) {
#if defined(CONFIG_SOC_SERIES_STM32H7X) && defined(CONFIG_CPU_CORTEX_M4)
    return LL_C2_EXTI_IsActiveFlag_0_31(1 << line);
#else
    return LL_EXTI_IsActiveFlag_0_31(1 << line);
#endif
}

static inline void stm32_exti_clear_pending(int line) {
#if defined(CONFIG_SOC_SERIES_STM32H7X) && defined(CONFIG_CPU_CORTEX_M4)
    LL_C2_EXTI_ClearFlag_0_31(1 << line);
#else
    LL_EXTI_ClearFlag_0_31(1 << line);
#endif
}

static void stm32_gpio_default_isr(void *arg) {
	(void) arg;
	printk("Please install GPIO ISR\n");
}

static int stm32_gpio_bus_unite(struct drvmgr_drv *drv, struct drvmgr_dev *dev) {
	return ofw_platform_bus_match(drv, dev, DRVMGR_BUS_TYPE_GPIO);
}

static int stm32_gpio_bus_intr_register(struct drvmgr_dev *dev, int index, 
	const char *info, drvmgr_isr isr, void *arg) {
	(void) info;
    rtems_interrupt_lock_context lock_context;
    struct stm32h7_gpio *priv = dev->priv;
	struct stm32h7_exti *exti = priv->exti;
	if (index >= EXTI_NUMS || exti_irq_table[index] == 0xFF) {
		printk("%s: gpio interrupt register failed: index(%d)\n", __func__, index);
		return -DRVMGR_EINVAL;
	}
	rtems_interrupt_lock_acquire(&exti->lock, &lock_context);
	exti->cb[index].cb = isr;
	exti->cb[index].arg = arg;
    stm32_exti_clear_pending(index);
    stm32_exti_enable(index);
	rtems_interrupt_lock_release(&exti->lock, &lock_context);
	return DRVMGR_OK;
}

static int stm32_gpio_bus_intr_unregister(struct drvmgr_dev *dev, int index, 
	drvmgr_isr isr, void *arg) {
	(void) isr;
	(void) arg;
    rtems_interrupt_lock_context lock_context;
    struct stm32h7_gpio *priv = dev->priv;
	struct stm32h7_exti *exti = priv->exti;
	if (index >= EXTI_NUMS || exti_irq_table[index] == 0xFF)
		return -DRVMGR_EINVAL;
	rtems_interrupt_lock_acquire(&exti->lock, &lock_context);
    stm32_exti_disable(index);
	exti->cb[index].cb = stm32_gpio_default_isr;
	exti->cb[index].arg = arg;
	rtems_interrupt_lock_release(&exti->lock, &lock_context);
	return DRVMGR_OK;
}
	
static int stm32_gpio_bus_intr_clear(struct drvmgr_dev *dev, int index) {
    rtems_interrupt_lock_context lock_context;
    struct stm32h7_gpio *priv = dev->priv;
	struct stm32h7_exti *exti = priv->exti;
	if (index >= EXTI_NUMS || exti_irq_table[index] == 0xFF)
		return -DRVMGR_EINVAL;
    rtems_interrupt_lock_acquire(&exti->lock, &lock_context);
	stm32_exti_clear_pending(index);
    rtems_interrupt_lock_release(&exti->lock, &lock_context);
    (void) exti;
	return DRVMGR_OK;
}

static int stm32_gpio_bus_intr_mask(struct drvmgr_dev *dev, int index) {
    rtems_interrupt_lock_context lock_context;
    struct stm32h7_gpio *priv = dev->priv;
	struct stm32h7_exti *exti = priv->exti;
	if (index >= EXTI_NUMS || exti_irq_table[index] == 0xFF)
		return -DRVMGR_EINVAL;
    rtems_interrupt_lock_acquire(&exti->lock, &lock_context);
    stm32_exti_disable(index);
    rtems_interrupt_lock_release(&exti->lock, &lock_context);
    (void) exti;
	return DRVMGR_OK;
}

static int stm32_gpio_bus_intr_unmask(struct drvmgr_dev *dev, int index) {
    rtems_interrupt_lock_context lock_context;
    struct stm32h7_gpio *priv = dev->priv;
	struct stm32h7_exti *exti = priv->exti;
	if (index >= EXTI_NUMS || exti_irq_table[index] == 0xFF)
		return -DRVMGR_EINVAL;
    rtems_interrupt_lock_acquire(&exti->lock, &lock_context);
	stm32_exti_enable(index);
    rtems_interrupt_lock_release(&exti->lock, &lock_context);
    (void)exti;
	return DRVMGR_OK;
}

static void stm32_exti_trigger(int line, int trigger) {
	_Assert(line < 32);
	switch (trigger) {
	case GPIO_EDGE_RISING:
		LL_EXTI_EnableRisingTrig_0_31(1 << line);
		LL_EXTI_DisableFallingTrig_0_31(1 << line);
		break;
	case GPIO_EDGE_FALLING:
		LL_EXTI_EnableFallingTrig_0_31(1 << line);
		LL_EXTI_DisableRisingTrig_0_31(1 << line);
		break;
	case GPIO_EDGE_BOTH:
		LL_EXTI_EnableRisingTrig_0_31(1 << line);
		LL_EXTI_EnableFallingTrig_0_31(1 << line);
		break;
	default:
		LL_EXTI_DisableRisingTrig_0_31(1 << line);
		LL_EXTI_DisableFallingTrig_0_31(1 << line);
        break;
	}
}

static int stm32_configure_gpio_intr(struct drvmgr_dev *dev,
    int pin, unsigned int mode) {
    struct stm32h7_gpio *priv = dev->priv;
	if (mode & (GPIO_LEVEL_HIGH|GPIO_LEVEL_LOW)) 
		return -ENOTSUP;
	stm32_enable_exit(priv->port, pin);
	stm32_exti_trigger(pin, GPIO_INTR_MASK(mode));
	return 0;
}

static void __fastcode __stm32_exti_isr(int min, int max, 
	struct stm32h7_exti *data) {
	for (int line = min; line < max; line++) {
		if (stm32_exti_is_pending(line)) {
			stm32_exti_clear_pending(line);
			data->cb[line].cb(data->cb[line].arg);
		}
	}
}

static inline void __fastcode __stm32_exti_isr_0(void *arg) {
	__stm32_exti_isr(0, 1, arg);
}

static inline void __fastcode __stm32_exti_isr_1(void *arg) {
	__stm32_exti_isr(1, 2, arg);
}

static inline void __fastcode __stm32_exti_isr_2(void *arg)
{
	__stm32_exti_isr(2, 3, arg);
}

static inline void __fastcode __stm32_exti_isr_3(void *arg) {
	__stm32_exti_isr(3, 4, arg);
}

static inline void __fastcode __stm32_exti_isr_4(void *arg) {
	__stm32_exti_isr(4, 5, arg);
}

static inline void __fastcode __stm32_exti_isr_9_5(void *arg) {
	__stm32_exti_isr(5, 10, arg);
}

static inline void __fastcode __stm32_exti_isr_15_10(void *arg) {
	__stm32_exti_isr(10, 16, arg);
}

static int stm32h7_exti_connect_irqs(void) {
    int err = 0;
	err |= rtems_interrupt_handler_install(EXTI0_IRQn, "EXTI0_IRQn", 
		RTEMS_INTERRUPT_UNIQUE, __stm32_exti_isr_0, &exti_controller);
	err |= rtems_interrupt_handler_install(EXTI1_IRQn, "EXTI1_IRQn", 
		RTEMS_INTERRUPT_UNIQUE, __stm32_exti_isr_1, &exti_controller);
	err |= rtems_interrupt_handler_install(EXTI2_IRQn, "EXTI2_IRQn", 
		RTEMS_INTERRUPT_UNIQUE, __stm32_exti_isr_2, &exti_controller);
	err |= rtems_interrupt_handler_install(EXTI3_IRQn, "EXTI3_IRQn", 
		RTEMS_INTERRUPT_UNIQUE, __stm32_exti_isr_3, &exti_controller);
	err |= rtems_interrupt_handler_install(EXTI4_IRQn, "EXTI4_IRQn", 
		RTEMS_INTERRUPT_UNIQUE, __stm32_exti_isr_4, &exti_controller);
	err |= rtems_interrupt_handler_install(EXTI9_5_IRQn, "EXTI9_5_IRQn", 
		RTEMS_INTERRUPT_UNIQUE, __stm32_exti_isr_9_5, &exti_controller);
	err |= rtems_interrupt_handler_install(EXTI15_10_IRQn, "EXTI15_10_IRQn", 
		RTEMS_INTERRUPT_UNIQUE, __stm32_exti_isr_15_10, &exti_controller);
    _Assert(err == 0);
    return err;
}

static void stm32h7_exti_init(void) {
    rtems_interrupt_lock_initialize(&exti_private.lock, "EXTI");
    for (int i = 0; i < EXTI_NUMS; i++) {
        exti_controller.cb[i].cb = stm32_gpio_default_isr;
        exti_controller.cb[i].arg = NULL;
    }
    stm32h7_exti_connect_irqs();
}

RTEMS_SYSINIT_ITEM(stm32h7_exti_init, 
    RTEMS_SYSINIT_BSP_PRE_DRIVERS, RTEMS_SYSINIT_ORDER_MIDDLE);

static const struct gpio_operations stm32_gpio_ops = {
	.configure = stm32_gpio_configure,
	.set_port = stm32_gpio_setport,
	.get_port = stm32_gpio_getport,
	.get_pin = stm32_gpio_getpin,
	.set_pin = stm32_gpio_setpin,
    .toggle_pin = stm32_gpio_toggle_pin
};

static struct drvmgr_bus_ops stm32h7_gpio_bus = {
	.init = {
		ofw_platform_bus_populate_device,
	},
	.unite		    = stm32_gpio_bus_unite,
	.int_register	= stm32_gpio_bus_intr_register,
	.int_unregister	= stm32_gpio_bus_intr_unregister,
	.int_clear	    = stm32_gpio_bus_intr_clear,
	.int_mask	    = stm32_gpio_bus_intr_mask,
	.int_unmask	    = stm32_gpio_bus_intr_unmask,
};

static int stm32_gpio_preprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    rtems_ofw_memory_area reg;
    struct stm32h7_gpio *priv;
    char bank_name[16];
    int ret;

    if (!rtems_ofw_has_prop(devp->np, "gpio-controller"))
        return -EINVAL;
    if (rtems_ofw_get_prop(devp->np, "st,bank-name", bank_name, sizeof(bank_name)) < 0)
        return -ENOSTR;
    priv = rtems_calloc(1, sizeof(struct stm32h7_gpio));
    if (priv == NULL) 
        return -ENOMEM;
    ret = rtems_ofw_get_reg(devp->np, &reg, sizeof(reg));
    if (ret < 0) {
        free(priv);
        ret = -ENOSTR;
    }
    priv->port = bank_name[4] - 'A';
    priv->gpio = (GPIO_TypeDef *)reg.start;
    priv->exti = &exti_controller;
    dev->priv = priv;
    devp->devops = &stm32_gpio_ops;
    return ofw_platform_bus_device_register(dev, &stm32h7_gpio_bus, 
    DRVMGR_BUS_TYPE_GPIO);
}

static int stm32_gpio_probe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct stm32h7_gpio *priv = dev->priv;
    int ret;
    ret = stm32_ofw_get_clkdev(devp->np, &priv->clk, &priv->clkid);
    if (ret) {
		printk("%s: request clock failed(%d)\n", __func__, ret);
        return ret;
	}
    /* Enable GPIO clock */
    clk_enable(priv->clk, &priv->clkid);
    return 0;
}

static struct drvmgr_drv_ops stm32h7_gpio_driver = {
	.init = {
        stm32_gpio_preprobe,
		stm32_gpio_probe,
	},
};
		
OFW_PLATFORM_DRIVER(stm32h7_gpio) = {
	.drv = {
		.drv_id   = DRIVER_PINCTRL_ID,
		.name     = "gpio",
		.bus_type = DRVMGR_BUS_TYPE_PINCTRL,
		.ops      = &stm32h7_gpio_driver
	}
};
