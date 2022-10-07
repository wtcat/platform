/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <stdlib.h>

#include <rtems/bspIo.h>
#include <rtems/malloc.h>
#include <rtems/irq-extension.h>
#include <bsp.h>
#include <sys/errno.h>

#include "drivers/mio.h"
#include "drivers/gpio.h"
#include "drivers/clock.h"
#include "drivers/platform_bus.h"

#include "dt-bindings/pinctrl/stm32-pinctrl.h"
#include "stm32_ll_exti.h"

#undef BIT
#define BIT(n) (0x1ul << (n))
#define EXTI_NUMS RTEMS_ARRAY_SIZE(exti_irq_table)
#define stm32_pinval_get(pin) BIT(pin)

#if defined(CONFIG_SOC_SERIES_STM32F1X) || \
	defined(CONFIG_SOC_SERIES_STM32H7X) || \
	defined(CONFIG_SOC_SERIES_STM32L1X) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G4X) || \
	defined(CONFIG_SOC_SERIES_STM32WLX)
static const IRQn_Type exti_irq_table[] = {
	EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn,
	EXTI4_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn,
	EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn
};
#elif defined(CONFIG_SOC_STM32F410RX) /* STM32F410RX has no OTG_FS_WKUP_IRQn */
static const IRQn_Type exti_irq_table[] = {
	EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn,
	EXTI4_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn,
	EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	PVD_IRQn, 0xFF, 0xFF, 0xFF,
	0xFF, TAMP_STAMP_IRQn, RTC_WKUP_IRQn
};
#elif defined(CONFIG_SOC_SERIES_STM32F2X) || \
	defined(CONFIG_SOC_SERIES_STM32F4X)
static const IRQn_Type exti_irq_table[] = {
	EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn,
	EXTI4_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn,
	EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	PVD_IRQn, 0xFF, OTG_FS_WKUP_IRQn, 0xFF,
	0xFF, TAMP_STAMP_IRQn, RTC_WKUP_IRQn
};
#elif defined(CONFIG_SOC_SERIES_STM32F7X)
static const IRQn_Type exti_irq_table[] = {
	EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn,
	EXTI4_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn,
	EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	PVD_IRQn, 0xFF, OTG_FS_WKUP_IRQn, 0xFF,
	0xFF, TAMP_STAMP_IRQn, RTC_WKUP_IRQn, LPTIM1_IRQn
};
#endif

struct exti_cb {
	void (*cb)(void *arg);
	void *arg;
};

struct stm32_exti_data {
	struct exti_cb cb[EXTI_NUMS];
    rtems_interrupt_lock lock;
};

struct stm32_gpio_priv {
    rtems_interrupt_lock lock;
    struct stm32_exti_data *exti;
	void *base;
	int port;
};

static struct stm32_exti_data exti_private;

static int stm32_configure_gpio_intr(struct drvmgr_dev *dev,
    int pin, unsigned int mode);

static int stm32_flags_to_conf(int flags, int *pincfg) {
	if ((flags & GPIO_OUTPUT) != 0) {
		*pincfg = STM32_PINCFG_MODE_OUTPUT;
		if ((flags & GPIO_SINGLE_ENDED) != 0) {
			if (flags & GPIO_LINE_OPEN_DRAIN) 
				*pincfg |= STM32_PINCFG_OPEN_DRAIN;
			else
				return -ENOTSUP;
		} else {
			*pincfg |= STM32_PINCFG_PUSH_PULL;
		}
		if ((flags & GPIO_PULL_UP) != 0)
			*pincfg |= STM32_PINCFG_PULL_UP;
		else if ((flags & GPIO_PULL_DOWN) != 0) 
			*pincfg |= STM32_PINCFG_PULL_DOWN;
		
	} else if  ((flags & GPIO_INPUT) != 0) {
		*pincfg = STM32_PINCFG_MODE_INPUT;
		if ((flags & GPIO_PULL_UP) != 0)
			*pincfg |= STM32_PINCFG_PULL_UP;
		else if ((flags & GPIO_PULL_DOWN) != 0)
			*pincfg |= STM32_PINCFG_PULL_DOWN;
		else
			*pincfg |= STM32_PINCFG_FLOATING;
	} else {
		*pincfg = STM32_PINCFG_MODE_ANALOG;
	}
	return 0;
}

void stm32_gpio_setup(struct drvmgr_dev *dev, int pin, int conf, int altf) {
	struct stm32_gpio_priv *priv = dev->priv;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)priv->base;
	int pin_ll = stm32_pinval_get(pin);
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
	struct stm32_gpio_priv *priv = dev->priv;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)priv->base;
    rtems_interrupt_lock_context lock_context;
    rtems_interrupt_lock_acquire(&priv->lock, &lock_context);
    uint32_t pv = LL_GPIO_ReadInputPort(gpio);
    rtems_interrupt_lock_release(&priv->lock, &lock_context);
	return pv & mask;
}

static int stm32_gpio_setport(struct drvmgr_dev *dev, uint32_t mask, uint32_t value) {
	const struct stm32_gpio_priv *priv = dev->priv;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)priv->base;
    rtems_interrupt_lock_context lock_context;
    rtems_interrupt_lock_acquire(&priv->lock, &lock_context);
	uint32_t pv = LL_GPIO_ReadOutputPort(gpio);
	LL_GPIO_WriteOutputPort(gpio, mask & pv);
    rtems_interrupt_lock_release(&priv->lock, &lock_context);
	return 0;
}

static int stm32_gpio_setpin(struct drvmgr_dev *dev, int pin, int value) {
	struct stm32_gpio_priv *priv = dev->priv;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)priv->base;
    uint32_t rv = 0x1u << ((!value << 4) + pin);
    rtems_interrupt_lock_context lock_context;
    rtems_interrupt_lock_acquire(&priv->lock, &lock_context);
	gpio->BSRR = rv;
    rtems_interrupt_lock_release(&priv->lock, &lock_context);
	return 0;
}

static int stm32_gpio_getpin(struct drvmgr_dev *dev, int pin) {
	struct stm32_gpio_priv *priv = dev->priv;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)priv->base;
    rtems_interrupt_lock_context lock_context;
    rtems_interrupt_lock_acquire(&priv->lock, &lock_context);
    int val = gpio->ODR & BIT(pin);
    rtems_interrupt_lock_release(&priv->lock, &lock_context);
	return !val;
}

static int stm32_gpio_toggle_pin(struct drvmgr_dev *dev, uint32_t pin) {
	struct stm32_gpio_priv *priv = dev->priv;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)priv->base;
    rtems_interrupt_lock_context lock_context;
	rtems_interrupt_lock_acquire(&priv->lock, &lock_context);
	gpio->ODR ^= pin;
    rtems_interrupt_lock_release(&priv->lock, &lock_context);
	return 0;
}

static int stm32_gpio_configure(struct drvmgr_dev *dev, int pin, uint32_t flags) {
	int pincfg, err = 0;
	err = gpio_stm32_flags_to_conf(flags, &pincfg);
	if (err != 0) 
		goto exit;
	if ((flags & GPIO_OUTPUT) != 0) {
		if (flags & GPIO_OUTPUT_INIT_HIGH)
			gpio_stm32_port_set_bits_raw(dev, BIT(pin));
		else if (flags & GPIO_OUTPUT_INIT_LOW)
			gpio_stm32_port_clear_bits_raw(dev, BIT(pin));
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

static int stm32_enable_int(int port, int pin) {
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

static int stm32_gpio_bus_unite(struct drvmgr_drv *drv, struct drvmgr_dev *dev) {
	return platform_bus_match(drv, dev, DRVMGR_BUS_TYPE_GPIO);
}

static int stm32_gpio_bus_intr_register(struct drvmgr_dev *dev, int index, 
	const char *info, drvmgr_isr isr, void *arg) {
	(void) info;
    rtems_interrupt_lock_context lock_context;
    struct stm32_gpio_priv *platdata = device_get_parent_priv(dev);
	struct stm32_exti_data *exti = platdata->exti;
	if (index >= EXTI_NUMS || exti_irq_table[index] == 0xFF)
		return -DRVMGR_EINVAL;
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
    struct stm32_gpio_priv *platdata = device_get_parent_priv(dev);
	struct stm32_exti_data *exti = platdata->exti;
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
	(void) dev;
    rtems_interrupt_lock_context lock_context;
    struct stm32_gpio_priv *platdata = device_get_parent_priv(dev);
	struct stm32_exti_data *exti = platdata->exti;
	if (index >= EXTI_NUMS || exti_irq_table[index] == 0xFF)
		return -DRVMGR_EINVAL;
    rtems_interrupt_lock_acquire(&exti->lock, &lock_context);
	stm32_exti_clear_pending(index);
    rtems_interrupt_lock_release(&exti->lock, &lock_context);
	return DRVMGR_OK;
}

static int stm32_gpio_bus_intr_mask(struct drvmgr_dev *dev, int index) {
    (void) dev;
    rtems_interrupt_lock_context lock_context;
    struct stm32_gpio_priv *platdata = device_get_parent_priv(dev);
	struct stm32_exti_data *exti = platdata->exti;
	if (index >= EXTI_NUMS || exti_irq_table[index] == 0xFF)
		return -DRVMGR_EINVAL;
    rtems_interrupt_lock_acquire(&exti->lock, &lock_context);
    stm32_exti_disable(index);
    rtems_interrupt_lock_release(&exti->lock, &lock_context);
	return DRVMGR_OK;
}

static int stm32_gpio_bus_intr_unmask(struct drvmgr_dev *dev, int index) {
	(void) dev;
    rtems_interrupt_lock_context lock_context;
    struct stm32_gpio_priv *platdata = device_get_parent_priv(dev);
	struct stm32_exti_data *exti = platdata->exti;
	if (index >= EXTI_NUMS || exti_irq_table[index] == 0xFF)
		return -DRVMGR_EINVAL;
    rtems_interrupt_lock_acquire(&exti->lock, &lock_context);
	stm32_exti_enable(index);
    rtems_interrupt_lock_release(&exti->lock, &lock_context);
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
	const struct gpio_stm32_config *cfg = dev->config;
	struct gpio_stm32_data *data = dev->data;
	if (mode & (GPIO_LEVEL_HIGH|GPIO_LEVEL_LOW)) 
		return -ENOTSUP;
	gpio_stm32_enable_int(cfg->port, pin);
	stm32_exti_trigger(pin, GPIO_INTR_MASK(mode));
	return 0;
}

static void __stm32_exti_isr(int min, int max, struct stm32_exti_data *data) {
	for (int line = min; line < max; line++) {
		if (stm32_exti_is_pending(line)) {
			stm32_exti_clear_pending(line);
			data->cb[line].cb(data->cb[line].data);
		}
	}
}

static inline void __stm32_exti_isr_0(void *arg) {
	__stm32_exti_isr(0, 1, arg);
}

static inline void __stm32_exti_isr_1(void *arg) {
	__stm32_exti_isr(1, 2, arg);
}

static inline void __stm32_exti_isr_2(void *arg)
{
	__stm32_exti_isr(2, 3, arg);
}

static inline void __stm32_exti_isr_3(void *arg) {
	__stm32_exti_isr(3, 4, arg);
}

static inline void __stm32_exti_isr_4(void *arg) {
	__stm32_exti_isr(4, 5, arg);
}

static inline void __stm32_exti_isr_9_5(void *arg) {
	__stm32_exti_isr(5, 10, arg);
}

static inline void __stm32_exti_isr_15_10(void *arg) {
	__stm32_exti_isr(10, 16, arg);
}

#if defined(CONFIG_SOC_SERIES_STM32F4X) || \
	defined(CONFIG_SOC_SERIES_STM32F7X) || \
	defined(CONFIG_SOC_SERIES_STM32F2X) || \
	defined(CONFIG_SOC_SERIES_STM32MP1X)
static inline void __stm32_exti_isr_16(void *arg) {
	__stm32_exti_isr(16, 17, arg);
}

static inline void __stm32_exti_isr_18(void *arg) {
	__stm32_exti_isr(18, 19, arg);
}

static inline void __stm32_exti_isr_21(void *arg) {
	__stm32_exti_isr(21, 22, arg);
}

static inline void __stm32_exti_isr_22(void *arg) {
	__stm32_exti_isr(22, 23, arg);
}
#endif
#if defined(CONFIG_SOC_SERIES_STM32F7X) || \
	defined(CONFIG_SOC_SERIES_STM32MP1X)
static inline void __stm32_exti_isr_23(void *arg) {
	__stm32_exti_isr(23, 24, arg);
}
#endif

static int stm32_exti_connect_irqs(struct drvmgr_dev *dev) {
    int err = 0;
#if defined(CONFIG_SOC_SERIES_STM32F1X) || \
	defined(CONFIG_SOC_SERIES_STM32F2X) || \
	defined(CONFIG_SOC_SERIES_STM32F3X) || \
	defined(CONFIG_SOC_SERIES_STM32F4X) || \
	defined(CONFIG_SOC_SERIES_STM32F7X) || \
	defined(CONFIG_SOC_SERIES_STM32H7X) || \
	defined(CONFIG_SOC_SERIES_STM32L1X) || \
	defined(CONFIG_SOC_SERIES_STM32L4X) || \
	defined(CONFIG_SOC_SERIES_STM32L5X) || \
	defined(CONFIG_SOC_SERIES_STM32MP1X) || \
	defined(CONFIG_SOC_SERIES_STM32U5X) || \
	defined(CONFIG_SOC_SERIES_STM32WBX) || \
	defined(CONFIG_SOC_SERIES_STM32G4X) || \
	defined(CONFIG_SOC_SERIES_STM32WLX)
	err |= drvmgr_interrupt_register(dev, IRQF_HARD(EXTI0_IRQn), 
		"EXTI0_IRQn", __stm32_exti_isr_0, &exti_private);
	err |= drvmgr_interrupt_register(dev, IRQF_HARD(EXTI1_IRQn), 
		"EXTI1_IRQn", __stm32_exti_isr_1, &exti_private);
	err |= drvmgr_interrupt_register(dev, IRQF_HARD(EXTI2_IRQn), 
		"EXTI2_IRQn", __stm32_exti_isr_2, &exti_private);
	err |= drvmgr_interrupt_register(dev, IRQF_HARD(EXTI3_IRQn), 
		"EXTI3_IRQn", __stm32_exti_isr_3, &exti_private);
	err |= drvmgr_interrupt_register(dev, IRQF_HARD(EXTI4_IRQn), 
		"EXTI4_IRQn", __stm32_exti_isr_4, &exti_private);
	err |= drvmgr_interrupt_register(dev, IRQF_HARD(EXTI9_5_IRQn), 
		"EXTI9_5_IRQn", __stm32_exti_isr_9_5, &exti_private);
	err |= drvmgr_interrupt_register(dev, IRQF_HARD(EXTI15_10_IRQn), 
		"EXTI15_10_IRQn", __stm32_exti_isr_15_10, &exti_private);

#if defined(CONFIG_SOC_SERIES_STM32F2X) || \
	defined(CONFIG_SOC_SERIES_STM32F4X) || \
	defined(CONFIG_SOC_SERIES_STM32F7X)
	err |= drvmgr_interrupt_register(dev, IRQF_HARD(PVD_IRQn), 
		"PVD_IRQn", __stm32_exti_isr_16, &exti_private);

#if !defined(CONFIG_SOC_STM32F410RX)
	err |= drvmgr_interrupt_register(dev, IRQF_HARD(OTG_FS_WKUP_IRQn), 
		"OTG_FS_WKUP_IRQn", __stm32_exti_isr_18, &exti_private);

#endif
	err |= drvmgr_interrupt_register(dev, IRQF_HARD(TAMP_STAMP_IRQn), 
		"TAMP_STAMP_IRQn", __stm32_exti_isr_21, &exti_private);
	err |= drvmgr_interrupt_register(dev, IRQF_HARD(RTC_WKUP_IRQn), 
		"RTC_WKUP_IRQn", __stm32_exti_isr_22, &exti_private);
#endif

#if CONFIG_SOC_SERIES_STM32F7X
	err |= drvmgr_interrupt_register(dev, IRQF_HARD(LPTIM1_IRQn), 
		"LPTIM1_IRQn", __stm32_exti_isr_23, &exti_private);
#endif /* CONFIG_SOC_SERIES_STM32F7X */
#endif
    return err;
}

static void stm32_gpio_default_isr(void *arg) {
	(void) arg;
	printk("Please install GPIO ISR\n");
}

static void stm32_exit_init(struct drvmgr_dev *dev) {
    static bool inited = false;
    if (!inited) {
        rtems_interrupt_lock_initialize(&exti_private.lock, "EXTI");
        for (int i = 0; i < EXTI_NUMS; i++) {
            exti_private.cb[i].cb = stm32_gpio_default_isr;
            exti_private.cb[i].arg = NULL;
        }
        int err = stm32_exti_connect_irqs(dev);
        _Assert(err == 0);
        inited = true;
        (void)err;
    }
}

static const struct gpio_operations stm32_gpio_ops = {
	.configure = stm32_gpio_configure,
	.set_port = stm32_gpio_setport,
	.get_port = stm32_gpio_getport,
	.get_pin = stm32_gpio_getpin,
	.set_pin = stm32_gpio_setpin,
    .toggle_pin = stm32_gpio_toggle_pin
};

static struct drvmgr_bus_ops stm32_gpio_bus_ops = {
	.init = {
		platform_bus_populate,
	},
	.unite		    = stm32_gpio_bus_unite,
	.int_register	= stm32_gpio_bus_intr_register,
	.int_unregister	= stm32_gpio_bus_intr_unregister,
	.int_clear	    = stm32_gpio_bus_intr_clear,
	.int_mask	    = stm32_gpio_bus_intr_mask,
	.int_unmask	    = stm32_gpio_bus_intr_unmask,
};

static int stm32_gpio_probe(struct drvmgr_dev *dev) {
	union drvmgr_key_value *prop;
	struct drvmgr_dev *rcc;
	struct stm32_gpio_priv *priv;
    struct dev_private *devp;
	int ret;
	rcc = drvmgr_dev_by_name("/dev/RCC");
	if (!rcc)
		return -ENODEV;
	prop = devcie_get_property(dev, "clocks");
	if (!prop)
		return -ENOSTR;
    priv = rtems_calloc(1, sizeof(struct stm32_gpio_priv));
    if (priv == NULL) 
        return -DRVMGR_NOMEM;

    devp = device_get_private(dev);
	rtems_interrupt_lock_initialize(&priv->lock, dev->name);
	devp->devops = &stm32_gpio_ops;
    priv->base = devp->base;
    priv->exti = &exti_private;
	dev->priv = priv;
	clk_enable(rcc, prop->ptr);
    stm32_exit_init(dev);
	ret = platform_bus_device_register(dev, &stm32_gpio_bus_ops, 
		DRVMGR_BUS_TYPE_GPIO);
	if (ret) {
        free(priv);
		printk("Register GPIO bus device(%s) failed\n", dev->name);
	}
	return ret;
}

static const struct dev_id id_table[] = {
    {.compatible = "ti,am4372-gpio", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops stm32_gpio_driver_ops = {
	.init = {
		stm32_gpio_probe,
	},
};
		
PLATFORM_DRIVER(gpio_bus) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "gpio",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &stm32_gpio_driver_ops
	},
	.ids = id_table
};
