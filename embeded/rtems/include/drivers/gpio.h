/*
 * Copyright wtcat 2022
 */
#ifndef DRIVER_GPIOD_H_
#define DRIVER_GPIOD_H_

#include "drivers/devbase.h"

#ifdef __cplusplus
extern "C"{
#endif

/*
 * GPIO attributes
 */
#define GPIO_INTR_MASK(n)  ((n) & 0x000F)
#define GPIO_EDGE_RISING   (1ul << 0)
#define GPIO_EDGE_FALLING  (1ul << 1)
#define GPIO_EDGE_BOTH     (GPIO_EDGE_RISING | GPIO_EDGE_FALLING)
#define GPIO_LEVEL_HIGH    (1ul << 2)
#define GPIO_LEVEL_LOW     (1ul << 3)
#define GPIO_INTR(type)  (GPIO_INPUT | (type))

#define GPIO_MASK(n)  ((n) & 0xFF00)
#define GPIO_INPUT    (0x01u << 8)
#define GPIO_OUTPUT   (0x02u << 8)
#define GPIO_PULLUP   (0x04u << 8)
#define GPIO_PULLDOWN (0x08u << 8)
#define GPIO_OPENDRAIN (0x10u << 8)
#define GPIO_OUTPUT_INIT_HIGH (0x20u << 8)
#define GPIO_OUTPUT_INIT_LOW  (0x40u << 8)

struct gpio_pin {
    struct drvmgr_dev *dev;
    int polarity;
    int pin;
};

struct gpio_operations {
	int (*configure)(struct drvmgr_dev *dev, int pin, uint32_t mode);
    int (*set_port)(struct drvmgr_dev *dev, uint32_t mask, uint32_t value);
    int (*get_port)(struct drvmgr_dev *dev, uint32_t mask, uint32_t *value);
	int (*set_pin)(struct drvmgr_dev *dev, int pin, int val);
	int (*get_pin)(struct drvmgr_dev *dev, int pin);
    int (*toggle_pin)(struct drvmgr_dev *dev, int pin);
};

#define gpiod_get_ops(_dev) (const struct gpio_operations *)device_get_operations(_dev)

/*
 * GPIO bus interrupt requst interface
 */
static inline int gpiod_irq_request(struct drvmgr_dev *dev, int pin, 
    const char *info, drvmgr_isr isr, void *arg) {
    if (!dev || !dev->bus || !dev->bus->ops->int_register)
        return -1;
    return dev->bus->ops->int_register(dev, pin, info, isr, arg);
}

static inline int gpiod_irq_remove(struct drvmgr_dev *dev, int pin, 
    drvmgr_isr isr, void *arg) {
    if (!dev || !dev->bus || !dev->bus->ops->int_unregister)
        return -1;
    return dev->bus->ops->int_unregister(dev, pin, isr, arg);
}

static inline int gpiod_pin_irq_request(struct gpio_pin *pin, const char *info, 
    drvmgr_isr isr, void *arg) {
    return gpiod_irq_request(pin->dev, pin->pin, info, isr, arg);
}

static inline int gpiod_pin_irq_remove(struct gpio_pin *pin, drvmgr_isr isr, 
    void *arg) {
    return gpiod_irq_remove(pin->dev, pin->pin, isr, arg);
}


/*
 * GPIO generic interface
 */
static inline int gpiod_configure(struct drvmgr_dev *dev, int pin, 
	unsigned int mode) {
    _Assert(dev != NULL);
    _Assert(dev->priv != NULL);
    const struct gpio_operations *ops = gpiod_get_ops(dev);
    return ops->configure(dev, pin, mode);
}

static inline int gpiod_write(struct drvmgr_dev *dev, uint32_t mask, 
    uint32_t val) {
    _Assert(dev != NULL);
    _Assert(dev->priv != NULL);
    const struct gpio_operations *ops = gpiod_get_ops(dev);
    return ops->set_port(dev, mask, val);
}

static inline int gpiod_read(struct drvmgr_dev *dev, uint32_t mask, 
    uint32_t *val) {
    _Assert(dev != NULL);
    _Assert(dev->priv != NULL);
    const struct gpio_operations *ops = gpiod_get_ops(dev);
    return ops->get_port(dev, mask, val);
}

static inline int gpiod_setpin(struct drvmgr_dev *dev, int pin, 
	int val) {
    _Assert(dev != NULL);
    _Assert(dev->priv != NULL);
    const struct gpio_operations *ops = gpiod_get_ops(dev);
    return ops->set_pin(dev, pin, val);
}

static inline int gpiod_getpin(struct drvmgr_dev *dev, int pin) {
    _Assert(dev != NULL);
    _Assert(dev->priv != NULL);
    const struct gpio_operations *ops = gpiod_get_ops(dev);
    return ops->get_pin(dev, pin);
}

static inline int gpiod_toggle(struct drvmgr_dev *dev, int pin) {
    _Assert(dev != NULL);
    _Assert(dev->priv != NULL);
    const struct gpio_operations *ops = gpiod_get_ops(dev);
    return ops->toggle_pin(dev, pin);
}

static inline int gpiod_pin_assert(struct gpio_pin *pin) {
    return gpiod_setpin(pin->dev, pin->pin, pin->polarity);
}

static inline int gpiod_pin_deassert(struct gpio_pin *pin) {
    return gpiod_setpin(pin->dev, pin->pin, !pin->polarity);
}

static inline int gpiod_get_pin(struct gpio_pin *pin) {
    return gpiod_getpin(pin->dev, pin->pin);
}

static inline bool gpiod_is_active(struct gpio_pin *pin) {
    int val = gpiod_getpin(pin->dev, pin->pin);
    return val == pin->polarity;
}

#ifdef CONFIG_OFW
struct gpio_pin *__ofw_gpios_request(phandle_t np, const char *prop, 
    uint32_t flags, int *groups);
struct gpio_pin *ofw_cs_gpios_request(phandle_t np, uint32_t flags, int *ngroups);
struct gpio_pin *ofw_gpios_request(phandle_t np, uint32_t flags, int *ngroups);
#endif

#ifdef __cplusplus
}
#endif
#endif /* DRIVER_GPIOD_H_ */
