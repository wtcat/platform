#ifndef BSP_GPIOD_H_
#define BSP_GPIOD_H_

#include <drvmgr/drvmgr.h>

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

#define GPIO_MASK(n)  ((n) & 0x0F00)
#define GPIO_INPUT    (1ul << 8)
#define GPIO_OUTPUT   (2ul << 8)
#define GPIO_PULLUP   (3ul << 8)
#define GPIO_PULLDOWN (4ul << 8)

struct gpio_operations {
	int (*configure)(struct drvmgr_dev *dev, int pin, unsigned int mode);
	int (*set_pin)(struct drvmgr_dev *dev, int pin, int val);
	int (*get_pin)(struct drvmgr_dev *dev, int pin);
    void (*dump)(struct drvmgr_dev *dev);
};

static inline int gpiod_configure(struct drvmgr_dev *dev, int pin, 
	unsigned int mode) {
    _Assert(dev != NULL);
    _Assert(dev->priv != NULL);
    const struct gpio_operations *ops = *(void **)dev->priv;
    return ops->configure(dev, pin, mode);
}

static inline int gpiod_setpin(struct drvmgr_dev *dev, int pin, 
	int val) {
    _Assert(dev != NULL);
    _Assert(dev->priv != NULL);
    const struct gpio_operations *ops = *(void **)dev->priv;
    return ops->set_pin(dev, pin, val);
}

static inline int gpiod_getpin(struct drvmgr_dev *dev, int pin) {
    _Assert(dev != NULL);
    _Assert(dev->priv != NULL);
    const struct gpio_operations *ops = *(void **)dev->priv;
    return ops->get_pin(dev, pin);
}

static inline void gpiod_dump(struct drvmgr_dev *dev) {
    _Assert(dev != NULL);
    _Assert(dev->priv != NULL);
    const struct gpio_operations *ops = *(void **)dev->priv;
    if (ops->dump)
        ops->dump(dev);
}

#ifdef __cplusplus
}
#endif
#endif /* BSP_GPIOD_H_ */
