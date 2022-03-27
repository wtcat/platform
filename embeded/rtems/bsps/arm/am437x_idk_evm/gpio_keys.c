#include <errno.h>
#include <stdlib.h>

#include <rtems/bspIo.h>
#include <rtems/malloc.h>

#include <bsp.h>
#include "bsp/gpiod.h"
#include "bsp/platform_bus.h"
#include "component/workq.h"

#define GPIO_DEBOUNCE_TIME 10

struct gpio_keys_priv {
    struct work_delayed_struct work;
    struct drvmgr_dev *dev;
    uint16_t code;
    uint8_t pin;
    uint8_t polarity;
};

static void report_key(struct gpio_keys_priv *priv, bool pressed) {

}

static void gpio_keys_work(struct work_struct *work) {
    struct gpio_keys_priv *priv = RTEMS_CONTAINER_OF(work, 
        struct gpio_keys_priv, work.work);
    int status;
    if (!gpiod_getpin(priv->dev, priv->pin, &status))
        report_key(priv, status == (int)priv->polarity);
}

static void gpio_keys_isr(void *arg) {
    struct gpio_keys_priv *priv = arg;
    work_delayed_submit(&priv->work, GPIO_DEBOUNCE_TIME);
}

static int gpio_keys_init(struct drvmgr_dev *dev) {
	struct gpio_keys_priv *priv;
    union drvmgr_key_value *prop;
    struct dev_private *devp;
	int ret;
    priv = rtems_calloc(1, sizeof(struct gpio_keys_priv));
    if (priv == NULL) 
        return -DRVMGR_NOMEM;
    devp = device_get_private(dev);
    prop = devcie_get_property(dev, "POLARITY");
    if (!prop) {
        priv->polarity = (uint8_t)prop->i;
        goto _free;
    }
    prop = devcie_get_property(dev, "CODE");
    if (!prop) {
        priv->code = (uint16_t)prop->i;
        goto _free;
    }
    priv->pin = (uint16_t)devp->base;
    priv->dev = dev;
	dev->priv = priv;
    work_delayed_init(&priv->work, gpio_keys_work);
    ret = drvmgr_interrupt_register(dev, priv->pin, dev->name, 
        gpio_keys_isr, priv);
    if (ret)
        goto _free;
    ret = gpiod_configure(dev, priv->pin, GPIO_INTR(GPIO_EDGE_BOTH));
    if (ret) 
        goto _unregister;
	return 0;
_unregister:
    drvmgr_interrupt_unregister(dev, priv->pin, gpio_keys_isr, priv);
_free:
    free(priv);
    return 0;
}

static const struct dev_id id_table[] = {
    {.compatible = "gpio_keys", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops gpio_keys_driver_ops = {
	.init = {
		gpio_keys_init,
	},
};
		
PLATFORM_DRIVER(gpio_keys) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "gpio_keys",
		.bus_type = DRVMGR_BUS_TYPE_GPIO,
		.ops      = &gpio_keys_driver_ops
	},
	.ids = id_table
};
