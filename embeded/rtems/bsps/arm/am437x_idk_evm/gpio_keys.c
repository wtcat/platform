#include <errno.h>
#include <stdlib.h>

#include <rtems/bspIo.h>
#include <rtems/malloc.h>
#include <bsp.h>

#include "base/workqueue.h"
#include "drivers/gpio.h"
#include "drivers/platform_bus.h"


#define GPIO_DEBOUNCE_TIME WQ_MSEC(10)

struct gpio_keys_priv {
    struct delayed_work_struct work;
    struct drvmgr_dev *parent;
    uint16_t code;
    uint8_t pin;
    uint8_t polarity;
};

static void report_key(struct gpio_keys_priv *priv, bool pressed) {
    printk("GPIO-KEYS(%d): %s\n", (int)priv->code, 
        pressed? "PRESSED": "RELEASE");
}

static void gpio_keys_work(struct work_struct *work) {
    struct gpio_keys_priv *priv = RTEMS_CONTAINER_OF(work, 
        struct gpio_keys_priv, work.work);
    int status = gpiod_getpin(priv->parent, priv->pin);
    report_key(priv, status == (int)priv->polarity);
}

static void gpio_keys_isr(void *arg) {
    struct gpio_keys_priv *priv = arg;
    schedule_delayed_work(&priv->work, GPIO_DEBOUNCE_TIME);
}

static int gpio_keys_init(struct drvmgr_dev *dev) {
	struct gpio_keys_priv *priv;
    union drvmgr_key_value *prop;
    struct dev_private *devp;
	int ret;
    priv = rtems_calloc(1, sizeof(struct gpio_keys_priv));
    if (priv == NULL) {
        printk("Error***(%s): no more memory\n", __func__);
        return -DRVMGR_NOMEM;
    }
    devp = device_get_private(dev);
    prop = devcie_get_property(dev, "POLARITY");
    if (!prop) {
        printk("Error***(%s): Not found POLARITY property\n", __func__);
        goto _free;
    }
    priv->polarity = (uint8_t)prop->i;
    prop = devcie_get_property(dev, "CODE");
    if (!prop) {
        printk("Error***(%s): Not found CODE property\n", __func__);
        goto _free;
    }
    priv->code = (uint16_t)prop->i;
    priv->pin = (uint16_t)devp->base;
    priv->parent = device_get_parent(dev);
	dev->priv = priv;
    delayed_work_init(&priv->work, gpio_keys_work);
    ret = drvmgr_interrupt_register(dev, priv->pin, dev->name, 
        gpio_keys_isr, priv);
    if (ret) {
        printk("Error***(%s): install pin(%d) isr failed: %d\n", 
            __func__, priv->pin, ret);
        goto _free;
    }
    ret = gpiod_configure(priv->parent, priv->pin, GPIO_INTR(GPIO_EDGE_BOTH));
    if (ret) {
        printk("Error***(%s): configure pin(%d) failed: %d\n", 
            __func__, priv->pin, ret);
        goto _unregister;
    }
	return 0;
_unregister:
    drvmgr_interrupt_unregister(dev, priv->pin, gpio_keys_isr, priv);
_free:
    free(priv);
    return 0;
}

static const struct dev_id id_table[] = {
    {.compatible = "gpio-keys", NULL},
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
