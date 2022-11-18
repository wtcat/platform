/*
 * Copyright 2022 wtcat
 */
#include <stdlib.h>

#include <rtems.h>
#include <rtems/imfs.h>
#include <rtems/malloc.h>

#include "base/workqueue.h"
#include "drivers/ofw_platform_bus.h"
#include "drivers/gpio.h"

// #define GPIO_KEYS_DEBUG 
#define GPIO_DEBOUNCE_TIME WQ_MSEC(10)

struct gpio_button {
    struct delayed_work_struct work;
    struct gpio_pin *btn;
    uint16_t code;
};

struct gpio_keys {
    int nr;
    struct gpio_button buttons[];
};

#ifdef GPIO_KEYS_DEBUG
#define devdbg(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define devdbg(...)
#endif

static void report_key(struct gpio_button *btn, bool pressed) {
    printk("GPIO-KEYS(%d): %s\n", (int)btn->code, 
        pressed? "PRESSED": "RELEASE");
}

static void __isr gpio_keys_work(struct work_struct *work) {
    struct gpio_button *btn = RTEMS_CONTAINER_OF(work, 
        struct gpio_button, work);
    report_key(btn, gpiod_is_active(btn->btn));
}

static void __isr gpio_keys_isr(void *arg) {
    (void) arg;
    struct gpio_button *btn = arg;
    schedule_delayed_work(&btn->work, GPIO_DEBOUNCE_TIME);
}

static int gpio_keys_preprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct gpio_keys *priv;
    phandle_t child;
    int nr = 0;

    ofw_foreach_child_node(devp->np, child) {
        if (rtems_ofw_has_prop(child, "gpios"))
            nr++;
    }
    priv = rtems_calloc(1, sizeof(struct gpio_keys) + nr*sizeof(struct gpio_button));
    if (!priv)
        return -ENOMEM;
    priv->nr = nr;
    dev->priv = priv;
    return 0;
}

static int gpio_keys_probe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct gpio_keys *priv = dev->priv;
    pcell_t cell;
    phandle_t child;
    int err = -EINVAL;
    int i, nr = 0;

    ofw_foreach_child_node(devp->np, child) {
        if (!rtems_ofw_has_prop(child, "gpios"))
            continue;
        if (rtems_ofw_get_enc_prop(child, "rtems,code", &cell, sizeof(cell)) < 0) {
            printk("%s: not found rtems,code\n", __func__);
            goto _freem;
        }
        priv->buttons[nr].btn = ofw_gpios_request(child, GPIO_INTR(GPIO_EDGE_BOTH), 
            NULL);
        if (!priv->buttons[nr].btn) {
            printk("%s: request gpios failed!\n", __func__);
            goto _freem;
        }
        err = gpiod_pin_irq_request(priv->buttons[nr].btn, "gpio-pin", 
            gpio_keys_isr, &priv->buttons[nr]);
        if (err) {
            printk("%s: gpio-pin request irq failed!(%d)\n", __func__, err);
            goto _freem;
        }
        delayed_work_init(&priv->buttons[nr].work, gpio_keys_work);
        devdbg("%s: code<%u>\n", __func__, cell);
        priv->buttons[nr].code = (uint16_t)cell;
        nr++;
    }
    return 0;

_freem:
    for (i = 0; i < nr; i++) {
        gpiod_pin_irq_remove(priv->buttons[nr].btn, gpio_keys_isr, 
            &priv->buttons[nr]);
        free(priv->buttons[nr].btn);
    }
    free(priv);
    return err;
}

static const struct dev_id id_table[] = {
    {.compatible = "gpio-keys", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops gpio_keys_driver = {
	.init = {
		gpio_keys_preprobe,
        NULL,
        gpio_keys_probe
	}
};
		
OFW_PLATFORM_DRIVER(gpio_keys) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "gpio-keys",
		.bus_type = DRVMGR_BUS_TYPE_ROOT,
		.ops      = &gpio_keys_driver
	},
	.ids = id_table
};
