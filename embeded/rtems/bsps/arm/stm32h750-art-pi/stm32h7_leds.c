/*
 * Copyright 2022 wtcat
 */
#include <stdlib.h>

#include <rtems.h>
#include <rtems/imfs.h>
#include <rtems/malloc.h>

#include "drivers/ofw_platform_bus.h"
#include "drivers/gpio.h"
#include "drivers/led.h"


struct led_group {
    struct gpio_pin *leds;
    rtems_id timer;
    uint32_t delay_on;
    uint32_t delay_off;
    bool on;
};
struct leds_private {
    int ngrp;
    struct led_group grp[];
};


static void led_blink_timer_cb(rtems_id timer, void *arg) {
    struct led_group *grp = arg;
    struct gpio_pin *pin = grp->leds;
    uint32_t delay;

    if (!grp->on) {
        grp->on = true;
        delay = grp->delay_on;
        gpiod_setpin(pin->dev, pin->pin, pin->polarity);
    } else {
        grp->on = false;
        delay = grp->delay_off;
        gpiod_setpin(pin->dev, pin->pin, !pin->polarity);
    }
    rtems_timer_server_fire_after(timer, delay, led_blink_timer_cb, arg);
}

static int gpio_led_on(struct drvmgr_dev *dev, uint32_t led) {
    struct leds_private *priv = dev->priv;
    struct gpio_pin *pin;
    if ((int)led > priv->ngrp)
        return -EINVAL;
    pin = priv->grp[led].leds;
    return gpiod_setpin(pin->dev, pin->pin, pin->polarity);
}

static int gpio_led_off(struct drvmgr_dev *dev, uint32_t led) {
    struct leds_private *priv = dev->priv;
    struct led_group *grp;
    struct gpio_pin *pin;
    if ((int)led > priv->ngrp)
        return -EINVAL;
    grp = &priv->grp[led];
    pin = grp->leds;
    rtems_timer_delete(grp->timer);
    return gpiod_setpin(pin->dev, pin->pin, !pin->polarity);
}

static int gpio_led_blink(struct drvmgr_dev *dev, uint32_t led, 
    uint32_t delay_on, uint32_t delay_off) {
    struct leds_private *priv = dev->priv;
    struct led_group *grp;
    rtems_status_code sc;
 
    if ((int)led > priv->ngrp)
        return -EINVAL;

    grp = &priv->grp[led];
    rtems_timer_delete(grp->timer);
    sc = rtems_timer_create(rtems_build_name('L', 'E', 'D', 'x'), &grp->timer);
    if (sc != RTEMS_SUCCESSFUL)
        goto _exit;
    
    grp->delay_on = RTEMS_MILLISECONDS_TO_TICKS(delay_on);
    grp->delay_off = RTEMS_MILLISECONDS_TO_TICKS(delay_off);
    grp->on = false;
    gpio_led_off(dev, led);
    sc = rtems_timer_server_fire_after(grp->timer, grp->delay_off,
        led_blink_timer_cb, grp);
    if (sc != RTEMS_SUCCESSFUL)
        rtems_timer_delete(grp->timer);
_exit:
    return -rtems_status_code_to_errno(sc);
}
        
static struct led_operations led_ops = {
    .on = gpio_led_on,
    .off = gpio_led_off,
    .blink = gpio_led_blink
};

static int stm32h7_leds_preprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct leds_private *priv;
    phandle_t child;
    int ngrp = 0;

    ofw_foreach_child_node(devp->np, child) {
        if (rtems_ofw_has_prop(child, "gpios"))
            ngrp++;
    }
    priv = rtems_malloc(sizeof(*priv) + ngrp * sizeof(struct led_group));
    if (!priv)
        return -ENOMEM;
    priv->ngrp = ngrp;
    dev->priv = priv;
    return 0;
}

static int stm32h7_leds_probe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct leds_private *priv = dev->priv;
    phandle_t child;
    int i = 0;

    ofw_foreach_child_node(devp->np, child) {
        if (!rtems_ofw_has_prop(child, "gpios"))
            continue;
        priv->grp[i].leds = ofw_gpios_request(child, GPIO_OUTPUT, NULL);
        if (!priv->grp[i].leds)
            goto _freem;
    }
    devp->devops = &led_ops;
    return 0;

_freem:
    free(priv);
    return -ENODEV;
}

static const struct dev_id id_table[] = {
    {.compatible = "gpio-leds", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops stm32h7_leds_driver = {
	.init = {
		stm32h7_leds_preprobe,
        stm32h7_leds_probe
	},
};
		
OFW_PLATFORM_DRIVER(stm32h7_leds) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "gpio-leds",
		.bus_type = DRVMGR_BUS_TYPE_ROOT,
		.ops      = &stm32h7_leds_driver
	},
	.ids = id_table
};
