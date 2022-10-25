/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <rtems/malloc.h>
#include "drivers/gpio.h"
#include "drivers/ofw_platform_bus.h"

struct gpio_pin *ofw_cs_gpio_pin_request(phandle_t np, uint32_t flags, int *pol) {
    struct gpio_pin *gp = rtems_malloc(sizeof(*gp));
    if (gp) {
        gp->dev = ofw_cs_gpio_request(np, flags, &gp->pin, pol);
        if (!gp->dev) {
            free(gp);
            gp = NULL;
        }
    }
    return gp;
}

struct drvmgr_dev *ofw_cs_gpio_request(phandle_t np, uint32_t flags, int *pin, int *pol) {
    struct drvmgr_dev *dev = NULL;
    pcell_t args[32];

    if (rtems_ofw_get_enc_prop(np, "cs-gpios", args, sizeof(args)) < 0) {
        errno = -ENOSTR;
        return NULL;
    }
    dev = ofw_device_get_by_phandle(args[0]);
    if (!dev) {
        errno = -ENODEV;
        return NULL;
    }
    if (args[2])
        flags |= GPIO_OUTPUT_INIT_HIGH;
    else
        flags |= GPIO_OUTPUT_INIT_LOW;
    if (gpiod_configure(dev, args[1], flags|GPIO_OUTPUT)) {
        errno = -ENXIO;
        return NULL;
    }
    if (pin)
        *pin = args[1];
    if (*pol)
        *pol = args[2];

    return dev;
}
