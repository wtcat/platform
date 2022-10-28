/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <rtems/malloc.h>
#include <rtems/bspIo.h>
#include "drivers/gpio.h"
#include "drivers/ofw_platform_bus.h"


#ifdef OFW_GPIO_DEBUG
#define devdbg(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define devdbg(...)
#endif /* OFW_GPIO_DEBUG */

static struct gpio_pin *__ofw_gpios_request(phandle_t np, const char *prop, 
    uint32_t flags, int *groups) {
    struct gpio_pin *pin_group;
    int len, unit;
    pcell_t *args;
    int ngpios;

    len = rtems_ofw_get_enc_prop_alloc(np, prop, (void **)&args);
    if (len < 0) {
        errno = -ENOSTR;
        return NULL;
    }
    
    unit = 3 * sizeof(pcell_t);
    if (len % unit) {
        errno = -EINVAL;
        goto _free_args;
    }

    ngpios = len / unit;
    pin_group = rtems_malloc(ngpios * sizeof(struct gpio_pin));
    if (!pin_group) {
        errno = -ENOMEM;
        goto _free_args;
    }
    for (int i = 0; i < ngpios; i++) {
        pin_group[i].pin = args[i*3 + 1];
        pin_group[i].polarity = args[i*3 + 2];
        pin_group[i].dev = ofw_device_get_by_phandle(args[i*3]);
        if (!pin_group[i].dev) {
            errno = -ENODEV;
            goto _free_pins;
        }
        if (flags & GPIO_OUTPUT) {
            if (pin_group[i].polarity)
                flags |= GPIO_OUTPUT_INIT_LOW;
            else
                flags |= GPIO_OUTPUT_INIT_HIGH;
        }
        devdbg("%s: %s pin(%d) polarity(%d) **\n", __func__,
            pin_group[i].dev->name,
            pin_group[i].pin,
            pin_group[i].polarity);
        if (gpiod_configure(pin_group[i].dev, pin_group[i].pin, flags)) {
            errno = -EIO;
            goto _free_pins;
        }
    }

    if (groups)
        *groups = ngpios;
    free(args);
    return pin_group;

_free_pins:
    free(pin_group);
_free_args:
    free(args);
    return NULL;
}

struct gpio_pin *ofw_cs_gpios_request(phandle_t np, uint32_t flags, int *ngroups) {
    return __ofw_gpios_request(np, "cs-gpios", 
    flags | GPIO_OUTPUT, ngroups);
}

struct gpio_pin *ofw_gpios_request(phandle_t np, uint32_t flags, int *ngroups) {
    return __ofw_gpios_request(np, "gpios", 
    flags | GPIO_OUTPUT, ngroups);
}
