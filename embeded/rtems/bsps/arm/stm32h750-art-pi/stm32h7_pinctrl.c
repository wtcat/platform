/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <stdlib.h>
#include <rtems/malloc.h>

#include "drivers/pinctrl.h"
#include "drivers/ofw_platform_bus.h"

#include "dt-bindings/pinctrl/stm32-pinctrl.h"


struct stm32h7_pinctrl {
    int nr;
    struct drvmgr_dev *gpios[];
};

static int ofw_pinctrl_bus_filter(phandle_t np, char *devname, size_t max) {
	int len;
	len = rtems_ofw_get_prop(np, "rtems,path", devname, max);
	if (len <= 0)
		return -EINVAL;
	return len;
}

static int stm32h7_setup_pinctrl(struct drvmgr_dev *pinctrl, phandle_t np) {
extern void stm32_gpio_setup(struct drvmgr_dev *dev, int pin, int conf, int altf);
    struct stm32h7_pinctrl *priv = pinctrl->priv;
    pcell_t parent, child;
    pcell_t ospeed = 0;
    pcell_t pins[32];
    int conf = 0;

    if (rtems_ofw_get_enc_prop(np, "pinctrl-0", &parent, sizeof(parent)) < 0)
        return -ENOSTR;

    ofw_foreach_child_node(parent, child) {
        int n = rtems_ofw_get_enc_prop(child, "pinmux", pins, sizeof(pins));
        if (n < 0)
            return -ENOSTR;

        rtems_ofw_get_enc_prop(child, "slew-rate", &ospeed, sizeof(ospeed));
        if (rtems_ofw_has_prop(child, "drive-push-pull"))
            conf = STM32_OTYPER_PUSH_PULL;

        for (int i = 0; i < n; i++) {
            int af = pins[i] & 0xFF;
            int port = (pins[i] >> 12) & 0xF;
            int pin = (pins[i] >> 8) & 0xF;       
            conf |= (ospeed << STM32_OTYPER_SHIFT) | STM32_MODER_ALT_MODE;
            if (port > priv->nr || !priv->gpios[port]) {
                printk("Invalid GPIO device port (%d)\n", port);
                return -EINVAL;
            }
            stm32_gpio_setup(priv->gpios[port], pin, conf, af);
        }
    }
    return 0;
}

static int stm32h7_pinctrl_set(struct drvmgr_dev *dev, struct drvmgr_dev *config) {
    struct dev_private *devp = device_get_private(config);
    return stm32h7_setup_pinctrl(dev, devp->np);
}

static int stm32h7_pinctrl_bus_unite(struct drvmgr_drv *drv, struct drvmgr_dev *dev) {
	return ofw_platform_bus_match(drv, dev, DRVMGR_BUS_TYPE_PINCTRL);
}

static const struct pinctrl_operations stm32h7_pinctrl_ops = {
    .set_state = stm32h7_pinctrl_set
};

static int ofw_pinctrl_bus_populate_device(struct drvmgr_bus *bus) {
    struct dev_private *devp =device_get_private(bus->dev);
    return __ofw_bus_populate_device(bus, devp->np, 
        ofw_pinctrl_bus_filter);
}

static struct drvmgr_bus_ops stm32h7_pinctrl_bus = {
	.init = {
		ofw_pinctrl_bus_populate_device,
	},
    .unite = stm32h7_pinctrl_bus_unite
};

static int stm32h7_pinctrl_probe(struct drvmgr_dev *dev) {
extern int stm32_gpio_to_portno(struct drvmgr_dev *dev);
    struct dev_private *devp = device_get_private(dev);
    int ret;

    devp->devops = &stm32h7_pinctrl_ops;
    ret = ofw_platform_bus_device_register(dev, &stm32h7_pinctrl_bus, 
    DRVMGR_BUS_TYPE_PINCTRL);
    if (!ret) {
        struct drvmgr_bus *bus = dev->bus;
        struct drvmgr_dev *child;
        struct stm32h7_pinctrl *priv;

        _Assert(bus->dev_cnt > 0);
        priv = rtems_calloc(1, 
            sizeof(struct stm32h7_pinctrl) + bus->dev_cnt * sizeof(struct drvmgr_dev *));
        if (!priv)
            return -ENOMEM;
        priv->nr = bus->dev_cnt;
        child = bus->children;
        while (child) {
            int port = stm32_gpio_to_portno(child);
            if (port < 0 || port > 10) {
                printk("Invalid GPIO device port (%s: %d)\n", child->name, port);
                return -EINVAL;
            }
            priv->gpios[port] = child;
            child = child->next_in_bus;
        }
        dev->priv = priv;
        return 0;
    }
    return ret;
}

static const struct dev_id id_table[] = {
    {.compatible = "st,stm32h743-pinctrl", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops stm32h7_pinctrl_driver = {
	.init = {
		stm32h7_pinctrl_probe,
	},
};
		
OFW_PLATFORM_DRIVER(stm32h7_pinctrl) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "pinctrl",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &stm32h7_pinctrl_driver
	},
	.ids = id_table
};
