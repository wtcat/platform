#include <rtems/bspIo.h>

#include "drivers/platform_bus.h"
#include "drivers/mio.h"

#include "bsp/board/pinctrl.h"

#ifdef DEBUG_ON
#define pr_debug(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define pr_debug(...)
#endif

static int pinctrl_init(struct drvmgr_dev *dev) {
    union drvmgr_key_value *prop;
    prop = devcie_get_property(dev, "pins");
    if (!prop) {
        printk("Error***(%s): Not found pins property\n", __func__);
        return -DRVMGR_EINVAL;
    }
    struct dev_private *devp = device_get_private(dev);
    struct pinctrl *pins = (struct pinctrl *)prop->ptr;
    while (pins->offset != 0xFFFFFFFF) {
        pr_debug("PinMux Write: 0x%x => 0x%x + 0x%x\n", pins->value,
            devp->base, pins->offset);
        writel(pins->value, devp->base + pins->offset);
        pins++;
    }
    return 0;
}

static const struct dev_id id_table[] = {
    {.compatible = "pinctrl-single,pins", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops pinctrl_driver_ops = {
	.init = { pinctrl_init, }
};
		
PLATFORM_DRIVER(pinctrl) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "pinctrl",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &pinctrl_driver_ops
	},
	.ids = id_table
};
