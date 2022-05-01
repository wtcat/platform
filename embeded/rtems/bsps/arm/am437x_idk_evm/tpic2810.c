#include <stdlib.h>
#include <rtems/thread.h>
#include <rtems/malloc.h>
#include <rtems/bspIo.h>

#include "bsp/platform_bus.h"
#include "bsp/gpiod.h"
#include "bsp/i2cd.h"

#define TPIC2810_WS_COMMAND 0x44

struct tpic2810_priv {
    const struct gpio_operations *ops;
    struct drvmgr_dev *i2c;
    rtems_mutex lock;
    uint16_t addr;
    uint8_t ov;
};

static int tpic2810_output(struct drvmgr_dev *dev, uint32_t bitmask, 
    uint32_t val) {
    struct tpic2810_priv *priv = dev->priv;
    uint8_t buffer[2];
    rtems_mutex_lock(&priv->lock);
    priv->ov = (priv->ov & ~bitmask) | (uint8_t)val;
    buffer[0] = TPIC2810_WS_COMMAND;
    buffer[1] = priv->ov;
    int err =  i2c_master_write(priv->i2c, buffer, sizeof(buffer), 
        priv->addr);
    rtems_mutex_unlock(&priv->lock);
    return err;
}

static int tpic2810_set(struct drvmgr_dev *dev, int pin, int val) {
    if (pin > 7)
        return -EINVAL;
    return tpic2810_output(dev, 1u << pin, val);
}

static const struct gpio_operations tpic2810_gpio_ops = {
    .set_pin = tpic2810_set,
	.set_port = tpic2810_output,
};

static int tpic2810_probe(struct drvmgr_dev *dev) {
    union drvmgr_key_value *prop;
    struct dev_private *devp;
    struct tpic2810_priv *priv;
	int ret;
    priv = rtems_calloc(1, sizeof(struct tpic2810_priv));
    if (priv == NULL) {
        printk("Error***(%s): no more memory\n", __func__);
        return -DRVMGR_NOMEM;
    }
    rtems_mutex_init(&priv->lock, "tpic2810");
    devp = device_get_private(dev);
    dev->priv = priv;
    priv->addr = (uint16_t)devp->base;
    priv->i2c = dev->parent->dev;
    priv->ops = &tpic2810_gpio_ops;
    priv->ov = 0;
	return 0;
_free:
    free(priv);
    return 0;
}

static const struct dev_id id_table[] = {
    {.compatible = "ti,tpic2810", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops tpic2810_driver_ops = {
	.init = { tpic2810_probe, },
};
		
PLATFORM_DRIVER(tpic2810) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "tpic2810",
		.bus_type = DRVMGR_BUS_TYPE_I2C,
		.ops      = &tpic2810_driver_ops
	},
	.ids = id_table
};