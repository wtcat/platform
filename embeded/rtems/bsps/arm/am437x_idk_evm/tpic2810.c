#include "bsp/i2cd.h"

#define TPIC2810_WS_COMMAND 0x44

struct tpic2810_priv {
    struct drvmgr_dev *master;
    uint16_t addr;
    uint8_t ov;
};

static void tpic2810_output(struct drvmgr_dev *dev, uint8_t bitmask, 
    uint8_t val) {
    struct tpic2810_priv *priv = dev->priv;
    uint8_t buffer[3];
    priv->ov = (priv->ov & ~bitmask) | val;
    buffer[0] = 0xC0;
    buffer[1] = TPIC2810_WS_COMMAND;
    buffer[2] = priv->ov;
    i2c_master_write(dev->master, buffer, sizeof(buffer), priv->addr);
}

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
    devp = device_get_private(dev);
    prop = devcie_get_property(dev, "address");
    if (!prop) {
        printk("Error***(%s): Not found address property\n", __func__);
        goto _free;
    }
    priv->addr = (uint16_t)prop->i;
    priv->master = dev->parent->dev;
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
		
PLATFORM_DRIVER(tpic2810)) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "gpio_keys",
		.bus_type = DRVMGR_BUS_TYPE_I2C,
		.ops      = &tpic2810_driver_ops
	},
	.ids = id_table
};