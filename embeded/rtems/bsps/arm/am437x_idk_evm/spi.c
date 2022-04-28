#include <dev/dev/spi.h>

#include "bsp/platform_bus.h"
#include "bsp/io.h"

struct spi_private {
    spi_bus bus;
    struct drvmgr_dev *dev;
    unsigned int base;

};

#define AM437X_SPI_TIMEOUT 1500

#define AM335X_SPI_REVISION         0x000
#define AM335X_SPI_SYSCONFIG        0x110
#define AM335X_SPI_SYSSTATUS        0x114
#define AM335X_SPI_IRQSTATUS        0x118
#define AM335X_SPI_IRQENABLE        0x11c
#define AM335X_SPI_WAKEUPENABLE     0x120
#define AM335X_SPI_SYST             0x124
#define AM335X_SPI_MODULCTRL        0x128
#define AM335X_SPI_CH0CONF          0x12c
#define AM335X_SPI_CH0STAT          0x130
#define AM335X_SPI_CH0CTRL          0x134
#define AM335X_SPI_TX0              0x138
#define AM335X_SPI_RX0              0x13C
#define AM335X_SPI_XFERLEVEL        0x17c

/* SPI sysconfig Register */
#define AM335X_SPI_SYSCONFIG_SOFTRESET (1 << 1)

/* SPI sysstatus Register */
#define AM335X_SPI_SYSSTATUS_RESETDONE (1 << 0)

/* SPI interrupt status Register */
#define AM335X_SPI_IRQSTATUS_TX0_EMPTY (1 << 0)
#define AM335X_SPI_IRQSTATUS_RX0_FULL  (1 << 2)

/* SPI interrupt enable Register */
#define AM335X_SPI_IRQENABLE_TX0_EMPTY (1 << 0)
#define AM335X_SPI_IRQENABLE_RX0_FULL  (1 << 2)

/* SPI system Register */
#define AM335X_SPI_SYST_SPIEN_0     (1 << 0)
#define AM335X_SPI_SYST_SPIDAT_0    (1 << 4)
#define AM335X_SPI_SYST_SPIDAT_1    (1 << 5)
#define AM335X_SPI_SYST_SPIDATDIR0  (1 << 8)
#define AM335X_SPI_SYST_SPIDATDIR1  (1 << 9)
#define AM335X_SPI_SYST_SSB         (1 << 11)

/* SPI modulctrl Register */
#define AM335X_SPI_MODULCTRL_SINGLE (1 << 0)
#define AM335X_SPI_MODULCTRL_PIN34  (1 << 1)
#define AM335X_SPI_MODULCTRL_MS     (1 << 2)

/* SPI Channel 0 Configuration Register */
#define AM335X_SPI_CH0CONF_PHA      (1 << 0)
#define AM335X_SPI_CH0CONF_POL      (1 << 1)
#define AM335X_SPI_CH0CONF_CLKD_SHIFT 2
#define AM335X_SPI_CH0CONF_CLKD_WIDTH 4
#define AM335X_SPI_CH0CONF_CLKD_MASK AM335X_MASK(AM335X_SPI_CH0CONF_CLKD_SHIFT, AM335X_SPI_CH0CONF_CLKD_WIDTH)
#define AM335X_SPI_CH0CONF_CLKD(X) (((X) << AM335X_SPI_CH0CONF_CLKD_SHIFT) & AM335X_SPI_CH0CONF_CLKD_MASK)
#define AM335X_SPI_CH0CONF_EPOL     (1 << 6)
#define AM335X_SPI_CH0CONF_WL_SHIFT 7
#define AM335X_SPI_CH0CONF_WL_WIDTH 5
#define AM335X_SPI_CH0CONF_WL_MASK AM335X_MASK(AM335X_SPI_CH0CONF_WL_SHIFT, AM335X_SPI_CH0CONF_WL_WIDTH)
#define AM335X_SPI_CH0CONF_WL(X) (((X) << AM335X_SPI_CH0CONF_WL_SHIFT) & AM335X_SPI_CH0CONF_WL_MASK)
#define AM335X_SPI_CH0CONF_TRM_SHIFT 12
#define AM335X_SPI_CH0CONF_TRM_WIDTH 2
#define AM335X_SPI_CH0CONF_TRM_MASK AM335X_MASK(AM335X_SPI_CH0CONF_TRM_SHIFT, AM335X_SPI_CH0CONF_TRM_WIDTH)
#define AM335X_SPI_CH0CONF_TRM(X) (((X) << AM335X_SPI_CH0CONF_TRM_SHIFT) & AM335X_SPI_CH0CONF_TRM_MASK)
#define AM335X_SPI_CH0CONF_DPE0     (1 << 16)
#define AM335X_SPI_CH0CONF_DPE1     (1 << 17)
#define AM335X_SPI_CH0CONF_IS       (1 << 18)
#define AM335X_SPI_CH0CONF_FORCE    (1 << 20)
#define AM335X_SPI_CH0CONF_SBPOL    (1 << 27)
#define AM335X_SPI_CH0CONF_FFEW     (1 << 27)
#define AM335X_SPI_CH0CONF_FFER     (1 << 28)

/* SPI Channel 0 Status Register */
#define AM335X_SPI_CH0STAT_RXS      (1 << 0)
#define AM335X_SPI_CH0STAT_TXS      (1 << 1)

/* SPI Channel 0 Control Register */
#define AM335X_SPI_CH0CTRL_EN       (1 << 0)

/* SPI Transfer Level Register */
#define AM335X_SPI_XFERLEVEL_WCNT(n) (((n) << 16) & 0xFFFF)
#define AM335X_SPI_XFERLEVEL_AFL(n)  (((n) << 8) & 0xFF)
#define AM335X_SPI_XFERLEVEL_AEL(n)  (((n) << 0) & 0xFF)


static int am437x_spi_reset(struct spi_private *priv) {
    int timeout = AM437X_SPI_TIMEOUT;
    uint32_t v = readl(priv->base + AM335X_SPI_SYSCONFIG);
    v |= AM335X_SPI_SYSCONFIG_SOFTRESET;
    writel(v, priv->base + AM335X_SPI_SYSCONFIG);
    do {
        if (timeout <= 0)
            return -EIO;
        timeout--;
        v = readl(priv->base + AM335X_SPI_SYSSTATUS));
    } while (!(v & AM335X_SPI_SYSSTATUS_RESETDONE) && timeout >= 0);
    return 0;
}

static int am437x_spi_init(struct spi_private *priv) {
    int ret = am437x_spi_reset(priv);
    if (ret) {
        printk("%s reset failed\n", priv->dev->name);
        return ret;
    }
    /*
     * Master mode
     * Single channel
     * SPIEN is usedas a chip select
     */
    unsigned int rv = readl(priv->base + AM335X_SPI_MODULCTRL)；
    rv &=  ~(AM335X_SPI_MODULCTRL_PIN34 | AM335X_SPI_MODULCTRL_MS);
    rv |= AM335X_SPI_MODULCTRL_SINGLE;
    writel(rv, priv->base + AM335X_SPI_MODULCTRL);
    return 0;
}

static void am437x_spi_interrupt(void *arg) {
    struct spi_private *priv = arg;
    uint32_t irqs;

    irqs = readl_relaxed(priv->base + AM335X_SPI_IRQSTATUS);
    writel_relaxed(irqs, priv->base + AM335X_SPI_IRQSTATUS);
    if (irqs & AM335X_SPI_IRQSTATUS_RX0_FULL) {

    }
    if (irqs & AM335X_SPI_IRQSTATUS_TX0_EMPTY) {

    }
}

static int am437x_spi_transfer(spi_bus *bus, const spi_ioc_transfer *msgs, 
    uint32_t msg_count) {
    for (size_t i = 0; i < msg_count; i++) {

    }
    return -EIO;
}

static int am437x_spi_setup(spi_bus *bus) {
    unsigned int div = bus->max_speed_hz / bus->speed_hz;
    if (div && (div & (div - 1)) == 0) {
        unsigned int rv = readl(priv->base + AM335X_SPI_CH0CONF)；
        rv &= 0x00060000; //Reset mask
        rv = (rv  & ~AM335X_SPI_CH0CONF_IS) | 
            AM335X_SPI_CH0CONF_DPE0 |
            AM335X_SPI_CH0CONF_FFEW |
            AM335X_SPI_CH0CONF_FFER |
            AM335X_SPI_CH0CONF_WL(bus->bits_per_word - 1) |
            AM335X_SPI_CH0CONF_EPOL |
            bus->mode;
        div = 31 - __builtin_clz(div);
        rv |= AM335X_SPI_CH0CONF_CLKD(div);
        writel(rv, priv->base + AM335X_SPI_CH0CONF);
        return 0;
    }
    return -EINVAL;
}

static void am437x_spi_destroy(spi_bus *bus) {
	struct spi_private *spi = (struct i2c_private *)bus;

	drvmgr_interrupt_unregister(spi->dev, 0, am437x_spi_interrupt, spi);
	spi_bus_destroy_and_free(bus);
}


static int spi_bus_unite(struct drvmgr_drv *drv, 
	struct drvmgr_dev *dev) {
	return platform_bus_match(drv, dev, DRVMGR_BUS_TYPE_I2C);
}

static struct drvmgr_bus_ops spi_bus_ops = {
	.init  = { platform_bus_populate, },
	.unite = spi_bus_unite,
};

static int am437x_spi_probe(struct drvmgr_dev *dev) {
	union drvmgr_key_value *prop;
	struct spi_private *spi;
    struct dev_private *devp;
	int ret;
    spi = rtems_calloc(1, sizeof(struct spi_private));
    if (spi == NULL) 
        return -DRVMGR_NOMEM;
    devp = device_get_private(dev);
	prop = devcie_get_property(dev, "CLKCTRL");
	if (!prop) 
		goto _free;
	spi->clkctrl = prop->i;
    spi->base = devp->base;
	spi->dev = dev;
	dev->priv = spi;

	/* Enable module and reset */
	writeb(0x2, i2c->clkctrl);
	ret = am437x_spi_init(i2c);
	if (ret) 
		goto _free;
    ret = drvmgr_interrupt_register(dev, 0, dev->name, 
		am437x_spi_interrupt, spi);
    if (ret) {
        devdbg("Register irq for %s failed\n", dev->name);
        goto _free;
    }
	ret = platform_bus_device_register(dev, &spi_bus_ops, 
		DRVMGR_BUS_TYPE_SPI);
	if (ret) {
		devdbg("Register spi bus device（%s） failed\n", dev->name);
		goto _remove_irq;
	}
	spi_bus_init(&spi->bus);
	spi->bus.transfer = am437x_spi_transfer;
    spi->bus.setup = am437x_spi_setup;
    spi->bus.destroy = am437x_spi_destroy;
	ret = spi_bus_register(&spi->bus, dev->name);
	if (ret)
		goto _destory;
	return 0;
_destory:
	spi_bus_destroy(&spi->bus);
_remove_irq:
	drvmgr_interrupt_unregister(dev, 0, am437x_spi_interrupt, spi);
_free:
	free(spi);
	return ret;
}

static const struct dev_id id_table[] = {
    {.compatible = "ti,am4372-i2c", NULL},
	{.compatible = "ti,omap4-i2c", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops spi_driver_ops = {
	.init = { am437x_spi_probe, }
};
		
PLATFORM_DRIVER(spi_bus) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "spi",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &spi_driver_ops
	},
	.ids = id_table
};

