#include <stdlib.h>

#include <rtems/malloc.h>
#include <rtems/bspIo.h>
#include <dev/spi/spi.h>

#include "bsp/platform_bus.h"
#include "bsp/io.h"

struct spi_private {
    spi_bus bus;
    rtems_id thread;
    struct drvmgr_dev *dev;
    const spi_ioc_transfer *msg;
    size_t msg_todo;
    size_t todo;
    int in_transfer;
    uint8_t *rx_buf;
    const uint8_t *tx_buf;
    uint32_t base; /* Register base address */
    uint32_t clkctrl; /* Clock address */
    uint32_t conf;
    void (*push)(struct spi_private *);
    void (*pop)(struct spi_private *);
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


#define AM335X_MASK(Shift, Width) (((1 << (Width)) - 1) << (Shift))

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
#define AM335X_SPI_CH0CONF_CLKD_MASK AM335X_MASK(2, 4)
#define AM335X_SPI_CH0CONF_CLKD(X) (((X) << 2) & AM335X_SPI_CH0CONF_CLKD_MASK)
#define AM335X_SPI_CH0CONF_EPOL     (1 << 6)
#define AM335X_SPI_CH0CONF_WL_MASK AM335X_MASK(7, 5)
#define AM335X_SPI_CH0CONF_WL(X) (((X) << 7) & AM335X_SPI_CH0CONF_WL_MASK)
#define AM335X_SPI_CH0CONF_TRM_MASK AM335X_MASK(12, 2)
#define AM335X_SPI_CH0CONF_TRM(X) (((X) << 12) & AM335X_SPI_CH0CONF_TRM_MASK)
#define AM335X_SPI_CH0CONF_DPE0     (1 << 16)
#define AM335X_SPI_CH0CONF_DPE1     (1 << 17)
#define AM335X_SPI_CH0CONF_IS       (1 << 18)
#define AM335X_SPI_CH0CONF_FORCE    (1 << 20)
#define AM335X_SPI_CH0CONF_TCS(n)   ((n << 25) & 0x3)
#define AM335X_SPI_CH0CONF_SBPOL    (1 << 27)
#define AM335X_SPI_CH0CONF_FFEW     (1 << 27)
#define AM335X_SPI_CH0CONF_FFER     (1 << 28)

/* SPI Channel 0 Status Register */
#define AM335X_SPI_CH0STAT_RXS      (1 << 0)
#define AM335X_SPI_CH0STAT_TXS      (1 << 1)
#define AM335X_SPI_CH0STAT_RXFFE    (1 << 5)

/* SPI Channel 0 Control Register */
#define AM335X_SPI_CH0CTRL_EN       (1 << 0)

/* SPI Transfer Level Register */
#define AM335X_SPI_XFERLEVEL_WCNT(n) (((n) << 16) & 0xFFFF)
#define AM335X_SPI_XFERLEVEL_AFL(n)  (((n) << 8) & 0xFF)
#define AM335X_SPI_XFERLEVEL_AEL(n)  (((n) << 0) & 0xFF)

#define AM437X_SPI_FIFO_SIZE 32

#ifdef DEBUG_ON
#define devdbg(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define devdbg(...)
#endif

static inline void am437x_spi_modreg(uint32_t reg, uint32_t mask, 
    uint32_t value) {
    uint32_t v = readl_relaxed(reg);
    v = (v & ~mask) | value;
    writel_relaxed(v, reg);
}

static int am437x_spi_rxfifo_empty(struct spi_private *priv) {
    return readl_relaxed(priv->base + AM335X_SPI_CH0STAT) & 
        AM335X_SPI_CH0STAT_RXFFE;
}

#define AM437X_SPI_PUSH_TEMPLATE(type) \
    static void am437x_spi_push_##type(struct spi_private *priv) { \
        while (priv->todo > 0 && priv->in_transfer < AM437X_SPI_FIFO_SIZE) { \
            type val = 0; \
            if (priv->tx_buf) { \
                val = *(type *)priv->tx_buf; \
                priv->tx_buf += sizeof(type); \
            } \
            priv->todo -= sizeof(type); \
            priv->in_transfer++; \
            writel_relaxed((uint32_t)val, priv->base + AM335X_SPI_TX0); \
        } \
    }

#define AM437X_SPI_POP_TEMPLATE(type) \
    static void am437x_spi_pop_##type(struct spi_private *priv) { \
        while (!am437x_spi_rxfifo_empty(priv)) { \
            uint32_t val = readl_relaxed(priv->base + AM335X_SPI_RX0); \
            if (priv->rx_buf) { \
                *(type *)priv->rx_buf = val; \
                priv->rx_buf += sizeof(type); \
            } \
            priv->in_transfer--; \
        } \
    }

AM437X_SPI_PUSH_TEMPLATE(uint8_t)
AM437X_SPI_POP_TEMPLATE(uint8_t)
AM437X_SPI_PUSH_TEMPLATE(uint16_t)
AM437X_SPI_POP_TEMPLATE(uint16_t)
AM437X_SPI_PUSH_TEMPLATE(uint32_t)
AM437X_SPI_POP_TEMPLATE(uint32_t)

static inline void am437x_spi_push(struct spi_private *priv) {
    priv->push(priv);
}

static inline void am437x_spi_pop(struct spi_private *priv) {
    priv->pop(priv);
}

static void am437x_spi_set_method(struct spi_private *priv, 
    uint8_t bits_per_word) {
    if (bits_per_word <= 8) {
        priv->push = am437x_spi_push_uint8_t;
        priv->pop = am437x_spi_pop_uint8_t;
    } else if (bits_per_word <= 16) {
        priv->push = am437x_spi_push_uint16_t;
        priv->pop = am437x_spi_pop_uint16_t;
    } else {
        priv->push = am437x_spi_push_uint32_t;
        priv->pop = am437x_spi_pop_uint32_t;
    }
}

static int am437x_spi_reset(struct spi_private *priv) {
    int timeout = AM437X_SPI_TIMEOUT;
    uint32_t v = readl(priv->base + AM335X_SPI_SYSCONFIG);
    v |= AM335X_SPI_SYSCONFIG_SOFTRESET;
    writel(v, priv->base + AM335X_SPI_SYSCONFIG);
    do {
        if (timeout <= 0)
            return -EIO;
        timeout--;
        v = readl(priv->base + AM335X_SPI_SYSSTATUS);
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
    am437x_spi_modreg(priv->base + AM335X_SPI_MODULCTRL, 
        AM335X_SPI_MODULCTRL_PIN34 | AM335X_SPI_MODULCTRL_MS,
        AM335X_SPI_MODULCTRL_SINGLE);
    return 0;
}

static void am437x_spi_done(struct spi_private *priv) {
    writel_relaxed(0, priv->base + AM335X_SPI_IRQENABLE);
    rtems_event_transient_send(priv->thread);
}

static int am437x_spi_configure(struct spi_private *priv,
    uint32_t speed_hz, uint32_t mode, uint8_t cs) {
    (void) mode;
    (void) cs;
    uint32_t div = priv->bus.max_speed_hz / speed_hz;
    if (div && (div & (div - 1)) == 0) {
        uint32_t rv = readl_relaxed(priv->base + AM335X_SPI_CH0CONF);
        rv &= 0x00060000; //Reset mask
        rv = (rv  & ~AM335X_SPI_CH0CONF_IS) | 
            AM335X_SPI_CH0CONF_DPE0 |
            AM335X_SPI_CH0CONF_FFEW |
            AM335X_SPI_CH0CONF_FFER |
            AM335X_SPI_CH0CONF_TCS(1) |
            AM335X_SPI_CH0CONF_WL(priv->bus.bits_per_word - 1) |
            AM335X_SPI_CH0CONF_EPOL |
            priv->bus.mode;
        div = 31 - __builtin_clz(div);
        rv |= AM335X_SPI_CH0CONF_CLKD(div);
        priv->conf = rv;
        writel(rv, priv->base + AM335X_SPI_CH0CONF);
        return 0;
    }
    return -EINVAL;
}

static void am437x_spi_next_message(struct spi_private *priv) {
    if (priv->msg_todo > 0) {
        const spi_ioc_transfer *msg = priv->msg;
        spi_bus *bus = &priv->bus;
        if (msg->speed_hz != bus->speed_hz || 
            msg->mode != bus->mode || 
            msg->cs != bus->cs) {
            am437x_spi_configure(priv, msg->speed_hz, msg->mode, msg->cs);
        }
        priv->todo = msg->len;
        priv->tx_buf = msg->tx_buf;
        priv->rx_buf = msg->rx_buf;
        am437x_spi_set_method(priv, msg->bits_per_word);
        am437x_spi_push(priv);
        writel_relaxed(AM335X_SPI_IRQENABLE_TX0_EMPTY, 
            priv->base + AM335X_SPI_IRQENABLE);
        
        writel_relaxed(priv->conf | AM335X_SPI_CH0CONF_FORCE,
            priv->base + AM335X_SPI_CH0CONF);
        writeb_relaxed(1, priv->base + AM335X_SPI_CH0CTRL);
    } else {
        am437x_spi_done(priv);
    }
}

static void am437x_spi_interrupt(void *arg) {
    struct spi_private *priv = arg;
    uint32_t irqs;
    irqs = readl_relaxed(priv->base + AM335X_SPI_IRQSTATUS);
    writel_relaxed(irqs, priv->base + AM335X_SPI_IRQSTATUS);
    am437x_spi_pop(priv);
    if (priv->todo > 0) {
        am437x_spi_push(priv);
    } else if (priv->in_transfer > 0) {
        writel_relaxed(AM335X_SPI_IRQENABLE_RX0_FULL, 
            priv->base + AM335X_SPI_IRQENABLE);
    } else {
        priv->msg_todo--;
        priv->msg++;
        am437x_spi_next_message(priv);
    }
}

static int am437x_spi_transfer(spi_bus *bus, const spi_ioc_transfer *msgs, 
    uint32_t n) {
    struct spi_private *priv = (struct spi_private *)bus;
    priv->msg_todo = n;
    priv->msg = &msgs[0];
    priv->thread = rtems_task_self();
    am437x_spi_next_message(priv);
    rtems_event_transient_receive(RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    writeb_relaxed(0, priv->base + AM335X_SPI_CH0CTRL);
    writel_relaxed(priv->conf, priv->base + AM335X_SPI_CH0CONF);
    return 0;
}

static int am437x_spi_setup(spi_bus *bus) {
    if (bus->max_speed_hz < bus->speed_hz)
        return -EINVAL;
    if (bus->bits_per_word < 4 || bus->bits_per_word > 32)
        return -EINVAL;
    return am437x_spi_configure((struct spi_private *)bus, 
        bus->max_speed_hz, bus->mode, bus->cs);
}

static void am437x_spi_destroy(spi_bus *bus) {
	struct spi_private *spi = (struct spi_private *)bus;
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

static void parse_parameters(struct drvmgr_dev *dev, spi_bus *bus) {
    union drvmgr_key_value *prop;
    prop = devcie_get_property(dev, "mode");
    if (prop)
        bus->mode = prop->i;
    else
        bus->mode = 0;
    prop = devcie_get_property(dev, "speed");
    if (prop)
        bus->speed_hz = prop->i;
    else
        bus->speed_hz = 48000000;
    prop = devcie_get_property(dev, "word-length");
    if (prop)
        bus->bits_per_word = prop->i;
    else
        bus->bits_per_word = 8;
}

static int am437x_spi_probe(struct drvmgr_dev *dev) {
	union drvmgr_key_value *prop;
	struct spi_private *spi;
    struct dev_private *devp;
	int ret = -EINVAL;
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
	writeb(0x2, spi->clkctrl);
	ret = am437x_spi_init(spi);
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
    parse_parameters(dev, &spi->bus);
	ret = spi_bus_register(&spi->bus, platform_dev_filename(dev));
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
    {.compatible = "ti,am4372-mcspi", NULL},
	{.compatible = "ti,omap4-mcspi", NULL},
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

