#include <rtems/bspIo.h>
#include <rtems/malloc.h>

#include "bsp/timlib.h"
#include "bsp/io.h"

struct dmtimer_priv {
    struct timlib_priv base;
    unsigned int base;
    unsigned int clkbase;
};

#define DMTIMER_TIOCP_CFG   0x10
#define DMTIMER_IRQ_EOI     0x20
#define DMTIMER_IRQSTS_RAW  0x24
#define DMTIMER_IRQSTS      0x28
#define DMTIMER_IRQEN_SET   0x2C
#define DMTIMER_IRQEN_CLR   0x30
#define DMTIMER_IRQWAKEEN   0x34
#define DMTIMER_TCLR        0x38
#define DMTIMER_TCRR        0x3C
#define DMTIMER_TLDR        0x40
#define DMTIMER_TTGR        0x44
#define DMTIMER_TWPS        0x48
#define DMTIMER_TMAR        0x4C
#define DMTIMER_TCAR1       0x50
#define DMTIMER_TSICR       0x54
#define DMTIMER_TCAR2       0x58

#define TCLR_ST         BIT(0)
#define TCLR_AR         BIT(1)
#define TCLR_PTV(n)     (((n) & 0x7) << 2)
#define TCLR_PRE        BIT(5)
#define TCLR_CE         BIT(6)
#define TCLR_SCPWM      BIT(7)
#define TCLR_TCM(n)     (((n) & 0x3) << 8)
#define TCLR_TRG(n)     (((n) & 0x3) << 10)
#define TCLR_PT         BIT(12)
#define TCLR_CAPT_MODE  BIT(13)
#define TCLR_GPO_CFG    BIT(14)


static void timer_reset(struct dmtimer_priv *priv) {
    uint32_t tmp;
    /* Enable function clock */
    writel(0x2, priv->clkbase);
    /* Reset timer */
    writel(0x1, priv->base + DMTIMER_TIOCP_CFG);
    do {
        tmp = readl_relaxed(priv->base + DMTIMER_TIOCP_CFG)
    } while(tmp & 0x1);
}

static void timer_start(struct dmtimer_priv *priv) {
    uint32_t rv = readl_relaxed(priv->base + DMTIMER_TCLR);
    rv |= TCLR_ST;
    /* Reload counter from load register */
    writel_relaxed(1, priv->base + DMTIMER_TTGR); 
    writel_relaxed(rv, priv->base + DMTIMER_TCLR);
}

static void timer_stop(struct dmtimer_priv *priv) {
    uint32_t rv = readl_relaxed(priv->base + DMTIMER_TCLR);
    rv &= ~TCLR_ST;
    writel_relaxed(rv, priv->base + DMTIMER_TCLR);    
}

static int timer_init(struct drvmgr_dev *dev) {
    union drvmgr_key_value *prop;
    struct dev_private *devp;
    struct dmtimer_priv *priv;
	int ret;
    priv = rtems_calloc(1, sizeof(struct dmtimer_priv));
    if (priv == NULL) {
        printk("Error***(%s): no more memory\n", __func__);
        return -DRVMGR_NOMEM;
    }
    prop = devcie_get_property(dev, "fck");
    if (!prop) {
        printk("Error***(%s): not found fck\n", __func__);
        return -DRVMGR_ENORES;
    }
    devp = device_get_private(dev);
    priv->base = devp->base;
    priv->clkbase = prop->i;
    dev->priv = priv;
    return 0;
}

static const struct dev_id id_table[] = {
    {.compatible = "ti,am4372-timer", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops timer_driver_ops = {
	.init = {
		timer_init,
	},
};
		
PLATFORM_DRIVER(gpio_keys) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "timer",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &timer_driver_ops
	},
	.ids = id_table
};