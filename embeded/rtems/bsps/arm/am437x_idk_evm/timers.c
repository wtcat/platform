#include <rtems/bspIo.h>
#include <rtems/malloc.h>

#include "bsp/platform_bus.h"
#include "bsp/timlib.h"
#include "bsp/io.h"

struct dmtimer_priv {
    struct timlib_priv timer;
    unsigned int base;
    unsigned int clkbase;
    int prescaler;
};


#ifndef BIT
#define BIT(n) (1ul << n)
#endif

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

#define TIMER_MAX_COUNT    0xFFFFFFFF
#define TIMER_CLKSRC_FREQ  24000000


static void timer_dump_registers(const char *name, 
    unsigned int base) {
    printk("Timer(%s) registers:\n", name);
    printk("    DMTIMER_TIOCP_CFG = 0x%x\n", 
        readl_relaxed(base + DMTIMER_TIOCP_CFG));
    printk("    DMTIMER_IRQSTS_RAW = 0x%x\n", 
        readl_relaxed(base + DMTIMER_IRQSTS_RAW));
    printk("    DMTIMER_IRQEN_SET = 0x%x\n", 
        readl_relaxed(base + DMTIMER_IRQEN_SET));
    printk("    DMTIMER_TCLR = 0x%x\n", 
        readl_relaxed(base + DMTIMER_TCLR));    
    printk("    DMTIMER_TCRR = 0x%x\n", 
        readl_relaxed(base + DMTIMER_TCRR));
    printk("    DMTIMER_TLDR = 0x%x\n", 
        readl_relaxed(base + DMTIMER_TLDR));
    printk("    DMTIMER_TTGR = 0x%x\n", 
        readl_relaxed(base + DMTIMER_TTGR));
    printk("    DMTIMER_TWPS = 0x%x\n", 
        readl_relaxed(base + DMTIMER_TWPS));  
    printk("    DMTIMER_TMAR = 0x%x\n", 
        readl_relaxed(base + DMTIMER_TMAR));  
    printk("    DMTIMER_TCAR1 = 0x%x\n", 
        readl_relaxed(base + DMTIMER_TCAR1));  
    printk("    DMTIMER_TSICR = 0x%x\n", 
        readl_relaxed(base + DMTIMER_TSICR));  
    printk("    DMTIMER_TCAR2 = 0x%x\n", 
        readl_relaxed(base + DMTIMER_TCAR2));  
}

static void timer_set_prescaler(struct dmtimer_priv *priv, 
    int prescale) {
    if (prescale) {
        uint32_t rv = readl_relaxed(priv->base + DMTIMER_TCLR);
        rv &= ~TCLR_PTV(7);
        rv |= TCLR_PRE | prescale;
        writel_relaxed(rv, priv->base + DMTIMER_TCLR);
    }
}
static void timer_reset(struct drvmgr_dev *dev) {
    struct dmtimer_priv *priv = dev->priv;
    printk("%s: Resetting timer ...\n", dev->name);
    /* Reset timer */
    writel(0x1, priv->base + DMTIMER_TIOCP_CFG);
    while(readl_relaxed(priv->base + DMTIMER_TIOCP_CFG) & 0x1);
    timer_set_prescaler(priv, priv->prescaler);
    printk("%s: Reset done ...\n", dev->name);
}

static void timer_start(struct drvmgr_dev *dev) {
    struct dmtimer_priv *priv = dev->priv;
    uint32_t rv = readl_relaxed(priv->base + DMTIMER_TCLR);
    rv |= TCLR_ST;
    /* Reload counter from load register */
    writel_relaxed(1, priv->base + DMTIMER_TTGR); 
    writel_relaxed(rv, priv->base + DMTIMER_TCLR);
}

static void timer_stop(struct drvmgr_dev *dev) {
    struct dmtimer_priv *priv = dev->priv;
    uint32_t rv = readl_relaxed(priv->base + DMTIMER_TCLR);
    rv &= ~TCLR_ST;
    writel_relaxed(rv, priv->base + DMTIMER_TCLR);    
}

static void timer_restart(struct drvmgr_dev *dev) {
    struct dmtimer_priv *priv = dev->priv;
    writel_relaxed(1, priv->base + DMTIMER_TTGR);  
}

static int timer_set_freq(struct drvmgr_dev *dev, uint32_t tickrate) {
    struct dmtimer_priv *priv = dev->priv;
    uint32_t ldv = TIMER_MAX_COUNT - tickrate;
    writel_relaxed(ldv, priv->base + DMTIMER_TLDR);   
}

static int timer_get_freq(struct drvmgr_dev *dev, uint32_t *basefreq,
    uint32_t *tickrate) {
    struct dmtimer_priv *priv = dev->priv;
    if (basefreq)
        *basefreq = TIMER_CLKSRC_FREQ / (priv->prescaler + 1);
    if (*tickrate) {
        uint32_t v = readl_relaxed(priv->base + DMTIMER_TLDR);
        *tickrate = TIMER_MAX_COUNT - v;
    }
    return 0;
}

static int timer_irq_install(struct drvmgr_dev *dev, 
    timlib_isr_t isr, void *arg) {
    int ret = drvmgr_interrupt_register(dev, 0, dev->name, 
        isr, arg);
    if (!ret) {
        struct dmtimer_priv *priv = dev->priv;
        writel_relaxed(0x2, priv->base + DMTIMER_IRQEN_SET);
    }
    return ret;
}

static int timer_irq_uninstall(struct drvmgr_dev *dev, 
    timlib_isr_t isr, void *arg) {
    struct dmtimer_priv *priv = dev->priv;
    writel_relaxed(0x2, priv->base + DMTIMER_IRQEN_CLR);
    return drvmgr_interrupt_unregister(dev, 0, isr, arg);
}

static uint32_t timer_get_counter(struct drvmgr_dev *dev) {
    struct dmtimer_priv *priv = dev->priv;
    uint32_t cnt = readl_relaxed(priv->base + DMTIMER_TCRR);
    return TIMER_MAX_COUNT - cnt;
}

static uint32_t timer_get_widthmask(struct drvmgr_dev *dev) {
    return TIMER_MAX_COUNT;
}

static void timer_dump_regs(struct drvmgr_dev *dev) {
    struct dmtimer_priv *priv = dev->priv;
    timer_dump_registers(dev->name, priv->base);
}

static const struct timlib_ops hw_timer_ops = {
    .reset = timer_reset,
    .start = timer_start,
    .stop = timer_stop,
    .restart = timer_restart,
    .set_freq = timer_set_freq,
    .get_freq = timer_get_freq,
    .reg_intr = timer_irq_install,
    .unreg_intr = timer_irq_uninstall,
    .get_counter = timer_get_counter,
    .get_widthmask = timer_get_widthmask,
    .dump = timer_dump_regs
};

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
    priv->clkbase = prop->i;
    prop = devcie_get_property(dev, "prescaler");
    priv->prescaler = prop? prop->i: 0;
    devp = device_get_private(dev);
    priv->base = devp->base;
    priv->timer.ops = &hw_timer_ops;
    dev->priv = priv;

    /* Enable function clock */
    writel(0x2, priv->clkbase);
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
		
PLATFORM_DRIVER(timer) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "timer",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &timer_driver_ops
	},
	.ids = id_table
};