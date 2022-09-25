#include <errno.h>
#include <stdlib.h>

#include <rtems/bspIo.h>
#include <rtems/malloc.h>
#include <rtems/irq-extension.h>

#include <bsp.h>

#include "drivers/mio.h"
#include "drivers/gpio.h"
#include "drivers/platform_bus.h"

#define GPIO_PAD_PINS 32
struct gpio_entry {
	void (*handler)(void *arg);
	void *arg;
};

struct gpio_priv {
	struct gpio_entry enties[GPIO_PAD_PINS];
	rtems_interrupt_lock lock;
	struct drvmgr_dev *dev;
	unsigned int base;
};


#ifndef BIT
#define BIT(n) (0x1ul << (n))
#endif

/* AM33XX GPIO registers */
#define OMAP_GPIO_REVISION		0x0000
#define OMAP_GPIO_SYSCONFIG		0x0010
#define OMAP_GPIO_SYSSTATUS		0x0114
#define OMAP_GPIO_IRQSTATUS1		0x002c
#define OMAP_GPIO_IRQSTATUS2		0x0030
#define OMAP_GPIO_IRQSTATUS_SET_0	0x0034
#define OMAP_GPIO_IRQSTATUS_SET_1	0x0038
#define OMAP_GPIO_CTRL			0x0130
#define OMAP_GPIO_OE			0x0134
#define OMAP_GPIO_DATAIN		0x0138
#define OMAP_GPIO_DATAOUT		0x013c
#define OMAP_GPIO_LEVELDETECT0		0x0140
#define OMAP_GPIO_LEVELDETECT1		0x0144
#define OMAP_GPIO_RISINGDETECT		0x0148
#define OMAP_GPIO_FALLINGDETECT		0x014c
#define OMAP_GPIO_DEBOUNCE_EN		0x0150
#define OMAP_GPIO_DEBOUNCE_VAL		0x0154
#define OMAP_GPIO_CLEARDATAOUT		0x0190
#define OMAP_GPIO_SETDATAOUT		0x0194
#define OMAP_GPIO_IRQSTATUS_CLR_0  0x003c
#define OMAP_GPIO_IRQSTATUS_CLR_1  0x0040

#ifdef DEBUG_ON
static void gpio_bus_dump(struct drvmgr_dev *dev) {
	struct gpio_priv *platdata = dev->priv;
	printk("GPIO DUMP(0x%x):\n", platdata->base);
	printk("	OMAP_GPIO_SYSCONFIG = 0x%x\n", 
		readl_relaxed(platdata->base + OMAP_GPIO_SYSCONFIG));
	printk("	OMAP_GPIO_SYSSTATUS = 0x%x\n", 
		readl_relaxed(platdata->base + OMAP_GPIO_SYSSTATUS));
	printk("	OMAP_GPIO_CTRL = 0x%x\n", 
		readl_relaxed(platdata->base + OMAP_GPIO_CTRL));
	printk("	OMAP_GPIO_OE = 0x%x\n", 
		readl_relaxed(platdata->base + OMAP_GPIO_OE));
	printk("	OMAP_GPIO_DATAIN = 0x%x\n", 
		readl_relaxed(platdata->base + OMAP_GPIO_DATAIN));
	printk("	OMAP_GPIO_DATAOUT = 0x%x\n", 
		readl_relaxed(platdata->base + OMAP_GPIO_DATAOUT));
	printk("	OMAP_GPIO_LEVELDETECT0 = 0x%x\n", 
		readl_relaxed(platdata->base + OMAP_GPIO_LEVELDETECT0));
	printk("	OMAP_GPIO_LEVELDETECT1 = 0x%x\n", 
		readl_relaxed(platdata->base + OMAP_GPIO_LEVELDETECT1));
	printk("	OMAP_GPIO_RISINGDETECT = 0x%x\n", 
		readl_relaxed(platdata->base + OMAP_GPIO_RISINGDETECT));
	printk("	OMAP_GPIO_FALLINGDETECT = 0x%x\n", 
		readl_relaxed(platdata->base + OMAP_GPIO_FALLINGDETECT));
	printk("	OMAP_GPIO_IRQSTATUS_SET_0 = 0x%x\n", 
		readl_relaxed(platdata->base + OMAP_GPIO_IRQSTATUS_SET_0));
	printk("	OMAP_GPIO_IRQSTATUS_SET_1 = 0x%x\n", 
		readl_relaxed(platdata->base + OMAP_GPIO_IRQSTATUS_SET_1));
}
#endif

static void gpio_bus_set_dir(struct drvmgr_dev *dev, int pin, 
	int input) {
	struct gpio_priv *platdata = dev->priv;
	uint32_t v = readl_relaxed(platdata->base + OMAP_GPIO_OE);
	v = (v & ~BIT(pin)) | (input << pin);
	writel_relaxed(v, platdata->base + OMAP_GPIO_OE);
}

static int gpio_bus_irqset(struct drvmgr_dev *dev, int pin,
	unsigned int type) {
	struct gpio_priv *platdata = dev->priv;
    uint32_t level_0, level_1;
    uint32_t edge_0, edge_1;
	uint32_t value = BIT(pin);
	uint32_t rvalue = ~value;
	rtems_interrupt_lock_context lock_context;
	rtems_interrupt_lock_acquire(&platdata->lock, &lock_context);
	level_0 = readl_relaxed(platdata->base + OMAP_GPIO_LEVELDETECT0);
	level_1 = readl_relaxed(platdata->base + OMAP_GPIO_LEVELDETECT1);
	edge_0 = readl_relaxed(platdata->base + OMAP_GPIO_FALLINGDETECT);
	edge_1 = readl_relaxed(platdata->base + OMAP_GPIO_RISINGDETECT);
	switch (type) {
	case GPIO_EDGE_RISING:
		writel_relaxed(value | edge_1, platdata->base + OMAP_GPIO_RISINGDETECT);
		writel_relaxed(rvalue & edge_0, platdata->base + OMAP_GPIO_FALLINGDETECT);
		writel_relaxed(rvalue & level_0, platdata->base + OMAP_GPIO_LEVELDETECT0);
		writel_relaxed(rvalue & level_1, platdata->base + OMAP_GPIO_LEVELDETECT1);
		break;
	case GPIO_EDGE_FALLING:
		writel_relaxed(value | edge_0, platdata->base + OMAP_GPIO_FALLINGDETECT);
		writel_relaxed(rvalue & edge_1, platdata->base + OMAP_GPIO_RISINGDETECT);
		writel_relaxed(rvalue & level_0, platdata->base + OMAP_GPIO_LEVELDETECT0);
		writel_relaxed(rvalue & level_1, platdata->base + OMAP_GPIO_LEVELDETECT1);
		break;
	case GPIO_EDGE_BOTH:
		writel_relaxed(value | edge_0, platdata->base + OMAP_GPIO_FALLINGDETECT);
		writel_relaxed(value | edge_1, platdata->base + OMAP_GPIO_RISINGDETECT);
		writel_relaxed(rvalue & level_0, platdata->base + OMAP_GPIO_LEVELDETECT0);
		writel_relaxed(rvalue & level_1, platdata->base + OMAP_GPIO_LEVELDETECT1);
		break;
	case GPIO_LEVEL_HIGH:
		writel_relaxed(value | level_1, platdata->base + OMAP_GPIO_LEVELDETECT1);
		writel_relaxed(rvalue & edge_0, platdata->base + OMAP_GPIO_FALLINGDETECT);
		writel_relaxed(rvalue & edge_1, platdata->base + OMAP_GPIO_RISINGDETECT);
		writel_relaxed(rvalue & level_0, platdata->base + OMAP_GPIO_LEVELDETECT0);
		break;
	case GPIO_LEVEL_LOW:
		writel_relaxed(value | level_0, platdata->base + OMAP_GPIO_LEVELDETECT0);
		writel_relaxed(rvalue & edge_0, platdata->base + OMAP_GPIO_FALLINGDETECT);
		writel_relaxed(rvalue & edge_1, platdata->base + OMAP_GPIO_RISINGDETECT);
		writel_relaxed(rvalue & level_1, platdata->base + OMAP_GPIO_LEVELDETECT1);
		break;
	default:
		rtems_interrupt_lock_release(&platdata->lock, &lock_context);
		return -EINVAL;
	}
	rtems_interrupt_lock_release(&platdata->lock, &lock_context);
	return 0;
}

static int gpio_bus_configure(struct drvmgr_dev *dev, int pin, 
	unsigned int mode) {
	int it_mode = GPIO_INTR_MASK(mode);
	int io_mode = GPIO_MASK(mode);
	if (it_mode && (io_mode & GPIO_OUTPUT))
		return -EINVAL;
	if (io_mode & GPIO_INPUT) {
		if (it_mode) {
			int ret = gpio_bus_irqset(dev, pin, it_mode);
			if (ret) {
				printk("GPIO interrupt options invalid\n");
				return ret;
			}
		}
		gpio_bus_set_dir(dev, pin, 1);
	} else {
		gpio_bus_set_dir(dev, pin, 0);
	}
	return 0;
}

static int gpio_bus_setpin(struct drvmgr_dev *dev, int pin, 
	int val) {
	struct gpio_priv *platdata = dev->priv;
	rtems_interrupt_lock_context lock_context;
	uint32_t ofs = OMAP_GPIO_CLEARDATAOUT + (val << 2);
	rtems_interrupt_lock_acquire(&platdata->lock, &lock_context);
	writel_relaxed(BIT(pin), platdata->base + ofs);
	rtems_interrupt_lock_release(&platdata->lock, &lock_context);
	return 0;
}

static int gpio_bus_getpin(struct drvmgr_dev *dev, int pin) {
	struct gpio_priv *platdata = dev->priv;
	rtems_interrupt_lock_context lock_context;
	rtems_interrupt_lock_acquire(&platdata->lock, &lock_context);
	uint32_t v = readl_relaxed(platdata->base + OMAP_GPIO_DATAIN);
	rtems_interrupt_lock_release(&platdata->lock, &lock_context);
	return !!(v & BIT(pin));
}

static int gpio_bus_setport(struct drvmgr_dev *dev, uint32_t mask,
	 uint32_t val) {
	struct gpio_priv *platdata = dev->priv;
	rtems_interrupt_lock_context lock_context;
	rtems_interrupt_lock_acquire(&platdata->lock, &lock_context);
	uint32_t v = readl_relaxed(platdata->base + OMAP_GPIO_DATAOUT);
	v = (v & ~mask) | val;
	writel_relaxed(v, platdata->base + OMAP_GPIO_DATAOUT);
	rtems_interrupt_lock_release(&platdata->lock, &lock_context);
	return 0;
}

static int gpio_bus_getport(struct drvmgr_dev *dev, uint32_t mask,
	 uint32_t *val) {
	struct gpio_priv *platdata = dev->priv;
	rtems_interrupt_lock_context lock_context;
	rtems_interrupt_lock_acquire(&platdata->lock, &lock_context);
	uint32_t v = readl_relaxed(platdata->base + OMAP_GPIO_DATAIN);
	*val = v & mask;
	rtems_interrupt_lock_release(&platdata->lock, &lock_context);
	return 0;
}

static void gpio_bus_default_isr(void *arg) {
	(void) arg;
	printk("Please install GPIO ISR\n");
}

static int gpio_bus_unite(struct drvmgr_drv *drv, struct drvmgr_dev *dev) {
	return platform_bus_match(drv, dev, DRVMGR_BUS_TYPE_GPIO);
}

static int gpio_bus_intr_register(struct drvmgr_dev *dev, int index, 
	const char *info, drvmgr_isr isr, void *arg) {
	(void) info;
	struct gpio_priv *platdata = dev->parent->dev->priv;
	if (index >= GPIO_PAD_PINS)
		return -DRVMGR_EINVAL;
	rtems_interrupt_lock_context lock_context;
	rtems_interrupt_lock_acquire(&platdata->lock, &lock_context);
	platdata->enties[index].arg = arg;	
	platdata->enties[index].handler = isr;
	writel_relaxed(BIT(index), platdata->base + OMAP_GPIO_IRQSTATUS_SET_0);
	rtems_interrupt_lock_release(&platdata->lock, &lock_context);
	return DRVMGR_OK;
}

static int gpio_bus_intr_unregister(struct drvmgr_dev *dev, int index, 
	drvmgr_isr isr, void *arg) {
	(void) isr;
	(void) arg;
	struct gpio_priv *platdata = dev->parent->dev->priv;
	if (index >= GPIO_PAD_PINS)
		return -DRVMGR_EINVAL;
	rtems_interrupt_lock_context lock_context;
	rtems_interrupt_lock_acquire(&platdata->lock, &lock_context);
	writel_relaxed(BIT(index), platdata->base + OMAP_GPIO_IRQSTATUS_CLR_0);
	platdata->enties[index].handler = gpio_bus_default_isr;
	platdata->enties[index].arg = NULL;
	rtems_interrupt_lock_release(&platdata->lock, &lock_context);
	return DRVMGR_OK;
}
	
static int gpio_bus_intr_clear(struct drvmgr_dev *dev, int index) {
	struct gpio_priv *platdata = dev->parent->dev->priv;;
	writel_relaxed(BIT(index), platdata->base + OMAP_GPIO_IRQSTATUS1);
	return DRVMGR_OK;
}

static int gpio_bus_intr_mask(struct drvmgr_dev *dev, int index) {
	struct gpio_priv *platdata = dev->parent->dev->priv;
	writel_relaxed(BIT(index), platdata->base + OMAP_GPIO_IRQSTATUS_CLR_0);
	return DRVMGR_OK;
}

static int gpio_bus_intr_unmask(struct drvmgr_dev *dev, int index) {
	struct gpio_priv *platdata = dev->priv;
	writel_relaxed(BIT(index), platdata->base + OMAP_GPIO_IRQSTATUS_SET_0);
	return DRVMGR_OK;
}

#ifdef RTEMS_SMP
static int gpio_bus_intr_set_affinity(struct drvmgr_dev *dev, 
	int index, const Processor_mask *cpus) {
	return DRVMGR_FAIL;
}
#endif

static inline void gpio_entry_isr(struct gpio_entry *entry) {
	entry->handler(entry->arg);
}

static void gpio_entry_init(struct gpio_entry *e) {
	for (int i = 0; i < GPIO_PAD_PINS; i++) {
		e[i].arg = NULL;
		e[i].handler = gpio_bus_default_isr;
	}
}

static void gpio_bus_isr(void *arg) {
	struct gpio_priv *platdata = arg;
	uint32_t status;
    status = readl_relaxed(platdata->base + OMAP_GPIO_IRQSTATUS1);
    writel_relaxed(status, platdata->base + OMAP_GPIO_IRQSTATUS1);
    while (status) {
        int index = __builtin_ctz(status);
        status &= ~BIT(index);
		gpio_entry_isr(platdata->enties + index);
    }
}

static const struct gpio_operations gpio_ops = {
	.configure = gpio_bus_configure,
	.set_port = gpio_bus_setport,
	.get_port = gpio_bus_getport,
	.get_pin = gpio_bus_getpin,
	.set_pin = gpio_bus_setpin
};

static struct drvmgr_bus_ops gpio_bus_ops = {
	.init = {
		platform_bus_populate,
	},
	.remove         = NULL,
	.unite		    = gpio_bus_unite,
	.int_register	= gpio_bus_intr_register,
	.int_unregister	= gpio_bus_intr_unregister,
	.int_clear	    = gpio_bus_intr_clear,
	.int_mask	    = gpio_bus_intr_mask,
	.int_unmask	    = gpio_bus_intr_unmask,
#ifdef RTEMS_SMP
	.int_set_affinity = gpio_bus_intr_set_affinity
#endif
	.get_params	    = NULL,
	.get_freq       = NULL
};

static int gpio_bus_init(struct drvmgr_dev *dev) {
	struct gpio_priv *platdata;
    struct dev_private *devp;
	int ret;
    platdata = rtems_calloc(1, sizeof(struct gpio_priv));
    if (platdata == NULL) 
        return -DRVMGR_NOMEM;
    devp = device_get_private(dev);
	gpio_entry_init(platdata->enties);
	rtems_interrupt_lock_initialize(&platdata->lock, dev->name);
	devp->devops = &gpio_ops;
    platdata->base = devp->base;
	platdata->dev = dev;
	dev->priv = platdata;
    ret = drvmgr_interrupt_register(dev, 0, dev->name, 
		gpio_bus_isr, platdata);
    if (ret) {
        printk("Register irq for %s failed\n", dev->name);
        goto _free;
    }
	ret = platform_bus_device_register(dev, &gpio_bus_ops, 
		DRVMGR_BUS_TYPE_GPIO);
	if (ret) {
		printk("Register GPIO bus device（%s） failed\n", dev->name);
		goto _free;
	}
	return 0;
_free:
	free(platdata);
	return ret;
}

static const struct dev_id id_table[] = {
    {.compatible = "ti,am4372-gpio", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops gpio_driver_ops = {
	.init = {
		gpio_bus_init,
	},
};
		
PLATFORM_DRIVER(gpio_bus) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "gpio",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &gpio_driver_ops
	},
	.ids = id_table
};
