#include <rtems/bspIo.h>

#include <bsp.h>
#include <drvmgr/drvmgr.h>

#include "bsp/gpio_bus.h"


struct gpio_entry {
	void (*handler)(void *arg);
	void *arg;
};

struct gpio_priv {
	struct drvmgr_dev *dev;
	unsigned int port;
};


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

int gpio_bus_irq_map(struct drvmgr_dev *dev, int index) {
	struct dev_private *priv;
	if (!dev)
		return -DRVMGR_EINVAL;
	priv = dev->businfo;
	if (index >= (int)priv->nirq)
		return -DRVMGR_EINVAL;
	if (index >= 0) 
		return priv->irqs[index];
	return -index;
}

static int gpio_bus_unite(struct drvmgr_drv *drv, struct drvmgr_dev *dev) {
	return platform_bus_match(drv, dev, DRVMGR_BUS_TYPE_GPIO);
}

static int gpio_bus_intr_register(struct drvmgr_dev *dev, int index, 
	const char *info, drvmgr_isr isr, void *arg) {
	rtems_status_code sc;
	(void) dev;
	int irq = gpio_bus_irq_map(dev, index);
	if (irq < 0)
		return DRVMGR_FAIL;
	sc = rtems_interrupt_handler_install((rtems_vector_number)irq, info, 
		RTEMS_INTERRUPT_SHARED, isr, arg);
	return (int)sc;
}

static int gpio_bus_intr_unregister(struct drvmgr_dev *dev, int index, 
	drvmgr_isr isr, void *arg) {
	(void) dev;
	int irq = gpio_bus_irq_map(dev, index);
	if (irq < 0)
		return DRVMGR_FAIL;
	return rtems_interrupt_handler_remove((rtems_vector_number)irq, isr, arg);
}
	
static int gpio_bus_intr_clear(struct drvmgr_dev *dev, int index) {
	return DRVMGR_FAIL;
}

static int gpio_bus_intr_mask(struct drvmgr_dev *dev, int index) {
	int irq = gpio_bus_irq_map(dev, index);
	if (irq < 0)
		return DRVMGR_FAIL;
	return rtems_interrupt_vector_disable((rtems_vector_number)irq);
}

static int gpio_bus_intr_unmask(struct drvmgr_dev *dev, int index) {
	int irq = gpio_bus_irq_map(dev, index);
	if (irq < 0)
		return DRVMGR_FAIL;
	return rtems_interrupt_vector_enable((rtems_vector_number)irq);
}

#ifdef RTEMS_SMP
static int gpio_bus_intr_set_affinity(struct drvmgr_dev *dev, 
	int index, const Processor_mask *cpus) {
	return DRVMGR_FAIL;
}
#endif

static int gpio_bus_get_params(struct drvmgr_dev *dev, 
	struct drvmgr_bus_params *param) {
	return DRVMGR_FAIL;
}
	
static int gpio_bus_get_freq(struct drvmgr_dev *dev, int no, 
	unsigned int *freq) {
	return DRVMGR_FAIL;
}

static inline int platform_bus_populate(struct drvmgr_bus *bus) {
	return platform_dev_populate_on_bus(bus, platform_res_get(), 
		platform_dev_register);
}

static void gpio_bus_isr(void *arg) {
	struct gpio_priv *platdata;
}

static int gpio_bus_init(struct drvmgr_dev *dev) {
	struct gpio_priv *platdata;
    struct dev_private *devp;
	int ret;

    platdata = rtems_calloc(1, sizeof(struct gpio_priv));
    if (platdata == NULL) 
        return -DRVMGR_NOMEM;
    devp = device_get_private(dev);
    platdata->port = devp->base;
	dev->priv = platdata;
    ret = drvmgr_interrupt_register(dev, 0, dev->name, 
		gpio_bus_isr, NULL);
    if (ret) {
		free();
        printk("Register irq for %s failed\n", dev->name);
        return ret;
    }
	return platform_bus_device_register(dev, &gpio_bus_ops, 
		DRVMGR_BUS_TYPE_GPIO);
}

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
	.get_params	    = gpio_bus_get_params,
	.get_freq       = gpio_bus_get_freq
};

static const struct dev_id id_table[] = {
    {.compatible = "ti,am4372-gpio", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops gpio_driver_ops = {
	.init = {
		gpio_bus_init,
	},
	.remove = NULL,
	.info = NULL
};
		
PLATFORM_DRIVER(gpio_bus) = {
	.drv = {
		.obj_type = DRVMGR_OBJ_DRV,
		.drv_id   = DRIVER_GPIO_ID,
		.name     = "root-bus",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &gpio_driver_ops
	},
	.ids = id_table
}
