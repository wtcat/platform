/*
 * CopyRight(c) 2022 wtcat
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <rtems/sysinit.h>
#include <rtems/bspIo.h>

#include <bsp.h>
#include <bsp/irq-generic.h>
#include <bsp/platform_bus.h>

#include <drvmgr/drvmgr.h>


static const struct bus_resource *platform_resources;


static int reg_resource_get(struct drvmgr_key *keys, unsigned int *reg) {
	union drvmgr_key_value *v;
	v = drvmgr_key_val_get(keys, "REG0", DRVMGR_KT_INT);
	if (v == NULL)
		v = drvmgr_key_val_get(keys, "REG", DRVMGR_KT_INT);
	if (v) {
		*reg = v->i;
		return 0;
	}
	return DRVMGR_ENORES;
}

static int irq_resource_get(struct drvmgr_key *keys, struct irq_res *irqres) {
	union drvmgr_key_value *v;
	char name[5] = {"IRQ0"};
	for (irqres-> = 0; i < MAX_IRQRES_NR; i++) {
		name[3] = '0' + i;
		v = drvmgr_key_val_get(keys, name, DRVMGR_KT_INT);
		if (v == NULL)
			continue;
		irqres->irqs = (unsigned short)v->i;
		irqres->nr++;
	}
	return (irqres->nr == 0)? DRVMGR_ENORES: 0;
}

static int platform_irq_map(struct drvmgr_dev *dev, int index) {
	struct dev_private *priv;
	if (!dev)
		return -DRVMGR_EINVAL;
	priv = dev->businfo;
	if (index >= priv->irqres.nr)
		return -DRVMGR_EINVAL;
	if (index >= 0) 
		return priv->irqres.irqs[index];
	return -index;
}

static int platform_bus_match(struct drvmgr_drv *drv, struct drvmgr_dev *dev) {
	if (!drv || !dev || !dev->parent || !dev->businfo)
		return 0;
	if (drv->bus_type != DRVMGR_BUS_TYPE_PLATFORM ||
		dev->parent->bus_type != DRVMGR_BUS_TYPE_PLATFORM)
		return 0;
	
	struct dev_driver *ddrv = RTEMS_CONTAINER_OF(drv, struct dev_driver, ids);
	struct dev_private *priv = (struct dev_private *)dev->businfo;
	while (ddrv->ids) {
		if (!strcmp(priv->compatible, ddrv->ids->compatible))
			return 1;
		ddrv->ids++;
	}
	return 0;
}

static int platform_bus_intr_register(struct drvmgr_dev *dev, int index, 
	const char *info, drvmgr_isr isr, void *arg) {
	rtems_status_code sc;
	(void) dev;
	int irq = platform_irq_map(dev, index);
	if (irq < 0)
		return DRVMGR_FAIL;
	sc = rtems_interrupt_handler_install((rtems_vector_number)irq, info, 
		RTEMS_INTERRUPT_SHARED, isr, arg);
	return (int)sc;
}

static int platform_bus_intr_unregister(struct drvmgr_dev *dev, int index, 
	drvmgr_isr isr, void *arg) {
	(void) dev;
	int irq = platform_irq_map(dev, index);
	if (irq < 0)
		return DRVMGR_FAIL;
	return rtems_interrupt_handler_remove((rtems_vector_number)irq, isr, arg);
}
	
static int platform_bus_intr_clear(struct drvmgr_dev *dev, int index) {
	return DRVMGR_FAIL;
}

static int platform_bus_intr_mask(struct drvmgr_dev *dev, int index) {
	int irq = platform_irq_map(dev, index);
	if (irq < 0)
		return DRVMGR_FAIL;
	return rtems_interrupt_vector_disable((rtems_vector_number)irq);
}

static int platform_bus_intr_unmask(struct drvmgr_dev *dev, int index) {
	int irq = platform_irq_map(dev, index);
	if (irq < 0)
		return DRVMGR_FAIL;
	return rtems_interrupt_vector_enable((rtems_vector_number)irq);
}

#ifdef RTEMS_SMP
static int platform_bus_intr_set_affinity(struct drvmgr_dev *dev, int index,
	const Processor_mask *cpus) {
	return DRVMGR_FAIL;
}
#endif

static int platform_bus_get_params(struct drvmgr_dev *dev, 
	struct drvmgr_bus_params *param) {
	return DRVMGR_FAIL;
}
	
static int platform_bus_get_freq(struct drvmgr_dev *dev, int no, 
	unsigned int *freq) {
	return DRVMGR_FAIL;
}

int platform_res_register(const struct bus_resource *r) {
	if (platform_resources == NULL) {
		platform_resources = r;
		return 0;
	}
	return DRVMGR_EBUSY;
}

const struct bus_resource *platform_res_get(void) {
	return platform_resources;
}


int platform_dev_register(struct drvmgr_bus *parent,
	const struct bus_resource *r) {
	struct drvmgr_dev *dev;
	drvmgr_alloc_dev(&dev, sizeof(struct dev_private));
	struct dev_private *priv = (struct dev_private *)(dev + 1);
	priv->compatible = r->comatible;
	if (irq_resource_get((struct drvmgr_key *)r->keys, &priv->irqres)) {
		printk(DRVMGR_WARN "Not found \"irq\" resource(%s)\n", 
			r->name?:r->comatible);
	}
	if (reg_resource_get((struct drvmgr_key *)r->keys, &priv->regbase)) {
		printk(DRVMGR_WARN "Not found \"reg\" resource(%s)\n", 
			r->name?:r->comatible);
		free(dev);
		return DRVMGR_ENORES;
	}
	dev->next = NULL;
	dev->parent = parent;
	dev->minor_drv = 0;
	dev->minor_bus = 0;
	dev->businfo = priv;
	dev->priv = NULL;
	dev->drv = NULL;
	dev->name = (char *)r->name;
	dev->next_in_drv = NULL;
	dev->bus = NULL;
	drvmgr_dev_register(dev);
	return 0;
}

static int platform_bus_populate(struct drvmgr_bus *bus) {
	const struct bus_resource *r = platform_resources;
	if (r == NULL)
		return DRVMGR_FAIL;
	while (r->comatible) {
		platform_dev_register(bus, platform_resources);
		r++;
	}
	return DRVMGR_OK;
}

static struct drvmgr_bus_ops platform_bus_ops = {
	.init = {
		platform_bus_populate,
	},
	.remove         = NULL,
	.unite		    = platform_bus_match,
	.int_register	= platform_bus_intr_register,
	.int_unregister	= platform_bus_intr_unregister,
	.int_clear	    = platform_bus_intr_clear,
	.int_mask	    = platform_bus_intr_mask,
	.int_unmask	    = platform_bus_intr_unmask,
#ifdef RTEMS_SMP
	.int_set_affinity = platform_bus_intr_set_affinity
#endif
	.get_params	    = platform_bus_get_params,
	.get_freq       = platform_bus_get_freq
};

static int platform_bus_device_register(struct drvmgr_dev *dev) {
	drvmgr_alloc_bus(&dev->bus, 0);
	dev->bus->bus_type = DRVMGR_BUS_TYPE_PLATFORM;
	dev->bus->next = NULL;
	dev->bus->dev = dev;
	dev->bus->priv = NULL;
	dev->bus->children = NULL;
	dev->bus->ops = &platform_bus_ops;
	dev->bus->dev_cnt = 0;
	dev->bus->reslist = NULL;
	dev->bus->maps_up = NULL;
	dev->bus->maps_down = NULL;
	dev->name = "root-device";
	dev->priv = NULL;
	return drvmgr_bus_register(dev->bus);
}

static struct drvmgr_drv_ops platform_driver_ops {
	.init = {
		platform_bus_device_register,
		},
	.remove = NULL,
	.info = NULL
};
		
static struct drvmgr_drv platform_bus_driver = {
	.obj_type = DRVMGR_OBJ_DRV,
	.drv_id   = DRIVER_ROOT_ID,
	.name     = "root-bus",
	.bus_type = DRVMGR_BUS_TYPE_ROOT,
	.ops      = &platform_driver_ops
};

static void platform_bus_driver_register(void) {
	/* Register root device driver */
	drvmgr_root_drv_register(&platform_bus_driver);
}

RTEMS_SYSINIT_ITEM(platform_bus_driver_register,
	RTEMS_SYSINIT_BSP_PRE_DRIVERS,
	RTEMS_SYSINIT_ORDER_MIDDLE
);

