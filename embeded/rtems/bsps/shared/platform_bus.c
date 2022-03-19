/*
 * CopyRight(c) 2022 wtcat
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <rtems/sysinit.h>
#include <rtems/bspIo.h>

#include <bsp.h>
#include <bsp/irq-generic.h>
#include <bsp/platform_bus.h>

#include <drvmgr/drvmgr.h>

#define RES_NAME_SIZE 32

static const struct bus_resource *const *platform_resources;

const struct bus_resource *const *platform_res_get(void) {
	return platform_resources;
}

int platform_res_register(const struct bus_resource *const *r) {
	if (platform_resources == NULL) {
		platform_resources = r;
		return 0;
	}
	return -DRVMGR_EBUSY;
}

int platform_res_count_get(struct drvmgr_key *keys, 
	const char *name, size_t len) {
	struct drvmgr_key *key = keys;
	if (!keys)
		return 0;
	int count = 0;
	while (key->key_type != DRVMGR_KT_NONE) {
		if (strncmp(name, key->key_name, len) == 0)
			count++;
		key++;
	}
	return count;
}

void *platform_resource_get(struct drvmgr_key *keys, enum drvmgr_kt key_type, 
	const char *fmt, ...) {
	char name[RES_NAME_SIZE];
	va_list ap;
	va_start(ap, fmt) ;
	vsnprintf(name, RES_NAME_SIZE-1, fmt, ap);
	va_end(ap);
	return drvmgr_key_val_get(keys, name, key_type);
}

int platform_reg_resource_get(struct drvmgr_key *keys, int index,
	unsigned int *reg) {
	union drvmgr_key_value *v = platform_resource_get(keys, 
		DRVMGR_KT_INT, "REG%d", index);
	if (!v) {
		if (index == 0) {
			v = drvmgr_key_val_get(keys, "REG", DRVMGR_KT_INT);
			if (v == NULL)
				return -DRVMGR_ENORES;
		}
		return -DRVMGR_ENORES;
	}
	*reg = v->i;
	return 0;
}

int platform_irq_resource_get(struct drvmgr_key *keys, int index,
	unsigned int *oirq) {
	union drvmgr_key_value *v = platform_resource_get(keys, 
		DRVMGR_KT_INT, "IRQ%d", index);
	if (!v) {
		if (index == 0) {
			v = drvmgr_key_val_get(keys, "IRQ", DRVMGR_KT_INT);
			if (v == NULL)
				return -DRVMGR_ENORES;
		}
		return -DRVMGR_ENORES;
	}
	*oirq = v->i;
	return 0;
}

const struct dev_id *device_match(struct drvmgr_dev *dev, 
	const struct dev_id *id_table) {
	struct dev_private *priv = (struct dev_private *)dev->businfo;
	while (id_table->compatible) {
		if (!strcmp(priv->res->compatible, id_table->compatible))
			return id_table;
		id_table++;
	}
	return NULL;
}

int platform_bus_match(struct drvmgr_drv *drv, struct drvmgr_dev *dev, 
	int bustype) {
	if (!drv || !dev || !dev->parent || !dev->businfo)
		return 0;
	if (drv->bus_type != bustype ||
		dev->parent->bus_type != bustype)
		return 0;
	struct dev_driver *ddrv = RTEMS_CONTAINER_OF(drv, struct dev_driver, drv);
	struct dev_private *priv = (struct dev_private *)dev->businfo;
	while (ddrv->ids->compatible) {
		if (!strcmp(priv->res->compatible, ddrv->ids->compatible))
			return 1;
		ddrv->ids++;
	}
	return 0;
}
	
int platform_irq_map(struct drvmgr_dev *dev, int index) {
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

static int platform_bus_unite(struct drvmgr_drv *drv, struct drvmgr_dev *dev) {
	return platform_bus_match(drv, dev, DRVMGR_BUS_TYPE_PLATFORM);
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
static int platform_bus_intr_set_affinity(struct drvmgr_dev *dev, 
	int index, const Processor_mask *cpus) {
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

int platform_dev_register(struct drvmgr_bus *parent,
	const struct bus_resource *r) {
	struct drvmgr_dev *dev;	
	int nr = platform_irq_count_get((struct drvmgr_key *)r->keys);
	drvmgr_alloc_dev(&dev, sizeof(struct dev_private) + nr * sizeof(short));
	struct dev_private *priv = device_get_private(dev);
	if (platform_reg_resource_get((struct drvmgr_key *)r->keys, 0, &priv->base)) {
		printk(DRVMGR_WARN "%s not found \"REG(0)\" resource(%s)\n", 
			r->name?: r->compatible);
		free(dev);
		return -DRVMGR_ENORES;
	}
	for (int i = 0; i < nr; i++) {
		unsigned int irqno;
		if (platform_irq_resource_get((struct drvmgr_key *)r->keys, i, &irqno)) {
			printk(DRVMGR_WARN "%s not found \"IRQ%d\" resource(%s)\n", 
				r->name?: r->compatible, i);
			free(dev);
			return -DRVMGR_ENORES;
		}
		priv->irqs[i] = (unsigned short)irqno;
	}
	priv->res = r;
	priv->nirq = nr;
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

int platform_dev_populate_on_bus(struct drvmgr_bus *bus,
	const struct bus_resource *const *r) {
	int ret;
	if (r == NULL)
		return DRVMGR_FAIL;
	while (*r) {
		if (!(*r)->compatible)
			continue;
		if ((*r)->parent_bus != bus->bus_type ||
			(*r)->parent_busid != bus->dev->minor_bus)
			continue;
		ret = platform_dev_register(bus, *r);
		if (ret)
			return ret;
		r++;
	}
	return DRVMGR_OK;
}

int platform_bus_device_register(struct drvmgr_dev *dev,
	struct drvmgr_bus_ops *bus_ops, int bustype) {
	if (!dev->name)
		return -DRVMGR_EINVAL;
	drvmgr_alloc_bus(&dev->bus, 0);
	dev->bus->bus_type = bustype;
	dev->bus->next = NULL;
	dev->bus->dev = dev;
	dev->bus->priv = NULL;
	dev->bus->children = NULL;
	dev->bus->ops = bus_ops;
	dev->bus->dev_cnt = 0;
	dev->bus->reslist = NULL;
	dev->bus->maps_up = NULL;
	dev->bus->maps_down = NULL;
	dev->name = dev->name;
	dev->priv = NULL;
	return drvmgr_bus_register(dev->bus);
}

static struct drvmgr_bus_ops platform_bus_ops = {
	.init = {
		platform_bus_populate,
	},
	.remove         = NULL,
	.unite		    = platform_bus_unite,
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

static int platform_bus_init(struct drvmgr_dev *dev) {
	return platform_bus_device_register(dev, &platform_bus_ops, 
		DRVMGR_BUS_TYPE_PLATFORM);
}

static struct drvmgr_drv_ops platform_driver_ops = {
	.init = {
		platform_bus_init,
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
