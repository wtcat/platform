/*
 * CopyRight(c) 2022 wtcat
 */
#include <errno.h>
#include <string.h>
#include <rtems/sysinit.h>
#include <rtems/bspIo.h>
#include <bsp/irq-generic.h>

#include "drivers/ofw_platform_bus.h"


#define OFW_DEBUG_ON

#ifdef OFW_DEBUG_ON
#define ofw_dbg(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define ofw_dbg(...)
#endif

#define OFW_FOREACH_DEVICE(head, pnode) \
	for (pnode = rtems_chain_first(head); \
		!rtems_chain_is_tail(head, pnode); \
		pnode = rtems_chain_next(pnode))

static RTEMS_CHAIN_DEFINE_EMPTY(ofw_device_list);

static struct drvmgr_dev *device_from_private(struct dev_private *priv) {
	struct drvmgr_dev *xdev = (struct drvmgr_dev *)priv;
	return xdev - 1;
}

const struct dev_id *ofw_device_match(struct drvmgr_dev *dev, 
	const struct dev_id *id_table) {
	struct dev_private *devp = device_get_private(dev);
	while (id_table->compatible) {
		if (rtems_ofw_is_node_compatible(devp->np, id_table->compatible))
			return id_table;
		id_table++;
	}
	return NULL;
}

phandle_t ofw_phandle_get(struct drvmgr_dev *dev) {
	struct dev_private *priv = device_get_private(dev);
	return priv->np;
}

struct drvmgr_dev *ofw_device_get_by_phandle(phandle_t np) {
	struct drvmgr_dev *dev;
	rtems_chain_node *pnode;
	OFW_FOREACH_DEVICE(&ofw_device_list, pnode) {
		struct dev_private *priv = RTEMS_CONTAINER_OF(pnode, 
			struct dev_private, node);
		dev = device_from_private(priv);
		if (np == (phandle_t)dev->businfo)
			return dev;
	}
	return NULL;
}

struct drvmgr_dev *ofw_device_get_by_devnode(phandle_t devnode) {
	struct drvmgr_dev *dev;
	rtems_chain_node *pnode;
	OFW_FOREACH_DEVICE(&ofw_device_list, pnode) {
		struct dev_private *priv = RTEMS_CONTAINER_OF(pnode, 
			struct dev_private, node);
		dev = device_from_private(priv);
		if (devnode == priv->np)
			return dev;
	}
	return NULL;
}

struct drvmgr_dev *ofw_device_get_by_path(const char *path) {
	phandle_t np = rtems_ofw_find_device(path);
	if ((int)np >= 0) 
		return ofw_device_get_by_devnode(np);
	return NULL;
}

int ofw_platform_bus_match(struct drvmgr_drv *drv, struct drvmgr_dev *dev, 
	int bustype) {
	struct dev_driver *ddrv;
	if (!drv || !dev || !dev->parent)
		return 0;
	if (drv->bus_type != bustype ||
		dev->parent->bus_type != bustype)
		return 0;
	ddrv = RTEMS_CONTAINER_OF(drv, struct dev_driver, drv);
	if (ddrv->ids) {
		struct dev_private *priv = device_get_private(dev);
		for (int index = 0; ddrv->ids[index].compatible; index++) {
			if (rtems_ofw_is_node_compatible(priv->np, ddrv->ids[index].compatible))
				goto _ok;
		}
		return 0;
	}
_ok:
	ofw_dbg("device register: %s\n", dev->name);
	return 1;
}
	
int ofw_platform_irq_map(struct drvmgr_dev *dev, int index) {
	if (!dev || !(index & IRQF_ABS))
		return -DRVMGR_EINVAL;
	return IRQF_INDEX(index);
}

static int ofw_platform_bus_unite(struct drvmgr_drv *drv, struct drvmgr_dev *dev) {
	return ofw_platform_bus_match(drv, dev, DRVMGR_BUS_TYPE_PLATFORM);
}

static int ofw_platform_bus_intr_register(struct drvmgr_dev *dev, int index, 
	const char *info, drvmgr_isr isr, void *arg) {
	rtems_status_code sc;
	int irq = ofw_platform_irq_map(dev, index);
	if (irq < 0)
		return DRVMGR_FAIL;
	if (index & IRQF_THREAD) {
		sc = rtems_interrupt_server_handler_install(IRQF_NTHREAD(index), 
			(rtems_vector_number)irq, info, RTEMS_INTERRUPT_SHARED, isr, arg);
	} else {
		sc = rtems_interrupt_handler_install((rtems_vector_number)irq, info, 
			RTEMS_INTERRUPT_SHARED, isr, arg);
	}
	return (int)sc;
}

static int ofw_platform_bus_intr_unregister(struct drvmgr_dev *dev, int index, 
 drvmgr_isr isr, void *arg) {
	rtems_status_code sc;
	int irq = ofw_platform_irq_map(dev, index);
	if (irq < 0)
		return DRVMGR_FAIL;
	if (index & IRQF_THREAD) {
		sc = rtems_interrupt_server_handler_remove(IRQF_NTHREAD(index), 
			(rtems_vector_number)irq, isr, arg);
	} else {
		sc = rtems_interrupt_handler_remove((rtems_vector_number)irq, isr, arg);
	}
	return sc;
}
	
static int ofw_platform_bus_intr_clear(struct drvmgr_dev *dev, int index) {
	(void) dev;
	(void) index;
	return DRVMGR_FAIL;
}

static int ofw_platform_bus_intr_mask(struct drvmgr_dev *dev, int index) {
	int irq = ofw_platform_irq_map(dev, index);
	if (irq < 0)
		return DRVMGR_FAIL;
	return rtems_interrupt_vector_disable((rtems_vector_number)irq);
}

static int ofw_platform_bus_intr_unmask(struct drvmgr_dev *dev, int index) {
	int irq = ofw_platform_irq_map(dev, index);
	if (irq < 0)
		return DRVMGR_FAIL;
	return rtems_interrupt_vector_enable((rtems_vector_number)irq);
}

#ifdef RTEMS_SMP
static int ofw_platform_bus_intr_set_affinity(struct drvmgr_dev *dev, 
	int index, const Processor_mask *cpus) {
	return DRVMGR_FAIL;
}
#endif

static int ofw_platform_bus_get_params(struct drvmgr_dev *dev, 
	struct drvmgr_bus_params *param) {
	(void) dev;
	(void) param;
	return DRVMGR_FAIL;
}

static int ofw_platform_bus_get_freq(struct drvmgr_dev *dev, int no, 
	unsigned int *freq) {
	(void) dev;
	(void) no;
	(void) freq;
	return DRVMGR_FAIL;
}

int __ofw_bus_populate_device(struct drvmgr_bus *bus, phandle_t parent, 
	int (*filter)(phandle_t, char *devname, size_t max)) {
	struct dev_private *devp;
    struct drvmgr_dev *dev;
    phandle_t child;
	pcell_t prop;
    char buffer[128];
    int len, err = -EINVAL;

    ofw_foreach_child_node(parent, child) {
       if (!rtems_ofw_node_status(child))
            continue;
		if (filter) {
			len = filter(child, buffer, sizeof(buffer));
			if (len <= 0)
				continue;
		} else {
			if (!rtems_ofw_has_prop(child, "compatible"))
				continue;
			len = rtems_ofw_get_prop(child, "rtems,path", buffer, sizeof(buffer));
			if (len <= 0)
				continue;
		}

        drvmgr_alloc_dev(&dev, sizeof(struct dev_private) + len + 1);
        _Assert(dev != NULL);
        devp = device_get_private(dev);
        char *name = (char *)devp + sizeof(struct dev_private);
        memcpy(name, buffer, len);
        devp->np = child;
        dev->parent = bus;
        dev->name = name;
		if (rtems_ofw_get_enc_prop(devp->np, "phandle", &prop, sizeof(prop)) > 0) 
			dev->businfo = (void *)prop;
		rtems_chain_append(&ofw_device_list, &devp->node);
        err = drvmgr_dev_register(dev);
        if (err) {
			printk("device register %s failed(%d)\n", name, err);
            break;
		}
    }
    return err;	
}

int ofw_bus_populate_device(struct drvmgr_bus *bus, phandle_t parent) {
	return __ofw_bus_populate_device(bus, parent, NULL);
}

int ofw_platform_bus_populate_device(struct drvmgr_bus *bus) {
	struct dev_private *devp =device_get_private(bus->dev);
	return ofw_bus_populate_device(bus, devp->np);
}

int ofw_platform_bus_device_register(struct drvmgr_dev *dev,
	const struct drvmgr_bus_ops *bus_ops, int bustype) {
	if (!dev->name)
		return -DRVMGR_EINVAL;
	drvmgr_alloc_bus(&dev->bus, 0);
	dev->bus->bus_type = bustype;
	dev->bus->dev = dev;
	dev->bus->ops = (void *)bus_ops;
	return drvmgr_bus_register(dev->bus);
}

static const struct dev_id id_table[] = {
    {.compatible = "simple-bus", NULL},
    {NULL, NULL}
};

static struct drvmgr_bus_ops platform_bus_ops = {
	.init = {
		ofw_platform_bus_populate_device,
	},
	.remove         = NULL,
	.unite		    = ofw_platform_bus_unite,
	.int_register	= ofw_platform_bus_intr_register,
	.int_unregister	= ofw_platform_bus_intr_unregister,
	.int_clear	    = ofw_platform_bus_intr_clear,
	.int_mask	    = ofw_platform_bus_intr_mask,
	.int_unmask	    = ofw_platform_bus_intr_unmask,
#ifdef RTEMS_SMP
	.int_set_affinity = ofw_platform_bus_intr_set_affinity
#endif
	.get_params	    = ofw_platform_bus_get_params,
	.get_freq       = ofw_platform_bus_get_freq
};

static int platform_bus_probe(struct drvmgr_dev *dev) {
	return ofw_platform_bus_device_register(dev, &platform_bus_ops, 
		DRVMGR_BUS_TYPE_PLATFORM);
}

static struct drvmgr_drv_ops platform_bus_driver = {
	.init = {
        platform_bus_probe,
	},
};
		
OFW_PLATFORM_DRIVER(platform_bus) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "platform-bus",
		.bus_type = DRVMGR_BUS_TYPE_ROOT,
		.ops      = &platform_bus_driver
	},
	.ids = id_table
};
