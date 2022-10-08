/*
 * CopyRight(c) 2022 wtcat
 */
#ifndef DRIVER_OFW_PLATFORM_BUS_H_
#define DRIVER_OFW_PLATFORM_BUS_H_

#include <drvmgr/drvmgr.h>
#include "base/compiler.h"

#ifdef __cplusplus
extern "C"{
#endif

#define DRVMGR_WARN "DRVMGR_WARNING: " 

/*
 * System Interupt Flags
 */
#define IRQF_INDEX(n)           ((uint32_t)(n) & 0xFFFF)
#define IRQF_ABS                (0x1u << 16)
#define IRQF_THREAD             (0x1u << 26)
#define IRQF_THREADED(n)        ((((uint32_t)(n) & 0x1F) << 27) | IRQF_THREAD)
#define IRQF_NTHREAD(n)         (((uint32_t)(n) >> 27) & 0x1F)

#define IRQF_SOFT_REL(svr, irq) (IRQF_THREADED(svr) | IRQF_INDEX(irq))
#define IRQF_SOFT(svr, irq)     (IRQF_SOFT_REL(svr, irq) | IRQF_ABS)
#define IRQF_HARD_REL(irq)      IRQF_INDEX(irq)
#define IRQF_HARD(irq)          (IRQF_HARD_REL(irq) | IRQF_ABS)

/* 
 * Bus types 
 */
enum drvmgr_bus_type {
	DRVMGR_BUS_TYPE_PLATFORM	= 10,
#define DRIVER_PLATFORM_ID		DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_PLATFORM)
	DRVMGR_BUS_TYPE_DMA,
#define DRIVER_DMA_ID		    DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_DMA)
	DRVMGR_BUS_TYPE_GPIO,
#define DRIVER_GPIO_ID		    DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_GPIO)
	DRVMGR_BUS_TYPE_I2C,
#define DRIVER_I2C_ID		    DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_I2C)
	DRVMGR_BUS_TYPE_SPI,
#define DRIVER_SPI_ID		    DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_SPI)
};
	
struct dev_id {
	const char *compatible;
	void *data;
};

struct dev_driver {
	struct drvmgr_drv drv;
	const struct dev_id *const ids;
};

struct dev_private {
	const void *devops;  /* Device operations */
    phandle_t np;
};

int platform_res_register(const struct bus_resource *const *r);
int platform_dev_register(struct drvmgr_bus *parent,
	const struct bus_resource *r);
int platform_bus_match(struct drvmgr_drv *drv, struct drvmgr_dev *dev, 
	int bustype);
int platform_dev_populate_on_bus(struct drvmgr_bus *bus,
	const struct bus_resource *const *r);
int platform_bus_device_register(struct drvmgr_dev *dev,
	struct drvmgr_bus_ops *bus_ops, int bustype);


static inline const char *platform_dev_filename(struct drvmgr_dev *dev) {
	return dev->name - 5;
}

static inline void *device_get_parent_priv(struct drvmgr_dev *dev) {
	return dev->parent->dev->priv;
}

static inline struct dev_private *device_get_private(struct drvmgr_dev *dev) {
	return (struct dev_private *)(dev + 1);
}

static inline struct drvmgr_dev *device_get_parent(struct drvmgr_dev *dev) {
	return dev->parent->dev;
}

static inline const void *device_get_operations(struct drvmgr_dev *dev) {
	return *(void **)(dev + 1);
}

static inline union drvmgr_key_value *
devcie_get_property(struct drvmgr_dev *dev, const char *prop) {
	struct dev_private *priv = device_get_private(dev);
	return drvmgr_key_val_get((struct drvmgr_key *)priv->res->keys, 
		(char *)prop, DRVMGR_KT_ANY);
}

#define platform_driver_init(drv) \
	RTEMS_STATIC_ASSERT(sizeof(drv) == sizeof(struct dev_driver), \
		"Device driver object type error!"); \
	static void platform_driver_##drv##_register(void) { \
		drvmgr_drv_register((struct drvmgr_drv *)&drv); \
	}	\
	RTEMS_SYSINIT_ITEM(platform_driver_##drv##_register, \
		RTEMS_SYSINIT_DRVMGR,  \
		RTEMS_SYSINIT_ORDER_MIDDLE \
	)

#define PLATFORM_DRIVER(name) \
	static struct dev_driver __drv_##name; \
	platform_driver_init(__drv_##name); \
	static struct dev_driver __drv_##name

#ifdef __cplusplus
}
#endif
#endif /* DRIVER_OFW_PLATFORM_BUS_H_ */
