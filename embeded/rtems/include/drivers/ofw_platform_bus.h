/*
 * CopyRight(c) 2022 wtcat
 */
#ifndef DRIVER_OFW_PLATFORM_BUS_H_
#define DRIVER_OFW_PLATFORM_BUS_H_

#include <drvmgr/drvmgr.h>
#include <ofw/ofw.h>
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
    phandle_t np; /* Device tree node */
};

phandle_t ofw_platform_bus_get_node(struct drvmgr_dev *dev);
int ofw_platform_bus_populate_device(struct drvmgr_bus *bus);
int ofw_platform_bus_device_register(struct drvmgr_dev *dev,
	struct drvmgr_bus_ops *bus_ops, int bustype);
int ofw_platform_bus_match(struct drvmgr_drv *drv, struct drvmgr_dev *dev, 
	int bustype);
const struct dev_id *ofw_device_match(struct drvmgr_dev *dev, 
	const struct dev_id *id_table);


static inline void *device_match_data(struct drvmgr_dev *dev) {
	const struct dev_id *id;
	id = ofw_device_match(dev, ((struct dev_driver *)dev->drv)->ids);
	return id->data;
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

#define _ofw_platform_driver_init(drv) \
	RTEMS_STATIC_ASSERT(sizeof(drv) == sizeof(struct dev_driver), \
		"Device driver object type error!"); \
	static void platform_driver_##drv##_register(void) { \
		drvmgr_drv_register((struct drvmgr_drv *)&drv); \
	}	\
	RTEMS_SYSINIT_ITEM(platform_driver_##drv##_register, \
		RTEMS_SYSINIT_DRVMGR,  \
		RTEMS_SYSINIT_ORDER_MIDDLE \
	)

#define OFW_PLATFORM_DRIVER(name) \
	static struct dev_driver __ofw_drv_##name; \
	_ofw_platform_driver_init(__ofw_drv_##name); \
	static struct dev_driver __ofw_drv_##name

#ifdef __cplusplus
}
#endif
#endif /* DRIVER_OFW_PLATFORM_BUS_H_ */
