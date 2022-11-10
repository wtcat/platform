/*
 * CopyRight(c) 2022 wtcat
 */
#ifndef DRIVER_DEV_BASE_H_
#define DRIVER_DEV_BASE_H_

#include "rtems/score/basedefs.h"
#include <rtems/sysinit.h>
#include <drvmgr/drvmgr.h>
#ifdef CONFIG_OFW
#include <ofw/ofw.h>
#include "ofw/ofw_extension.h"
#endif
#include "base/bitops.h"
#include "base/compiler.h"
#include "base/byteorder.h"
#include "base/sections.h"
#include "base/minmax.h"

#ifdef __cplusplus
extern "C"{
#endif

RTEMS_STATIC_ASSERT((sizeof(struct drvmgr_dev) % 4) == 0, "");

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
	DRVMGR_BUS_TYPE_PLATFORM	= 20,
#define DRIVER_PLATFORM_ID		DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_PLATFORM)
	DRVMGR_BUS_TYPE_DMA,
#define DRIVER_DMA_ID		    DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_DMA)
	DRVMGR_BUS_TYPE_GPIO,
#define DRIVER_GPIO_ID		    DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_GPIO)
	DRVMGR_BUS_TYPE_I2C,
#define DRIVER_I2C_ID		    DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_I2C)
	DRVMGR_BUS_TYPE_SPI,
#define DRIVER_SPI_ID		    DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_SPI)
	DRVMGR_BUS_TYPE_PINCTRL,
#define DRIVER_PINCTRL_ID		DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_PINCTRL)
	DRVMGR_BUS_TYPE_MMCHOST,
#define DRIVER_MMCHOST_ID			DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_MMCHOST)
	DRVMGR_BUS_TYPE_MMC,
#define DRIVER_MMC_ID			DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_MMC)
	DRVMGR_BUS_TYPE_SDHCI,
#define DRIVER_SDHCI_ID			DRIVER_ROOTBUS_ID(DRVMGR_BUS_TYPE_SDHCI)
};
	
struct dev_id {
	const char *compatible;
	void *data;
};

struct dev_driver {
	struct drvmgr_drv drv;
	const struct dev_id *const ids;
};

/* Running at the begin of Init thread */
struct drv_posthook {
	struct drv_posthook *next;
	void (*run)(void *);
	void *arg;
};

static inline void *device_get_parent_priv(struct drvmgr_dev *dev) {
	return dev->parent->dev->priv;
}

static inline void *device_get_private(struct drvmgr_dev *dev) {
	return (void *)(dev + 1);
}

static inline struct drvmgr_dev *device_get_parent(struct drvmgr_dev *dev) {
	return dev->parent->dev;
}

static inline const void *device_get_operations(struct drvmgr_dev *dev) {
	void **ops = (void **)device_get_private(dev);
	return *ops;
}

static inline void device_set_operations(struct drvmgr_dev *dev,
	const void *ops) {
	void **mp = (void **)device_get_private(dev);
	*mp = (void *)ops;
}

#define __platform_driver_init(drv) \
	RTEMS_STATIC_ASSERT(sizeof(drv) == sizeof(struct dev_driver), \
		"Device driver object type error!"); \
	static void platform_driver_##drv##_register(void) { \
		drvmgr_drv_register((struct drvmgr_drv *)&drv); \
	}	\
	RTEMS_SYSINIT_ITEM(platform_driver_##drv##_register, \
		RTEMS_SYSINIT_DRVMGR,  \
		RTEMS_SYSINIT_ORDER_MIDDLE \
	)


struct drvmgr_dev *device_add(struct drvmgr_dev *parent, 
    const struct drvmgr_bus_ops *bus_ops, 
    int bustype, const char *name, size_t devp_size, size_t priv_size, bool attach);
int device_delete(struct drvmgr_dev *parent, struct drvmgr_dev *dev);
int device_attach(struct drvmgr_dev *dev);

int driver_register_posthook(struct drv_posthook *hook);

#ifdef __cplusplus
}
#endif
#endif /* DRIVER_DEV_BASE_H_ */
