/*
 * CopyRight(c) 2022 wtcat
 */
/*
 * Resource define macros
 *
 * Example:
 *
 * TEMPLATE_RESOURCE(uart0, "st,uart", "uart0", DRVMGR_BUS_TYPE_PLATFORM,
 * 	TRN("REG0", 0, 0x100000000),
 * 	TRN("IRQ0", 0, 0X16)
 * );
 * 
 * TEMPLATE_RESOURCE(spi0, "st,spi", "uart0", DRVMGR_BUS_TYPE_PLATFORM,
 * 	TRN("REG0", 0, 0x102000000),
 * 	TRN("IRQ0", 0, 0X11)
 * );
 *
 * TEMPLATE_RESOURCES_REGISTER(platform_resources,
 * 	RN(uart0),
 * 	RN(spi0)
 * );
 *
 */

#ifndef DRIVER_PLATFORM_BUS_H_
#define DRIVER_PLATFORM_BUS_H_

#include <rtems/sysinit.h>
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
#define IRQF_SOFT(svr, irq)     (IRQF_THREADED(svr) | IRQF_INDEX(irq))
#define IRQF_SOFT_ABS(svr, irq) (IRQ_SOFT(svr, irq) | IRQF_ABS)
#define IRQF_HARD(irq)           IRQF_INDEX(irq)
#define IRQF_HARD_ABS(irq)      (IRQF_INDEX(irq) | IRQF_ABS)


/* Bus resource class */
#define DEVICE_RESOURCE_BASE \
	const char *compatible; \
	const char *name; \
	int parent_bus; \
	const char *parent_name;

#define RESOURCE_BASE_DECLARE \
	DEVICE_RESOURCE_BASE; \
	const struct drvmgr_key keys[];


#define RN(node) (const struct bus_resource *)(&node)
#define TRN(rname, type, value) {rname, type, {.ptr = (void *)value}}
#define __TEMPLATE_RESOURCE(_name, _compatible, _parent, _parent_name, ...) \
	static const char __dev_filename_##_name[] = { "/dev/"#_name }; \
	static const struct resoruce_##_name {\
		RESOURCE_BASE_DECLARE } \
		_name = { \
			.compatible = _compatible, \
			.name = &__dev_filename_##_name[5], \
			.parent_bus = _parent, \
			.parent_name = _parent_name, \
			.keys = { __VA_ARGS__, DRVMGR_KEY_EMPTY } \
		}

#define TEMPLATE_RESOURCE(_name, _compatible, _parent, _parent_name, ...) \
	static const char __dev_filename_parent_##_name[] = { \
		"/dev/"#_parent_name }; \
	__TEMPLATE_RESOURCE(_name, _compatible, _parent, \
		&__dev_filename_parent_##_name[5], __VA_ARGS__)

#define PLATFORM_RESOURCE(_name, _compatible, ...) \
	__TEMPLATE_RESOURCE(_name, _compatible, DRVMGR_BUS_TYPE_PLATFORM, \
		"root bus", __VA_ARGS__)

#define TEMPLATE_RESOURCES_REGISTER(_name, ...) \
	static void platform_resource_##_name##_register(void) { \
		static const struct bus_resource *const _name[] = {__VA_ARGS__}; \
		platform_res_register((const struct bus_resource *const*)_name); \
	}	\
	RTEMS_SYSINIT_ITEM(platform_resource_##_name##_register, \
		RTEMS_SYSINIT_BSP_START,  \
		RTEMS_SYSINIT_ORDER_MIDDLE \
	)

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

struct bus_resource {
	RESOURCE_BASE_DECLARE
};

struct dev_private {
	const void *devops;  /* Device operations */
	const struct bus_resource *res;
	unsigned int base;
	unsigned short nirq;
	unsigned short irqs[];
};

const struct bus_resource *const * platform_res_get(void);
int platform_res_count_get(struct drvmgr_key *keys, 
	const char *name, size_t len);
void *platform_resource_get(struct drvmgr_key *keys, enum drvmgr_kt key_type, 
	const char *fmt, ...);
int platform_reg_resource_get(struct drvmgr_key *keys, int index,
	unsigned int *reg);
int platform_irq_resource_get(struct drvmgr_key *keys, int index,
	unsigned int *oirq);
int platform_res_register(const struct bus_resource *const *r);
int platform_dev_register(struct drvmgr_bus *parent,
	const struct bus_resource *r);
int platform_bus_match(struct drvmgr_drv *drv, struct drvmgr_dev *dev, 
	int bustype);
int platform_dev_populate_on_bus(struct drvmgr_bus *bus,
	const struct bus_resource *const *r);
int platform_bus_device_register(struct drvmgr_dev *dev,
	struct drvmgr_bus_ops *bus_ops, int bustype);
int platform_irq_map(struct drvmgr_dev *dev, int index);
const struct dev_id *device_match(struct drvmgr_dev *dev, 
	const struct dev_id *id_table);

static inline const char *platform_dev_filename(struct drvmgr_dev *dev) {
	return dev->name - 5;
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

static inline int platform_irq_count_get(struct drvmgr_key *keys) {
	return platform_res_count_get(keys, "IRQ", 3);	
}

static inline int platform_reg_count_get(struct drvmgr_key *keys) {
	return platform_res_count_get(keys, "REG", 3);	
}

static inline int platform_bus_populate(struct drvmgr_bus *bus) {
	return platform_dev_populate_on_bus(bus, platform_res_get());
}

static inline int platform_reg_get(struct drvmgr_dev *dev,
	int index, unsigned int *reg) {
	struct dev_private *priv = device_get_private(dev);
	return platform_reg_resource_get((struct drvmgr_key *)priv->res->keys, 
		index, reg);
}
	
static inline int platform_irq_get(struct drvmgr_dev *dev,
	int index, unsigned int *oirq) {
	struct dev_private *priv = device_get_private(dev);
	return platform_irq_resource_get((struct drvmgr_key *)priv->res->keys, 
		index, oirq);
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

#define platform_devres_init(res) \
	static void platform_resource_##res##_register(void) { \
		platform_res_register((const struct bus_resource *)&res); \
	}	\
	RTEMS_SYSINIT_ITEM(platform_resource_##res##_register, \
		RTEMS_SYSINIT_BSP_START,  \
		RTEMS_SYSINIT_ORDER_MIDDLE \
	)

#define PLATFORM_RESOURCE_REGISTER(name) \
	static const struct bus_resource __res_##name; \
	platform_devres_init(__res_##name); \
	static const struct bus_resource __res_##name

#define PLATFORM_DRIVER(name) \
	static struct dev_driver __drv_##name; \
	platform_driver_init(__drv_##name); \
	static struct dev_driver __drv_##name

#ifdef __cplusplus
}
#endif
#endif /* DRIVER_PLATFORM_BUS_H_ */

