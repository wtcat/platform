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

#ifndef DRVMGR_PLATFORM_BUS_H_
#define DRVMGR_PLATFORM_BUS_H_

#include <rtems/sysinit.h>
#include <drvmgr/drvmgr.h>

#ifdef __cplusplus
extern "C"{
#endif

#define DRVMGR_WARN "DRVMGR_WARNING: " 

/* Bus resource class */
#define RESOURCE_BASE_DECLARE \
	const char *compatible; \
	const char *name; \
	int parent_bus; \
	const struct drvmgr_key keys[];


#define RN(node) (const struct bus_resource *)(&node)
#define TRN(rname, type, value) {rname, type, {value}}
#define TEMPLATE_RESOURCE(_name, _compatible, _devname, _parent, ...) \
	static const struct resoruce_##_name {\
		RESOURCE_BASE_DECLARE } \
		_name = { \
			.compatible = _compatible, \
			.name = _devname, \
			.parent_bus = _parent, \
			.keys = { __VA_ARGS__, DRVMGR_KEY_EMPTY } \
		}

#define TEMPLATE_RESOURCES_REGISTER(_name, ...) \
	static void platform_resource_##_name##_register(void) { \
		static const void *_name[] = { \
			__VA_ARGS__, {NULL} \
		}; \
		platform_res_register((const struct bus_resource *)_name); \
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
	DRVMGR_BUS_TYPE_GPIO,
	DRVMGR_BUS_TYPE_I2C,
	DRVMGR_BUS_TYPE_SPI,
};
	
struct dev_id {
	const char *compatible;
	void *data;
};

struct dev_driver {
	struct drvmgr_drv drv;
	const struct dev_id *ids;
};

struct bus_resource {
	RESOURCE_BASE_DECLARE
};

struct dev_private {
	const struct bus_resource *res;
	unsigned int base;
	unsigned short nirq;
	unsigned short irqs[];
};

const struct bus_resource *platform_res_get(void);
int platform_res_count_get(struct drvmgr_key *keys, 
	const char *name, size_t len);
int platform_reg_resource_get(struct drvmgr_key *keys, int index,
	unsigned int *reg);
int platform_irq_resource_get(struct drvmgr_key *keys, int index,
	unsigned int *oirq);
int platform_res_register(const struct bus_resource *r);
int platform_dev_register(struct drvmgr_bus *parent,
	const struct bus_resource *r);
int platform_bus_match(struct drvmgr_drv *drv, struct drvmgr_dev *dev, 
	int bustype);
int platform_dev_populate_on_bus(struct drvmgr_bus *bus,
	const struct bus_resource *r);
int platform_bus_device_register(struct drvmgr_dev *dev,
	struct drvmgr_bus_ops *bus_ops, int bustype);
int platform_irq_map(struct drvmgr_dev *dev, int index);


static inline int platform_irq_count_get(struct drvmgr_key *keys) {
	return platform_res_count_get(keys, "IRQ", 3);	
}

static inline int platform_reg_count_get(struct drvmgr_key *keys) {
	return platform_res_count_get(keys, "REG", 3);	
}

static inline int platform_bus_populate(struct drvmgr_bus *bus) {
	return platform_dev_populate_on_bus(bus, platform_res_get());
}

static inline int device_reg_get(struct dev_private *priv, int index, 
	unsigned int *reg) {
	return platform_reg_resource_get(priv->res->keys, index, reg);
}

static inline int device_irq_get(struct dev_private *priv, int index, 
	unsigned int *oirq) {
	return platform_irq_resource_get(priv->res->keys, index, oirq);
}


#define platform_driver_register(drv) \
	RTEMS_STATIC_ASSERT(sizeof(drv) == sizeof(struct dev_driver), \
		"Device driver object type error!"); \
	static void platform_driver_##drv##_register(void) { \
		drvmgr_drv_register(&drv); \
	}	\
	RTEMS_SYSINIT_ITEM(platform_driver_##drv##_register, \
		RTEMS_SYSINIT_DRVMGR,  \
		RTEMS_SYSINIT_ORDER_MIDDLE \
	)

#define platform_dev_resource_register(res) \
	static void platform_resource_##res##_register(void) { \
		platform_res_register((const struct bus_resource *)res); \
	}	\
	RTEMS_SYSINIT_ITEM(platform_resource_##res##_register, \
		RTEMS_SYSINIT_BSP_START,  \
		RTEMS_SYSINIT_ORDER_MIDDLE \
	)

#ifdef __cplusplus
}
#endif
#endif /* DRVMGR_PLATFORM_BUS_H_ */

