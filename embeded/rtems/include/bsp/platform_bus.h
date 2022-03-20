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
	int parent_busid; \
	const struct drvmgr_key keys[];


#define RN(node) (const struct bus_resource *)(&node)
#define TRN(rname, type, value) {rname, type, {value}}
#define TEMPLATE_RESOURCE(_name, _compatible, _parent, _parent_busid, ...) \
	static const struct resoruce_##_name {\
		RESOURCE_BASE_DECLARE } \
		_name = { \
			.compatible = _compatible, \
			.name = "/dev/"#_name, \
			.parent_bus = _parent, \
			.parent_busid = _parent_busid, \
			.keys = { __VA_ARGS__, DRVMGR_KEY_EMPTY } \
		}

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

static inline struct dev_private *device_get_private(struct drvmgr_dev *dev) {
	return (struct dev_private *)(dev + 1);
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

#define PLATFORM_RESOURCE(name) \
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
#endif /* DRVMGR_PLATFORM_BUS_H_ */

