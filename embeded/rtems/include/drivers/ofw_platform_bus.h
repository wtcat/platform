/*
 * CopyRight(c) 2022 wtcat
 */
#ifndef DRIVER_OFW_PLATFORM_BUS_H_
#define DRIVER_OFW_PLATFORM_BUS_H_

#include "drivers/devbase.h"
#include "rtems/chain.h"
#include <ofw/ofw.h>

#ifdef __cplusplus
extern "C"{
#endif

typedef rtems_ofw_memory_area ofw_reg_t;

struct dev_private {
	rtems_chain_node node;
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

#define OFW_PLATFORM_DRIVER(name) \
	static struct dev_driver __ofw_drv_##name; \
	__platform_driver_init(__ofw_drv_##name); \
	static struct dev_driver __ofw_drv_##name

#ifdef __cplusplus
}
#endif
#endif /* DRIVER_OFW_PLATFORM_BUS_H_ */
