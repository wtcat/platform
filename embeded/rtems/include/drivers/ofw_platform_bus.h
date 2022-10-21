/*
 * CopyRight(c) 2022 wtcat
 */
#ifndef DRIVER_OFW_PLATFORM_BUS_H_
#define DRIVER_OFW_PLATFORM_BUS_H_

#include <rtems/chain.h>
#include <ofw/ofw.h>

#include "drivers/devbase.h"

#ifdef __cplusplus
extern "C"{
#endif

typedef rtems_ofw_memory_area ofw_reg_t;

struct dev_private {
	const void *devops;  /* Device operations must be at first */
    phandle_t np; /* Device tree node */
	rtems_chain_node node;
};

int __ofw_bus_populate_device(struct drvmgr_bus *bus, phandle_t parent, 
	int (*filter)(phandle_t, char *devname, size_t max));
	
struct drvmgr_dev *ofw_device_get_by_path(const char *path);
struct drvmgr_dev *ofw_device_get_by_phandle(phandle_t np);

phandle_t ofw_platform_bus_get_node(struct drvmgr_dev *dev);
int ofw_bus_populate_device(struct drvmgr_bus *bus, phandle_t parent);
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

#define ofw_foreach_child_node(parent, child) \
    for (child = rtems_ofw_child(parent); \
        child != 0; \
        child = rtems_ofw_peer(child))
		
#define OFW_PLATFORM_DRIVER(name) \
	static struct dev_driver __ofw_drv_##name; \
	__platform_driver_init(__ofw_drv_##name); \
	static struct dev_driver __ofw_drv_##name

#ifdef __cplusplus
}
#endif
#endif /* DRIVER_OFW_PLATFORM_BUS_H_ */
