/*
 * CopyRight(c) 2022 wtcat
 */
#include <errno.h>
#include <string.h>

#include <rtems/sysinit.h>
#include <rtems/bspIo.h>
#include <ofw/ofw.h>

#include "base/minmax.h"
#include "drivers/ofw_platform_bus.h"

#define SIMPLE_BUS "simple-bus"
#define SIMPLE_BUS_SIZE (sizeof(SIMPLE_BUS) - 1)

static int ofw_root_bus_filter(phandle_t np, char *devname, size_t max) {
	int len;
	if (rtems_ofw_is_node_compatible(np, SIMPLE_BUS)) {
		len = min(max, SIMPLE_BUS_SIZE);
		memcpy(devname, SIMPLE_BUS, len);
		return len;
	}
	if (!rtems_ofw_has_prop(np, "compatible"))
		return -EINVAL;
	len = rtems_ofw_get_prop(np, "rtems,path", devname, max);
	if (len <= 0)
		return -EINVAL;
	return len;
}

static int ofw_root_bus_unite(struct drvmgr_drv *drv, struct drvmgr_dev *dev) {
	return ofw_platform_bus_match(drv, dev, DRVMGR_BUS_TYPE_ROOT);
}

static int ofw_root_bus_populate_device(struct drvmgr_bus *bus) {
	return __ofw_bus_populate_device(bus, rtems_ofw_peer(0), 
	ofw_root_bus_filter);
}

static struct drvmgr_bus_ops root_bus_ops = {
	.init = {
		ofw_root_bus_populate_device,
	},
	.unite = ofw_root_bus_unite
};

static int ofw_root_bus_init(struct drvmgr_dev *dev) {
	return ofw_platform_bus_device_register(dev, &root_bus_ops, 
		DRVMGR_BUS_TYPE_ROOT);
}

static struct drvmgr_drv_ops root_driver_ops = {
	.init = {
		ofw_root_bus_init,
	}
};
		
static struct drvmgr_drv root_bus_driver = {
	.obj_type = DRVMGR_OBJ_DRV,
	.drv_id   = DRIVER_ROOT_ID,
	.name     = "root-bus",
	.bus_type = DRVMGR_BUS_TYPE_ROOT,
	.ops      = &root_driver_ops
};

static void ofw_root_bus_driver_register(void) {
	/* Register root device driver */
	drvmgr_root_drv_register(&root_bus_driver);
}

RTEMS_SYSINIT_ITEM(ofw_root_bus_driver_register,
	RTEMS_SYSINIT_BSP_PRE_DRIVERS,
	RTEMS_SYSINIT_ORDER_MIDDLE
);
