/*
 * CopyRight(c) 2022 wtcat
 */
#include <errno.h>

#include <rtems/sysinit.h>
#include <rtems/bspIo.h>
#include <ofw/ofw.h>

#include "drivers/ofw_platform_bus.h"

static int ofw_root_bus_unite(struct drvmgr_drv *drv, struct drvmgr_dev *dev) {
	return ofw_platform_bus_match(drv, dev, DRVMGR_BUS_TYPE_CPU);
}

static int ofw_root_bus_populate_device(struct drvmgr_bus *bus) {
	return ofw_bus_populate_device(bus, rtems_ofw_peer(0));
}

static struct drvmgr_bus_ops root_bus_ops = {
	.init = {
		ofw_root_bus_populate_device,
	},
	.unite = ofw_root_bus_unite
};

static int ofw_root_bus_init(struct drvmgr_dev *dev) {
	return ofw_platform_bus_device_register(dev, &root_bus_ops, 
		DRVMGR_BUS_TYPE_CPU);
}

static struct drvmgr_drv_ops root_driver_ops = {
	.init = {
		ofw_root_bus_init,
	}
};
		
static struct drvmgr_drv platform_bus_driver = {
	.obj_type = DRVMGR_OBJ_DRV,
	.drv_id   = DRIVER_CPU_ID,
	.name     = "root-bus",
	.bus_type = DRVMGR_BUS_TYPE_CPU,
	.ops      = &root_driver_ops
};

static void ofw_platform_bus_driver_register(void) {
	/* Register root device driver */
	drvmgr_root_drv_register(&platform_bus_driver);
}

RTEMS_SYSINIT_ITEM(ofw_platform_bus_driver_register,
	RTEMS_SYSINIT_BSP_PRE_DRIVERS,
	RTEMS_SYSINIT_ORDER_MIDDLE
);
