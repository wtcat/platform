#include <rtems.h>
#include <rtems/bspIo.h>

#include "bsp/platform_bus.h"

PLATFORM_DRIVER(null_driver) = {
    .drv = {
        .obj_type = DRVMGR_OBJ_DRV,
        .drv_id   = DRIVER_PLATFORM_ID,
        .name     = "serial",
        .bus_type = DRVMGR_BUS_TYPE_ROOT,
        .ops      = NULL
    },
    .ids = NULL
};


static rtems_task Init(
  rtems_task_argument ignored
)
{
  printk( "Hello World\n" );
}


/* NOTICE: the clock driver is explicitly disabled */
#define CONFIGURE_APPLICATION_DOES_NOT_NEED_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_SIMPLE_CONSOLE_DRIVER

#define CONFIGURE_MAXIMUM_TASKS            1

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
