#include <rtems.h>
#include <rtems/bspIo.h>
#include "bsp/asm/platform.h"

static rtems_task Init(
  rtems_task_argument ignored
)
{
  printk( "Hello World\n" );
}


#define CONFIGURE_INIT
#include <rtems/confdefs.h>
