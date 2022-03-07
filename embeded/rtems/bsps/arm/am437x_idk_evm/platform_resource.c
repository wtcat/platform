#include "bsp/platform_bus.h"

#define nIRQ(n) ((n) + 32)

/*
 * UARTn
 */
TEMPLATE_RESOURCE(uart0, "ti,am4372-uart", "uart1", DRVMGR_BUS_TYPE_PLATFORM,
  	TRN("REG0", DRVMGR_KT_INT, 0x44e09000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(72)),
    TRN("stdout", DRVMGR_KT_INT, 0) /* Console */
);
TEMPLATE_RESOURCE(uart1, "ti,am4372-uart", "uart2", DRVMGR_BUS_TYPE_PLATFORM,
  	TRN("REG0", DRVMGR_KT_INT, 0x48022000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(73))
);
TEMPLATE_RESOURCE(uart2, "ti,am4372-uart", "uart3", DRVMGR_BUS_TYPE_PLATFORM,
  	TRN("REG0", DRVMGR_KT_INT, 0x48024000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(74))
);
TEMPLATE_RESOURCE(uart3, "ti,am4372-uart", "uart4", DRVMGR_BUS_TYPE_PLATFORM,
  	TRN("REG0", DRVMGR_KT_INT, 0x481a6000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(44))
);
TEMPLATE_RESOURCE(uart4, "ti,am4372-uart", "uart5", DRVMGR_BUS_TYPE_PLATFORM,
  	TRN("REG0", DRVMGR_KT_INT, 0x481aa000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(46))
);

TEMPLATE_RESOURCES_REGISTER(platform_resources,
    RN(uart0), RN(uart1), RN(uart2), RN(uart3), RN(uart4), 
	NULL
);