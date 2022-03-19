#include "bsp/platform_bus.h"

#define nIRQ(n) ((n) + 32)

/*
 * UARTn
 */
TEMPLATE_RESOURCE(ttyS0, "ti,am4372-uart", DRVMGR_BUS_TYPE_PLATFORM, 0,
  	TRN("REG0", DRVMGR_KT_INT, 0x44e09000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(72)),
    TRN("stdout", DRVMGR_KT_INT, 921600) /* Console : baudrate*/
);
TEMPLATE_RESOURCE(ttyS1, "ti,am4372-uart", DRVMGR_BUS_TYPE_PLATFORM, 0,
  	TRN("REG0", DRVMGR_KT_INT, 0x48022000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(73))
);
TEMPLATE_RESOURCE(ttyS2, "ti,am4372-uart", DRVMGR_BUS_TYPE_PLATFORM, 0,
  	TRN("REG0", DRVMGR_KT_INT, 0x48024000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(74))
);
TEMPLATE_RESOURCE(ttyS3, "ti,am4372-uart", DRVMGR_BUS_TYPE_PLATFORM, 0,
  	TRN("REG0", DRVMGR_KT_INT, 0x481a6000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(44))
);
TEMPLATE_RESOURCE(ttyS4, "ti,am4372-uart", DRVMGR_BUS_TYPE_PLATFORM, 0,
  	TRN("REG0", DRVMGR_KT_INT, 0x481aa000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(46))
);

/*
 * GPIO
 */
TEMPLATE_RESOURCE(gpio0, "ti,am4372-gpio", DRVMGR_BUS_TYPE_PLATFORM, 0,
  	TRN("REG0", DRVMGR_KT_INT, 0x44e07000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(96))
);
TEMPLATE_RESOURCE(gpio1, "ti,am4372-gpio", DRVMGR_BUS_TYPE_PLATFORM, 0,
  	TRN("REG0", DRVMGR_KT_INT, 0x4804c000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(98))
);
TEMPLATE_RESOURCE(gpio2, "ti,am4372-gpio", DRVMGR_BUS_TYPE_PLATFORM, 0,
  	TRN("REG0", DRVMGR_KT_INT, 0x481ac000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(32))
);
TEMPLATE_RESOURCE(gpio3, "ti,am4372-gpio", DRVMGR_BUS_TYPE_PLATFORM, 0,
  	TRN("REG0", DRVMGR_KT_INT, 0x481ae000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(62))
);
TEMPLATE_RESOURCE(gpio4, "ti,am4372-gpio", DRVMGR_BUS_TYPE_PLATFORM, 0,
  	TRN("REG0", DRVMGR_KT_INT, 0x48320000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(106))
);
TEMPLATE_RESOURCE(gpio5, "ti,am4372-gpio", DRVMGR_BUS_TYPE_PLATFORM, 0,
  	TRN("REG0", DRVMGR_KT_INT, 0x48322000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(148))
);

TEMPLATE_RESOURCES_REGISTER(platform_resources,
    RN(ttyS0), RN(ttyS1), RN(ttyS2), RN(ttyS3), RN(ttyS4), 
	RN(gpio0), RN(gpio1), RN(gpio2), RN(gpio3), RN(gpio4), RN(gpio5),
	NULL
);
