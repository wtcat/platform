#include "bsp/platform_bus.h"
#include "bsp/board/pinctrl.h"

/* Control Module */
#define CTRL_MODULE 		0x44E10000

/* PRCM Base Address */
#define PRCM_BASE			0x44DF0000
#define	CM_WKUP				0x44DF2800
#define	CM_PER				0x44DF8800
#define CM_DPLL				0x44DF4200
#define CM_RTC				0x44DF8500
#define PRM_RSTCTRL			(PRCM_BASE + 0x4000)
#define PRM_RSTST			(PRM_RSTCTRL + 4)

#define PER_REG(ofs)  (CM_PER + (ofs)) 
#define WKUP_REG(ofs) (CM_WKUP + (ofs)) 
#define nIRQ(n) ((n) + 32)

/*
 * UARTn
 */
PLATFORM_RESOURCE(ttyS0, "ti,am4372-uart",
  	TRN("REG0", DRVMGR_KT_INT, 0x44e09000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(72)),
    TRN("stdout", DRVMGR_KT_INT, 921600) /* Console : baudrate*/
);
PLATFORM_RESOURCE(ttyS1, "ti,am4372-uart",
  	TRN("REG0", DRVMGR_KT_INT, 0x48022000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(73))
);
PLATFORM_RESOURCE(ttyS2, "ti,am4372-uart",
  	TRN("REG0", DRVMGR_KT_INT, 0x48024000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(74))
);
PLATFORM_RESOURCE(ttyS3, "ti,am4372-uart",
  	TRN("REG0", DRVMGR_KT_INT, 0x481a6000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(44))
);
PLATFORM_RESOURCE(ttyS4, "ti,am4372-uart",
  	TRN("REG0", DRVMGR_KT_INT, 0x481aa000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(46))
);

/*
 * GPIO
 */
PLATFORM_RESOURCE(gpio0, "ti,am4372-gpio",
  	TRN("REG0", DRVMGR_KT_INT, 0x44e07000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(96))
);
PLATFORM_RESOURCE(gpio1, "ti,am4372-gpio",
  	TRN("REG0", DRVMGR_KT_INT, 0x4804c000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(98))
);
PLATFORM_RESOURCE(gpio2, "ti,am4372-gpio",
  	TRN("REG0", DRVMGR_KT_INT, 0x481ac000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(32))
);
PLATFORM_RESOURCE(gpio3, "ti,am4372-gpio",
  	TRN("REG0", DRVMGR_KT_INT, 0x481ae000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(62))
);
PLATFORM_RESOURCE(gpio4, "ti,am4372-gpio",
  	TRN("REG0", DRVMGR_KT_INT, 0x48320000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(106))
);
PLATFORM_RESOURCE(gpio5, "ti,am4372-gpio",
  	TRN("REG0", DRVMGR_KT_INT, 0x48322000),
  	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(148))
);

/*
 * User button
 */
TEMPLATE_RESOURCE(gpio_keys, "gpio-keys", DRVMGR_BUS_TYPE_GPIO, gpio4,
  	TRN("REG0", DRVMGR_KT_INT, 2),
	TRN("POLARITY", DRVMGR_KT_INT, 0),
	TRN("CODE", DRVMGR_KT_INT, 0x10)
);

/*
 * DMTimer
 */
PLATFORM_RESOURCE(timer2, "ti,am4372-timer",
  	TRN("REG0", DRVMGR_KT_INT, 0x48040000),
	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(68)),
	TRN("fck", DRVMGR_KT_INT, PER_REG(0x530))
);
PLATFORM_RESOURCE(timer3, "ti,am4372-timer",
  	TRN("REG0", DRVMGR_KT_INT, 0x48042000),
	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(69)),
	TRN("fck", DRVMGR_KT_INT, PER_REG(0x538))
);
PLATFORM_RESOURCE(timer4, "ti,am4372-timer",
  	TRN("REG0", DRVMGR_KT_INT, 0x48044000),
	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(92)),
	TRN("fck", DRVMGR_KT_INT, PER_REG(0x540))
);

/*
 * I2C
 */
PLATFORM_RESOURCE(i2c0, "ti,am4372-i2c",
  	TRN("REG0", DRVMGR_KT_INT, 0x44e0b000),
	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(70)),
	TRN("CLKCTRL", DRVMGR_KT_INT, WKUP_REG(0x340))
);
PLATFORM_RESOURCE(i2c2, "ti,am4372-i2c",
  	TRN("REG0", DRVMGR_KT_INT, 0x4819c000),
	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(30)),
	TRN("CLKCTRL", DRVMGR_KT_INT, PER_REG(0x4b0))
);

/*
 * Digital Output
 */
TEMPLATE_RESOURCE(tpic2810, "ti,tpic2810", DRVMGR_BUS_TYPE_I2C, i2c2,
  	TRN("REG0", DRVMGR_KT_INT, 0x60)
);

/*
 * SPI
 */
PLATFORM_RESOURCE(spi1, "ti,am4372-mcspi",
  	TRN("REG0", DRVMGR_KT_INT, 0x481a0000),
	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(125)),
	TRN("CLKCTRL", DRVMGR_KT_INT, PER_REG(0x508)),
	TRN("mode", DRVMGR_KT_INT, 0),
	TRN("speed", DRVMGR_KT_INT, 48000000),
	TRN("word-length", DRVMGR_KT_INT, 16)
);


/*
 * EDMA
 */
static const int edma_memcpy_channels[] = {58, 59, -1};
static const uint32_t edma_tptcs[][3] = {
	{0x49800000, nIRQ(112), 7}, //edma_tptc0
	{0x49900000, nIRQ(113), 5}, //edma_tptc1
	{0x49a00000, nIRQ(114), 0}, //edma_tptc2
	{0}
};
PLATFORM_RESOURCE(edma, "ti,edma3-tpcc",
  	TRN("REG0", DRVMGR_KT_INT, 0x49000000),
	TRN("IRQ0", DRVMGR_KT_INT, nIRQ(12)), //edma3_ccint
	TRN("IRQ1", DRVMGR_KT_INT, nIRQ(13)), //edma3_mperr
	TRN("IRQ2", DRVMGR_KT_INT, nIRQ(14)), //edma3_ccerrint
	TRN("dma-requests", DRVMGR_KT_INT, 64),
	TRN("ti,edma-memcpy-channels", DRVMGR_KT_POINTER, edma_memcpy_channels),
	TRN("ti,tptcs", DRVMGR_KT_POINTER, edma_tptcs)
);

/*
 * Pin-ctrl
 */
static const struct pinctrl am437x_pads[] = {
	//AM4372_IOPAD(0x988, PIN_INPUT | SLEWCTRL_FAST | MUX_MODE0), /* i2c0_sda.i2c0_sda */
	//AM4372_IOPAD(0x98c, PIN_INPUT | SLEWCTRL_FAST | MUX_MODE0), /* i2c0_scl.i2c0_scl */	

	// AM4372_IOPAD(0x988, PIN_INPUT_PULLDOWN | MUX_MODE7),
	// AM4372_IOPAD(0x98c, PIN_INPUT_PULLDOWN | MUX_MODE7),
	AM4372_IOPAD(0x9e8, PIN_INPUT | SLEWCTRL_FAST | MUX_MODE3), /* cam1_data1.i2c2_scl */
	AM4372_IOPAD(0x9ec, PIN_INPUT | SLEWCTRL_FAST | MUX_MODE3), /* cam1_data0.i2c2_sda */

	PINCTRL_TERMINAL
};

PLATFORM_RESOURCE(pin_ctrl, "pinctrl-single,pins",
  	TRN("REG0", DRVMGR_KT_INT, CTRL_MODULE),
	TRN("pins", DRVMGR_KT_POINTER, am437x_pads)
);

TEMPLATE_RESOURCES_REGISTER(platform_resources,
	RN(pin_ctrl),
    RN(ttyS0), RN(ttyS1), RN(ttyS2), RN(ttyS3), RN(ttyS4), 
	RN(gpio0), RN(gpio1), RN(gpio2), RN(gpio3), RN(gpio4), RN(gpio5),
	RN(gpio_keys),
	RN(timer2), RN(timer3), RN(timer4),
	RN(i2c0), RN(i2c2),
	RN(tpic2810),
	RN(edma),
	NULL
);
