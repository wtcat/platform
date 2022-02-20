/*
 * CopyRight(c) 2022 wtcat
 */
/*
* Example: Resource define
*
* static const struct drvmgr_key st_uart0[] = {
*	 {"REG0", DRVMGR_KT_INT, 0x40000000},
*	 {"IRQ0", DRVMGR_KT_INT, 32},
*	 {NULL, 0, 0}
* };
* static const struct bus_resource r[] = {
*	 {"st,uart", "uart0", st_uart0},
*	 {NULL}
* };
*
*/

#ifndef DRVMGR_PLATFORM_BUS_H_
#define DRVMGR_PLATFORM_BUS_H_

#include <drvmgr/drvmgr.h>

#ifdef __cplusplus
extern "C"{
#endif

#define DRVMGR_WARN "DRVMGR_WARNING: " 
	
	/* Platform Bus */
enum drvmgr_bus_type {
	DRVMGR_BUS_TYPE_PLATFORM	= 10,
	DRVMGR_BUS_TYPE_GPIO,
	DRVMGR_BUS_TYPE_I2C,
	DRVMGR_BUS_TYPE_SPI,
};
	
struct irq_res {
#define MAX_IRQRES_NR 8
	unsigned char nr;
	unsigned short irqs[MAX_IRQRES_NR];
};

struct dev_id {
	const char *compatible;
	void *data;
};

struct dev_driver {
	struct drvmgr_drv drv;
	const struct dev_id *ids;
};

struct bus_resource {
	const char *comatible;
	const char *name;
	const struct drvmgr_key *keys;
};

struct dev_private {
#define MAX_IRQRES_NR 8
	const char *compatible;
	unsigned int regbase;
	struct irq_res irqres;
};

int platform_res_register(const struct bus_resource *r);
int platform_dev_register(struct drvmgr_bus *parent,
	const struct bus_resource *r);

#ifdef __cplusplus
}
#endif
#endif /* DRVMGR_PLATFORM_BUS_H_ */

