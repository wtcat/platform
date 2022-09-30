#include <rtems/console.h>
#include <rtems/bspIo.h>

#include <bsp.h>
#include <bsp/fdt.h>
#include <bsp/bootcard.h>
#include <bsp/irq-generic.h>
#include <bsp/linker-symbols.h>

#ifdef RTEMS_DRVMGR_STARTUP
#include <drvmgr/drvmgr.h>

/* All drivers included by BSP, this is overridden by the user by including
 * the drvmgr_confdefs.h. By default the Timer and UART driver are included.
 */
drvmgr_drv_reg_func drvmgr_drivers[] __attribute__((weak)) =
{
  NULL /* End array with NULL */
};

#endif

struct NS16550 {
#define UART_REG(x) uint32_t x

	UART_REG(rbr);		/* 0 */
	UART_REG(ier);		/* 1 */
	UART_REG(fcr);		/* 2 */
	UART_REG(lcr);		/* 3 */
	UART_REG(mcr);		/* 4 */
	UART_REG(lsr);		/* 5 */
	UART_REG(msr);		/* 6 */
	UART_REG(spr);		/* 7 */
	UART_REG(mdr1);		/* 8 */
	UART_REG(reg9);		/* 9 */
	UART_REG(regA);		/* A */
	UART_REG(regB);		/* B */
	UART_REG(regC);		/* C */
	UART_REG(regD);		/* D */
	UART_REG(regE);		/* E */
	UART_REG(uasr);		/* F */
	UART_REG(scr);		/* 10*/
	UART_REG(ssr);		/* 11*/

#define thr rbr
#define iir fcr
#define dll rbr
#define dlm ier
};

static void debug_uart_putc(char ch)
{
	volatile struct NS16550 *com_port = 
        (volatile struct NS16550 *)0x44e09000;
	while (!(com_port->lsr & 0x20));
	com_port->thr = ch;
}

rtems_status_code __attribute__((weak)) console_initialize(
    rtems_device_major_number major,
    rtems_device_minor_number minor,
    void *arg) {
    return RTEMS_SUCCESSFUL;
}

void bsp_start(void) __attribute__((weak));
void bsp_start(void) {}

uint32_t __attribute__((weak)) bsp_fdt_map_intr(const uint32_t *intr, 
	size_t icells)
{
    return intr[1] + BSP_MAGIC_IRQ_OFFSET;
}


BSP_output_char_function_type BSP_output_char = debug_uart_putc;
BSP_polling_getchar_function_type BSP_poll_char = NULL;
