/*
 * Copyright 2022 wtcat
 */
#include <rtems/console.h>
#include <rtems/bspIo.h>

#include <bsp.h>
#include <bsp/start.h>

#ifdef RTEMS_DRVMGR_STARTUP
#include <drvmgr/drvmgr.h>
drvmgr_drv_reg_func drvmgr_drivers[] RTEMS_WEAK = { NULL };
#endif /* RTEMS_DRVMGR_STARTUP */

/*
 * Make weak and let the user override.
 */
BSP_START_TEXT_SECTION void setup_mmu_and_cache(void) RTEMS_WEAK;
BSP_START_TEXT_SECTION void setup_mmu_and_cache(void) {}

BSP_START_TEXT_SECTION void bsp_start_hook_0(void) RTEMS_WEAK;
BSP_START_TEXT_SECTION void bsp_start_hook_0(void) {}

BSP_START_TEXT_SECTION void bsp_start_hook_1(void) RTEMS_WEAK;
BSP_START_TEXT_SECTION void bsp_start_hook_1(void) {}

void bsp_reset(void) RTEMS_WEAK;
void bsp_reset(void) {}

void bsp_start(void) RTEMS_WEAK;
void bsp_start(void) {}

/* For rtems-libbsd */
uint32_t bsp_fdt_map_intr(const uint32_t *intr, 
	size_t icells) RTEMS_WEAK;
uint32_t bsp_fdt_map_intr(const uint32_t *intr, 
	size_t icells) {
    (void) icells;
    return intr[1] + 0;
}

rtems_status_code console_initialize(rtems_device_major_number major,
    rtems_device_minor_number minor, void *arg) {
    (void) major;
    (void) minor;
    (void) arg;
    return RTEMS_SUCCESSFUL;
}

static void _bsp_empty_putchar(char c) {
    (void)c;
}

static int _bsp_empty_getchar(void) {
    return -1;
}
BSP_output_char_function_type BSP_output_char = _bsp_empty_putchar;
BSP_polling_getchar_function_type BSP_poll_char = _bsp_empty_getchar;