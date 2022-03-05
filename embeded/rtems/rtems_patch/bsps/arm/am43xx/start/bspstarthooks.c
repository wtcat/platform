#define ARM_CP15_TEXT_SECTION BSP_START_TEXT_SECTION
#include <bsp.h>


/*
 * Make weak and let the user override.
 */
BSP_START_TEXT_SECTION void setup_mmu_and_cache(void) __attribute__ ((weak));
BSP_START_TEXT_SECTION void setup_mmu_and_cache(void) {}

BSP_START_TEXT_SECTION void bsp_start_hook_0(void) __attribute__ ((weak));
BSP_START_TEXT_SECTION void bsp_start_hook_0(void) {}

BSP_START_TEXT_SECTION void bsp_start_hook_1(void) __attribute__ ((weak));
BSP_START_TEXT_SECTION void bsp_start_hook_1(void) {}
