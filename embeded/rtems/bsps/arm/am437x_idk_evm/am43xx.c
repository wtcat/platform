#include <rtems/bspIo.h>
#include <rtems/sysinit.h>

#include <bsp.h>
#include <bsp/fdt.h>
#include <bsp/bootcard.h>
#include <bsp/irq-generic.h>
#include <bsp/linker-symbols.h>
#include <bsp/arm-cp15-start.h>
#include <bsp/arm-a9mpcore-start.h>


/*
 * User mmu configure infomation
 */
BSP_START_DATA_SECTION static const
arm_cp15_start_section_config _am43xx_mmu_configs[] = {
  ARMV7_CP15_START_DEFAULT_SECTIONS,
  {
    .begin = 0x40000000U,
    .end = 0x4FFFFFFF,
    .flags = ARMV7_MMU_DEVICE
  }
};

/*
 * Overwrite bsp default mmu configuration
 */
BSP_START_TEXT_SECTION void setup_mmu_and_cache(void)
{
    uint32_t ctrl = arm_cp15_start_setup_mmu_and_cache(
      ARM_CP15_CTRL_A | ARM_CP15_CTRL_I | ARM_CP15_CTRL_C | ARM_CP15_CTRL_M,
      ARM_CP15_CTRL_AFE | ARM_CP15_CTRL_Z
    );

    arm_cp15_start_setup_translation_table(
      (uint32_t *) bsp_translation_table_base,
      ARM_MMU_DEFAULT_CLIENT_DOMAIN,
      &_am43xx_mmu_configs[0],
      RTEMS_ARRAY_SIZE(_am43xx_mmu_configs)
    );

    /* Enable MMU and cache */
    ctrl |= ARM_CP15_CTRL_I | ARM_CP15_CTRL_C | ARM_CP15_CTRL_M;
    arm_cp15_set_control(ctrl);
}

BSP_START_TEXT_SECTION void bsp_start_hook_0(void)
{
    arm_a9mpcore_start_hook_0();
}

BSP_START_TEXT_SECTION void bsp_start_hook_1(void)
{ 
    arm_a9mpcore_start_hook_1();
    bsp_start_copy_sections();
    setup_mmu_and_cache();
    rtems_cache_enable_data();
    bsp_start_clear_bss();
}

void bsp_start(void)
{
    bsp_interrupt_initialize();
    rtems_cache_coherent_add_area(bsp_section_nocacheheap_begin,
        (uintptr_t)bsp_section_nocacheheap_size);
}

uint32_t bsp_fdt_map_intr(const uint32_t *intr, 
	size_t icells)
{
    return intr[1] + 32;
}

void bsp_reset(void)
{
    
}

/* LIBBSD PHY DRIVER */
#if defined(__rtems_libbsd__)
#include <rtems/bsd/bsd.h>
SYSINIT_DRIVER_REFERENCE(micphy, miibus);
//SYSINIT_DRIVER_REFERENCE(ukphy, miibus);
#endif