#ifndef LIBBSP_ARM_AM43XX_BSP_H
#define LIBBSP_ARM_AM43XX_BSP_H

#include <bspopts.h>

#define BSP_FEATURE_IRQ_EXTENSION

#if BSP_START_COPY_FDT_FROM_U_BOOT
#define BSP_FDT_IS_SUPPORTED
#endif

#ifndef ASM
#include <rtems.h>

#include <bsp/default-initial-extension.h>
#include <bsp/start.h>
#include <ofw/ofw.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define BSP_ARM_A9MPCORE_SCU_BASE 0x48240000

#define BSP_ARM_A9MPCORE_GT_BASE  0x48240200
#define BSP_ARM_A9MPCORE_PT_BASE  0xf8f00600
#define BSP_ARM_GIC_DIST_BASE     0x48241000
#define BSP_ARM_GIC_CPUIF_BASE    0x48240100
#define BSP_ARM_L2C_310_BASE      0x48242000
#define BSP_ARM_L2C_310_ID        0x410000c9

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ASM */

/** @} */

#endif /* LIBBSP_ARM_AM43XX_BSP_H */
