#ifndef LIBBSP_ARM_AM43XX_IRQ_H
#define LIBBSP_ARM_AM43XX_IRQ_H

#ifndef ASM

#include <rtems/irq.h>
#include <rtems/irq-extension.h>

#include <bsp/arm-a9mpcore-irq.h>
#include <dev/irq/arm-gic-irq.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define BSP_INTERRUPT_VECTOR_MIN 0
#define BSP_INTERRUPT_VECTOR_MAX 211
#define BSP_INTERRUPT_VECTOR_COUNT 211

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* ASM */

#endif /* LIBBSP_ARM_AM43XX_IRQ_H */
