/*
 * Copyright 2022 wtcat
 */
#ifndef BASE_SECTIONS_H_
#define BASE_SECTIONS_H_

#include <bsp/linker-symbols.h>
#ifdef __cplusplus
extern "C"{
#endif

#define __fastcode    BSP_FAST_TEXT_SECTION
#define __fastdata    BSP_FAST_DATA_SECTION
#define __nocache     BSP_NOCACHE_SECTION
#define __noinit      RTEMS_SECTION(".noinit")
#define __rtemsstack  RTEMS_SECTION(".rtemsstack")

#define __isr   __fastcode
#define __cache __fastdata

#ifdef __cplusplus
}
#endif
#endif /* BASE_SECTIONS_H_ */
