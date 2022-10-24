/*
 * Copyright 2022 wtcat
 */
#ifndef BASE_SECTIONS_H_
#define BASE_SECTIONS_H_

#ifdef __cplusplus
extern "C"{
#endif

#define __fastcode    __attribute__((section(".bsp_fast_text")))
#define __fastdata    __attribute__((section(".bsp_fast_data")))
#define __nocache     __attribute__((section(".bsp_nocache")))
#define __noinit      __attribute__((section(".noinit")))
#define __rtemsstack  __attribute__((section(".rtemsstack")))

#define __isr   __fastcode
#define __cache __fastdata

#ifdef __cplusplus
}
#endif
#endif /* BASE_SECTIONS_H_ */
