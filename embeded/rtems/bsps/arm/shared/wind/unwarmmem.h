/***************************************************************************
 * ARM Stack Unwinder, Michael.McTernan.2001@cs.bris.ac.uk
 *
 * This program is PUBLIC DOMAIN.
 * This means that there is no copyright and anyone is able to take a copy
 * for free and use it as they wish, with or without modifications, and in
 * any context, commerically or otherwise. The only limitation is that I
 * don't guarantee that the software is fit for any purpose or accept any
 * liablity for it's use or misuse - this software is without warranty.
 ***************************************************************************
 * File Description: Interface to the memory tracking sub-system.
 **************************************************************************/
/*
 * Copyright 2022 wtcat
 */

#ifndef BSP_ARM_UNWARMMEM_H_
#define BSP_ARM_UNWARMMEM_H_

#include "unwarm.h"

#ifdef __cplusplus
extern "C"{
#endif

Boolean UnwMemHashRead  (MemData * const memData,
                         Int32           addr,
                         Int32   * const data,
                         Boolean * const tracked);

Boolean UnwMemHashWrite (MemData * const memData,
                         Int32           addr,
                         Int32           val,
                         Boolean         valValid);

void    UnwMemHashGC    (UnwState * const state);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ARM_UNWARMMEM_H_ */
