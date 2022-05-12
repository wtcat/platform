/***************************************************************************
 * ARM Stack Unwinder, Michael.McTernan.2001@cs.bris.ac.uk
 *
 * This program is PUBLIC DOMAIN.
 * This means that there is no copyright and anyone is able to take a copy
 * for free and use it as they wish, with or without modifications, and in
 * any context, commercially or otherwise. The only limitation is that I
 * don't guarantee that the software is fit for any purpose or accept any
 * liability for it's use or misuse - this software is without warranty.
 ***************************************************************************
 * File Description: Unwinder client that reads local memory.
 *   This client reads from local memory and is designed to run on target
 *   along with the unwinder.  Memory read requests are implemented by
 *   casting a point to read the memory directly, although checks for
 *   alignment should probably also be made if this is to be used in
 *   production code, as otherwise the ARM may return the memory in a
 *   rotated/rolled format, or the MMU may generate an alignment exception
 *   if present and so configured.
 **************************************************************************/

/*
 * Copyright 2022 wtcat 
 */
#include "unwarm.h"

static Boolean  CliReport(void *data, Int32 address) {
    const rtems_printer *printer = data;
    rtems_printf(printer, "< 0x%08x >", address);
    return TRUE;
}

static Boolean CliReadW(const Int32 a, Int32 *v) {
    *v = *(Int32 *)a;
    return TRUE;
}

static Boolean CliReadH(const Int32 a, Int16 *v) {
    *v = *(Int16 *)a;
    return TRUE;
}

static Boolean CliReadB(const Int32 a, Int8 *v) {
    *v = *(Int8 *)a;
    return TRUE;
}

Boolean CliInvalidateW(const Int32 a) {
    *(Int32 *)a = 0xdeadbeef;
    return TRUE;
}

const UnwindCallbacks cliCallbacks = {
    .report = CliReport,
    .readW = CliReadW,
    .readH = CliReadH,
    .readB = CliReadB
#if defined(UNW_DEBUG)
    ,
    .printf = printf
#endif
};