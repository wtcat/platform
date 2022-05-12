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
 * File Description: Implementation of the interface into the ARM unwinder.
 **************************************************************************/

/*
 * Copyright 2022 wtcat
 */
#include "unwarm.h"


UnwResult UnwindStart(Int32                  spValue,
                      Int32                  retAddress,   
                      const UnwindCallbacks *cb,
                      void                  *data)
{
    UnwState state;

    /* Initialise the unwinding state */
    UnwInitState(&state, cb, data, retAddress, spValue);

    /* Check the Thumb bit */
    if(retAddress & 0x1)
    {
        return UnwStartThumb(&state);
    }
    else
    {
        return UnwStartArm(&state);
    }
}
