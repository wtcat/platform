/*
 * Copyright 2022 wtcat
 */
#ifndef BASE_INIT_H_
#define BASE_INIT_H_

#include <rtems/sysinit.h>

#ifdef __cplusplus
extern "C"{
#endif

/* at begin of the Init thread */
#define SYSINIT_INIT_BEGIN       007000

/* The enum helps to detect typos in the module and order parameters */
#define _SYSINIT_INDEX_ITEM(handler, index) \
  enum { _Sysinit_##handler = index }; \
  RTEMS_LINKER_ROSET_ITEM_ORDERED( \
    _Init, \
    rtems_sysinit_item, \
    handler, \
    index \
  ) = { handler }

/* Create index from module and order */
#define _SYSINIT_ITEM(handler, module, order) \
  _SYSINIT_INDEX_ITEM(handler, 0x##module##order)

/* Perform parameter expansion */
#define SYSINIT_ITEM(handler, module, order) \
  _SYSINIT_ITEM(handler, module, order)


static inline void sysinit_post(void) {
    RTEMS_LINKER_ROSET_DECLARE(_Init, rtems_sysinit_item);
    rtems_sysinit_item *iter;
    RTEMS_LINKER_SET_FOREACH(_Init, iter) {
        iter->handler();
    }
}

#ifdef __cplusplus
}
#endif
#endif /* BASE_INIT_H_ */
