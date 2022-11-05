/*
 * Copyright 2022 wtcat
 */

#ifndef DRIVER_MMC_MMC_BASE_H_
#define DRIVER_MMC_MMC_BASE_H_

#include "drivers/devbase.h"

#ifdef __cplusplus
extern "C"{
#endif

#ifndef MMC_ASSERT
#define MMC_ASSERT(c) (void) (c)
#endif

struct drvmgr_dev;
struct mmc_base_ops {
	int (*read_ivar)(struct drvmgr_dev *bus, struct drvmgr_dev *child, int which, uintptr_t *result);
	int (*write_ivar)(struct drvmgr_dev *bus, struct drvmgr_dev *child, int which, uintptr_t value);
};

#define __MMC_BUS_ACCESSOR(varp, var, ivarp, ivar, type)			\
    static inline type varp ## _get_ ## var(struct drvmgr_dev *dev) {			\
        uintptr_t v;							\
        int err;							\
        err = __mmc_read_ivar(device_get_parent(dev), dev,		\
            ivarp ## _IVAR_ ## ivar, &v);				\
        MMC_ASSERT(err == 0);						\
        return ((type) v);						\
    }									\
                                        \
    static inline void varp ## _set_ ## var(struct drvmgr_dev *dev, type t) {		\
        uintptr_t v = (uintptr_t) t;					\
        __mmc_write_ivar(device_get_parent(dev), dev,			\
            ivarp ## _IVAR_ ## ivar, v);				\
    }

static inline int __mmc_read_ivar(struct drvmgr_dev *dev, struct drvmgr_dev *child, 
    int index, uintptr_t *result) {
	const struct mmc_base_ops *ops = device_get_operations(dev);
    return ops->read_ivar(dev, child, index, result);
}

static inline int __mmc_write_ivar(struct drvmgr_dev *dev, struct drvmgr_dev *child, 
    int index, uintptr_t value) {
	const struct mmc_base_ops *ops = device_get_operations(dev);
    return ops->write_ivar(dev, child, index, value);
}
                                  
#ifdef __cplusplus
}
#endif
#endif /* DRIVER_MMC_MMC_BASE_H_ */