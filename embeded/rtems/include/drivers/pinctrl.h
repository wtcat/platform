/*
 * Copyright wtcat 2022
 */
#ifndef DRIVER_PINCTRL_H_
#define DRIVER_PINCTRL_H_

#include "drivers/devbase.h"
#include <drvmgr/drvmgr.h>

#ifdef __cplusplus
extern "C"{
#endif

struct pinctrl_operations {
    int (*set_state)(struct drvmgr_dev *dev, struct drvmgr_dev *config);
};

static inline int pinctrl_generic_set_state(struct drvmgr_dev *pctldev, 
    struct drvmgr_dev *config) {
    const struct pinctrl_operations *ops = device_get_operations(pctldev);
    _Assert(pctldev != NULL);
    _Assert(ops != NULL);
    _Assert(ops->set_state != NULL);
    return ops->set_state(pctldev, config);
}

#ifdef __cplusplus
}
#endif
#endif /* DRIVER_PINCTRL_H_ */
