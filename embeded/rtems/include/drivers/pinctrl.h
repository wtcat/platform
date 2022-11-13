/*
 * Copyright wtcat 2022
 */
#ifndef DRIVER_PINCTRL_H_
#define DRIVER_PINCTRL_H_

#include <errno.h>
#include "drivers/devbase.h"
#include <drvmgr/drvmgr.h>

#ifdef __cplusplus
extern "C"{
#endif

struct pinctrl_operations {
    int (*set_state)(struct drvmgr_dev *dev, struct drvmgr_dev *config);
    int (*set_state_np)(struct drvmgr_dev *dev, phandle_t np);
};

static inline int pinctrl_generic_set_state(struct drvmgr_dev *pctldev, 
    struct drvmgr_dev *config) {
    const struct pinctrl_operations *ops = device_get_operations(pctldev);
    _Assert(pctldev != NULL);
    _Assert(ops != NULL);
    _Assert(ops->set_state != NULL);
    return ops->set_state(pctldev, config);
}

static inline int pinctrl_simple_set(const char *pindev, struct drvmgr_dev *config) {
    struct drvmgr_dev *pinctrl = drvmgr_dev_by_name(pindev);
    if (pinctrl)
        return pinctrl_generic_set_state(pinctrl, config);
    return -1;
}

static inline int pinctrl_simple_set_np(const char *pindev, phandle_t np) {
    struct drvmgr_dev *pinctrl = drvmgr_dev_by_name(pindev);
    if (pinctrl) {
        const struct pinctrl_operations *ops = device_get_operations(pinctrl);
        if (ops->set_state_np)
            return ops->set_state_np(pinctrl, np);
    }
    return -1;
}

#ifdef __cplusplus
}
#endif
#endif /* DRIVER_PINCTRL_H_ */
