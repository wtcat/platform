/*
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 wtcat
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_CLOCK_H_
#define DRIVERS_CLOCK_H_

#include <errno.h>
#include <stddef.h>

#include "drivers/devbase.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*clock_async_cb)(struct drvmgr_dev *dev, void *p, void *user);

/**
 * @brief Current clock status.
 */
enum clock_state {
	CLOCK_STATE_STARTING,
	CLOCK_STATE_OFF,
	CLOCK_STATE_ON,
	CLOCK_STATE_UNAVAILABLE,
	CLOCK_STATE_UNKNOWN
};

struct clock_driver_api {
    int (*enable)(struct drvmgr_dev *dev, void *clk);
    int (*disable)(struct drvmgr_dev *dev, void *clk);
    int (*async_enable)(struct drvmgr_dev *dev, void *clk, clock_async_cb cb, void *user);
    int (*get_rate)(struct drvmgr_dev *dev, void *p, uint32_t *rate);
    enum clock_state (*get_state)(struct drvmgr_dev *dev, void *clk);
};

#define _clock_getops(dev) \
    (struct clock_driver_api *)device_get_operations(dev);

/**
 * @brief Enable a clock controlled by the device
 *
 * On success, the clock is enabled and ready when this function
 * returns. This function may sleep, and thus can only be called from
 * thread context.
 *
 * Use @ref clock_control_async_on() for non-blocking operation.
 *
 * @param dev Device structure whose driver controls the clock.
 * @param clk Opaque data representing the clock.
 * @return 0 on success, negative errno on failure.
 */
static inline int clk_enable(struct drvmgr_dev *dev, void *clk) {
	const struct clock_driver_api *api = _clock_getops(dev);
	return api->enable(dev, clk);
}

/**
 * @brief Disable a clock controlled by the device
 *
 * This function is non-blocking and can be called from any context.
 * On success, the clock is disabled when this function returns.
 *
 * @param dev Device structure whose driver controls the clock
 * @param clk Opaque data representing the clock
 * @return 0 on success, negative errno on failure.
 */
static inline int clk_disable(struct drvmgr_dev *dev, void *clk) {
	const struct clock_driver_api *api = _clock_getops(dev);
	return api->disable(dev, clk);
}

/**
 * @brief Request clock to start with notification when clock has been started.
 *
 * Function is non-blocking and can be called from any context. User callback is
 * called when clock is started.
 *
 * @param dev	    Device.
 * @param clk	    A pointer to an opaque data representing the sub-system.
 * @param cb	    Callback.
 * @param user_data User context passed to the callback.
 *
 * @retval 0 if start is successfully initiated.
 * @retval -EALREADY if clock was already started and is starting or running.
 * @retval -ENOTSUP If the requested mode of operation is not supported.
 * @retval -ENOSYS if the interface is not implemented.
 * @retval other negative errno on vendor specific error.
 */
static inline int clk_async_enable(struct drvmgr_dev *dev, void *clk,
	clock_async_cb cb, void *user_data) {
	const struct clock_driver_api *api = _clock_getops(dev);
	if (api->async_enable == NULL)
		return -ENOSYS;
	return api->async_enable(dev, clk, cb, user_data);
}

/**
 * @brief Get clock status.
 *
 * @param dev Device.
 * @param p A pointer to an opaque data representing the sub-system.
 *
 * @return Status.
 */
static inline enum clock_state clk_get_state(struct drvmgr_dev *dev, void *clk) {
	const struct clock_driver_api *api = _clock_getops(dev);
	if (!api->get_state)
		return CLOCK_STATE_UNKNOWN;
	return api->get_state(dev, clk);
}

/**
 * @brief Obtain the clock rate of given sub-system
 * @param dev Pointer to the device structure for the clock controller driver
 *        instance
 * @param clk A pointer to an opaque data representing the sub-system
 * @param[out] rate Subsystem clock rate
 */
static inline int clk_get_rate(struct drvmgr_dev *dev, void *clk, 
    uint32_t *rate) {
	const struct clock_driver_api *api = _clock_getops(dev);
	if (api->get_rate == NULL)
		return -ENOSYS;
	return api->get_rate(dev, clk, rate);
}

#ifdef CONFIG_OFW
struct drvmgr_dev *ofw_clock_request(phandle_t np, const char *name, 
    pcell_t *pcell, size_t maxsize);
#endif /* CONFIG_OFW */

#ifdef __cplusplus
}
#endif
#endif /* DRIVERS_CLOCK_H_ */
