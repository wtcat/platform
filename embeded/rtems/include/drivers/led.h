/*
 * Copyright (c) 2018 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 /*
  * Copyright 2022 wtcat
  */

#ifndef DRIVERS_LED_H_
#define DRIVERS_LED_H_

#include <errno.h>
#include <stdint.h>

#include "drivers/devbase.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LED information structure
 *
 * This structure gathers useful information about LED controller.
 *
 * @param label LED label.
 * @param num_colors Number of colors per LED.
 * @param index Index of the LED on the controller.
 * @param color_mapping Mapping of the LED colors.
 */
struct led_info {
	const char *label;
	uint32_t index;
	uint8_t num_colors;
	const uint8_t *color_mapping;
};

/**
 * @brief LED driver operations
 */
struct led_operations {
    int (*on)(struct drvmgr_dev *dev, uint32_t led);
    int (*off)(struct drvmgr_dev *dev, uint32_t led);

    /* Option operations */
    int (*blink)(struct drvmgr_dev *dev, uint32_t led, uint32_t delay_on, 
        uint32_t delay_off);        
    int (*set_brightness)(struct drvmgr_dev *dev, uint32_t led, uint8_t value);
    int (*set_color)(struct drvmgr_dev *dev, uint32_t led, uint8_t num_colors, 
        const uint8_t *color);	
    int (*get_info)(struct drvmgr_dev *dev, uint32_t led, const struct led_info **info);
    int (*write_channels)(struct drvmgr_dev *dev, uint32_t start_channel, 
        uint32_t num_channels, const uint8_t *buf);                 	      
};


/**
 * @brief Blink an LED
 *
 * This optional routine starts blinking a LED forever with the given time
 * period.
 *
 * @param dev LED drvmgr_dev
 * @param led LED number
 * @param delay_on Time period (in milliseconds) an LED should be ON
 * @param delay_off Time period (in milliseconds) an LED should be OFF
 * @return 0 on success, negative on error
 */
static inline int led_blink(struct drvmgr_dev *dev, uint32_t led,
    uint32_t delay_on, uint32_t delay_off) {
	const struct led_operations *ops = device_get_operations(dev);
	if (ops->blink == NULL) 
		return -ENOSYS;
	return ops->blink(dev, led, delay_on, delay_off);
}

/**
 * @brief Get LED information
 *
 * This optional routine provides information about a LED.
 *
 * @param dev LED drvmgr_dev
 * @param led LED number
 * @param info Pointer to a pointer filled with LED information
 * @return 0 on success, negative on error
 */
static inline int led_get_info(struct drvmgr_dev *dev, uint32_t led,
    const struct led_info **info) {
    const struct led_operations *ops = device_get_operations(dev);
    if (ops->get_info == NULL) {
        *info = NULL;
        return -ENOSYS;
    }
    return ops->get_info(dev, led, info);
}

/**
 * @brief Set LED brightness
 *
 * This optional routine sets the brightness of a LED to the given value.
 * Calling this function after led_blink() won't affect blinking.
 *
 * LEDs which can only be turned on or off may provide this function.
 * These should simply turn the LED on if @p value is nonzero, and off
 * if @p value is zero.
 *
 * @param dev LED drvmgr_dev
 * @param led LED number
 * @param value Brightness value to set in percent
 * @return 0 on success, negative on error
 */
static inline int led_set_brightness(struct drvmgr_dev *dev, uint32_t led, 
    uint8_t value) {	    
    const struct led_operations *ops = device_get_operations(dev);
	if (ops->set_brightness == NULL) 
		return -ENOSYS;
	return ops->set_brightness(dev, led, value);
}

/**
 * @brief Write/update a strip of LED channels
 *
 * This optional routine writes a strip of LED channels to the given array of
 * levels. Therefore it can be used to configure several LEDs at the same time.
 *
 * Calling this function after led_blink() won't affect blinking.
 *
 * @param dev LED drvmgr_dev
 * @param start_channel Absolute number (i.e. not relative to a LED) of the
 *        first channel to update.
 * @param num_channels The number of channels to write/update.
 * @param buf array of values to configure the channels with. num_channels
 *        entries must be provided.
 * @return 0 on success, negative on error
 */
static inline int led_write_channels(struct drvmgr_dev *dev, uint32_t start_channel,
    uint32_t num_channels, const uint8_t *buf) {
    const struct led_operations *ops = device_get_operations(dev);
	if (ops->write_channels == NULL) 
		return -ENOSYS;
	return ops->write_channels(dev, start_channel, num_channels, buf);
}

/**
 * @brief Set a single LED channel
 *
 * This optional routine sets a single LED channel to the given value.
 *
 * Calling this function after led_blink() won't affect blinking.
 *
 * @param dev LED drvmgr_dev
 * @param channel Absolute channel number (i.e. not relative to a LED)
 * @param value Value to configure the channel with
 * @return 0 on success, negative on error
 */
static inline int led_set_channel(struct drvmgr_dev *dev, uint32_t channel, 
    uint8_t value) { 
	return led_write_channels(dev, channel, 1, &value);
}

/**
 * @brief Set LED color
 *
 * This routine configures all the color channels of a LED with the given
 * color array.
 *
 * Calling this function after led_blink() won't affect blinking.
 *
 * @param dev LED drvmgr_dev
 * @param led LED number
 * @param num_colors Number of colors in the array.
 * @param color Array of colors. It must be ordered following the color
 *        mapping of the LED controller. See the the color_mapping member
 *        in struct led_info.
 * @return 0 on success, negative on error
 */
static inline int led_set_color(struct drvmgr_dev *dev, uint32_t led,
    uint8_t num_colors, const uint8_t *color) {
    const struct led_operations *ops = device_get_operations(dev);
	if (ops->set_color == NULL)
		return -ENOSYS;
	return ops->set_color(dev, led, num_colors, color);
}

/**
 * @brief Turn on an LED
 *
 * This routine turns on an LED
 *
 * @param dev LED drvmgr_dev
 * @param led LED number
 * @return 0 on success, negative on error
 */
static inline int led_on(struct drvmgr_dev *dev, uint32_t led) {
    const struct led_operations *ops = device_get_operations(dev);
	return ops->on(dev, led);
}

/**
 * @brief Turn off an LED
 *
 * This routine turns off an LED
 *
 * @param dev LED drvmgr_dev
 * @param led LED number
 * @return 0 on success, negative on error
 */
static inline int led_off(struct drvmgr_dev *dev, uint32_t led) {
    const struct led_operations *ops = device_get_operations(dev);
	return ops->off(dev, led);
}

#ifdef __cplusplus
}
#endif

#endif	/* DRIVERS_LED_H_ */
