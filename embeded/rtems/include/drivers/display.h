/*
 * Copyright (c) 2017 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 /*
  * Copyright (c) 2022 wtcat
  */

/**
 * @file
 * @brief Public API for display drivers and applications
 */

#ifndef DRIVERS_DISPLAY_H_
#define DRIVERS_DISPLAY_H_

/**
 * @brief Display Interface
 * @defgroup display_interface Display Interface
 * @ingroup display_interfaces
 * @{
 */

#include <errno.h>
#include <stddef.h>

#include "drivers/platform_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Display pixel formats
 *
 * Display pixel format enumeration.
 *
 * In case a pixel format consists out of multiple bytes the byte order is
 * big endian.
 */
enum display_pixel_format {
	PIXEL_FORMAT_RGB_888		= BIT(0),
	PIXEL_FORMAT_MONO01		= BIT(1), /* 0=Black 1=White */
	PIXEL_FORMAT_MONO10		= BIT(2), /* 1=Black 0=White */
	PIXEL_FORMAT_ARGB_8888		= BIT(3),
	PIXEL_FORMAT_RGB_565		= BIT(4),
	PIXEL_FORMAT_BGR_565		= BIT(5),
};

enum display_screen_info {
	/**
	 * If selected, one octet represents 8 pixels ordered vertically,
	 * otherwise ordered horizontally.
	 */
	SCREEN_INFO_MONO_VTILED		= BIT(0),
	/**
	 * If selected, the MSB represents the first pixel,
	 * otherwise MSB represents the last pixel.
	 */
	SCREEN_INFO_MONO_MSB_FIRST	= BIT(1),
	/**
	 * Electrophoretic Display.
	 */
	SCREEN_INFO_EPD			= BIT(2),
	/**
	 * Screen has two alternating ram buffers
	 */
	SCREEN_INFO_DOUBLE_BUFFER	= BIT(3),
	/**
	 * Screen has x alignment constrained to width.
	 */
	SCREEN_INFO_X_ALIGNMENT_WIDTH	= BIT(4),
};

/**
 * @enum display_orientation
 * @brief Enumeration with possible display orientation
 *
 */
enum display_orientation {
	DISPLAY_ORIENTATION_NORMAL,
	DISPLAY_ORIENTATION_ROTATED_90,
	DISPLAY_ORIENTATION_ROTATED_180,
	DISPLAY_ORIENTATION_ROTATED_270,
};

/** @brief Structure holding display capabilities. */
struct display_capabilities {
	/** Display resolution in the X direction */
	uint16_t x_resolution;
	/** Display resolution in the Y direction */
	uint16_t y_resolution;
	/** Bitwise or of pixel formats supported by the display */
	uint32_t supported_pixel_formats;
	/** Information about display panel */
	uint32_t screen_info;
	/** Currently active pixel format for the display */
	enum display_pixel_format current_pixel_format;
	/** Current display orientation */
	enum display_orientation current_orientation;
};

/** @brief Structure to describe display data buffer layout */
struct display_buffer_descriptor {
	/** Data buffer size in bytes */
	uint32_t buf_size;
	/** Data buffer row width in pixels */
	uint16_t width;
	/** Data buffer column height in pixels */
	uint16_t height;
	/** Number of pixels between consecutive rows in the data buffer */
	uint16_t pitch;
};


/* enum display_screen_info */
#define SCREEN_INFO_VSYNC	    BIT(5) /* Screen has vsync */
#define SCREEN_INFO_ZERO_BUFFER	BIT(6) /* Screen has no ram buffers */

/**
 * @struct display_callback
 * @brief Structure to register specific event callback
 */
struct display_callback {
	/* (*vsync)() is called when a vsync event is received (called in isr),
	 * timestamp is measured in cycles.
	 */
	void (*vsync)(const struct display_callback *callback, uint32_t timestamp);
	/* (*complete)() is called when refresh complete (called in isr) */
	void (*complete)(const struct display_callback *callback);
	/* (*pm_notify) is called when pm state changed */
	void (*pm_notify)(const struct display_callback *callback, uint32_t pm_action);
};

/**
 * @typedef display_blanking_on_api
 * @brief Callback API to turn on display blanking
 * See display_blanking_on() for argument description
 */
typedef int (*display_blanking_on_api)(struct drvmgr_dev *dev);

/**
 * @typedef display_blanking_off_api
 * @brief Callback API to turn off display blanking
 * See display_blanking_off() for argument description
 */
typedef int (*display_blanking_off_api)(struct drvmgr_dev *dev);

/**
 * @typedef display_write_api
 * @brief Callback API for writing data to the display
 * See display_write() for argument description
 */
typedef int (*display_write_api)(struct drvmgr_dev *dev, const uint16_t x,
				 const uint16_t y,
				 const struct display_buffer_descriptor *desc,
				 const void *buf);

/**
 * @typedef display_read_api
 * @brief Callback API for reading data from the display
 * See display_read() for argument description
 */
typedef int (*display_read_api)(struct drvmgr_dev *dev, const uint16_t x,
				const uint16_t y,
				const struct display_buffer_descriptor *desc,
				void *buf);

/**
 * @typedef display_get_framebuffer_api
 * @brief Callback API to get framebuffer pointer
 * See display_get_framebuffer() for argument description
 */
typedef void *(*display_get_framebuffer_api)(struct drvmgr_dev *dev);

/**
 * @typedef display_set_brightness_api
 * @brief Callback API to set display brightness
 * See display_set_brightness() for argument description
 */
typedef int (*display_set_brightness_api)(struct drvmgr_dev *dev,
					  const uint8_t brightness);

/**
 * @typedef display_set_contrast_api
 * @brief Callback API to set display contrast
 * See display_set_contrast() for argument description
 */
typedef int (*display_set_contrast_api)(struct drvmgr_dev *dev,
					const uint8_t contrast);

/**
 * @typedef display_get_capabilities_api
 * @brief Callback API to get display capabilities
 * See display_get_capabilities() for argument description
 */
typedef void (*display_get_capabilities_api)(struct drvmgr_dev *dev,
					     struct display_capabilities *
					     capabilities);

/**
 * @typedef display_set_pixel_format_api
 * @brief Callback API to set pixel format used by the display
 * See display_set_pixel_format() for argument description
 */
typedef int (*display_set_pixel_format_api)(struct drvmgr_dev *dev,
					    const enum display_pixel_format
					    pixel_format);

/**
 * @typedef display_set_orientation_api
 * @brief Callback API to set orientation used by the display
 * See display_set_orientation() for argument description
 */
typedef int (*display_set_orientation_api)(struct drvmgr_dev *dev,
					   const enum display_orientation
					   orientation);

/**
 * @typedef display_register_callback_api
 * @brief Callback API to register event callback
 * See display_register_callback() for argument description
 */
typedef int (*display_register_callback_api)(struct drvmgr_dev *dev,
					   const struct display_callback *callback);

/**
 * @typedef display_unregister_callback_api
 * @brief Callback API to unregister event callback
 * See display_unregister_callback() for argument description
 */
typedef int (*display_unregister_callback_api)(struct drvmgr_dev *dev,
					   const struct display_callback *callback);
/**
 * @brief Display driver API
 * API which a display driver should expose
 */
struct display_driver {
	display_blanking_on_api blanking_on;
	display_blanking_off_api blanking_off;
	display_write_api write;
	display_read_api read;
	display_get_framebuffer_api get_framebuffer;
	display_set_brightness_api set_brightness;
	display_set_contrast_api set_contrast;
	display_get_capabilities_api get_capabilities;
	display_set_pixel_format_api set_pixel_format;
	display_set_orientation_api set_orientation;
	display_register_callback_api register_callback;
	display_unregister_callback_api unregister_callback;
};

/**
 * @brief Write data to display
 *
 * @param dev Pointer to device structure
 * @param x x Coordinate of the upper left corner where to write the buffer
 * @param y y Coordinate of the upper left corner where to write the buffer
 * @param desc Pointer to a structure describing the buffer layout
 * @param buf Pointer to buffer array
 *
 * @retval 0 on success else negative errno code.
 */
static inline int display_write(struct drvmgr_dev *dev, const uint16_t x,
				const uint16_t y,
				const struct display_buffer_descriptor *desc,
				const void *buf)
{
	struct display_driver *api =
		(struct display_driver *)device_get_operations(dev);

	return api->write(dev, x, y, desc, buf);
}

/**
 * @brief Read data from display
 *
 * @param dev Pointer to device structure
 * @param x x Coordinate of the upper left corner where to read from
 * @param y y Coordinate of the upper left corner where to read from
 * @param desc Pointer to a structure describing the buffer layout
 * @param buf Pointer to buffer array
 *
 * @retval 0 on success else negative errno code.
 */
static inline int display_read(struct drvmgr_dev *dev, const uint16_t x,
			       const uint16_t y,
			       const struct display_buffer_descriptor *desc,
			       void *buf)
{
	struct display_driver *api =
		(struct display_driver *)device_get_operations(dev);

	return api->read(dev, x, y, desc, buf);
}

/**
 * @brief Get pointer to framebuffer for direct access
 *
 * @param dev Pointer to device structure
 *
 * @retval Pointer to frame buffer or NULL if direct framebuffer access
 * is not supported
 *
 */
static inline void *display_get_framebuffer(struct drvmgr_dev *dev)
{
	struct display_driver *api =
		(struct display_driver *)device_get_operations(dev);

	return api->get_framebuffer(dev);
}

/**
 * @brief Turn display blanking on
 *
 * This function blanks the complete display.
 * The content of the frame buffer will be retained while blanking is enabled
 * and the frame buffer will be accessible for read and write operations.
 *
 * In case backlight control is supported by the driver the backlight is
 * turned off. The backlight configuration is retained and accessible for
 * configuration.
 *
 * In case the driver supports display blanking the initial state of the driver
 * would be the same as if this function was called.
 *
 * @param dev Pointer to device structure
 *
 * @retval 0 on success else negative errno code.
 */
static inline int display_blanking_on(struct drvmgr_dev *dev)
{
	struct display_driver *api =
		(struct display_driver *)device_get_operations(dev);

	return api->blanking_on(dev);
}

/**
 * @brief Turn display blanking off
 *
 * Restore the frame buffer content to the display.
 * In case backlight control is supported by the driver the backlight
 * configuration is restored.
 *
 * @param dev Pointer to device structure
 *
 * @retval 0 on success else negative errno code.
 */
static inline int display_blanking_off(struct drvmgr_dev *dev)
{
	struct display_driver *api =
		(struct display_driver *)device_get_operations(dev);

	return api->blanking_off(dev);
}

/**
 * @brief Set the brightness of the display
 *
 * Set the brightness of the display in steps of 1/256, where 255 is full
 * brightness and 0 is minimal.
 *
 * @param dev Pointer to device structure
 * @param brightness Brightness in steps of 1/256
 *
 * @retval 0 on success else negative errno code.
 */
static inline int display_set_brightness(struct drvmgr_dev *dev,
					 uint8_t brightness)
{
	struct display_driver *api =
		(struct display_driver *)device_get_operations(dev);

	return api->set_brightness(dev, brightness);
}

/**
 * @brief Set the contrast of the display
 *
 * Set the contrast of the display in steps of 1/256, where 255 is maximum
 * difference and 0 is minimal.
 *
 * @param dev Pointer to device structure
 * @param contrast Contrast in steps of 1/256
 *
 * @retval 0 on success else negative errno code.
 */
static inline int display_set_contrast(struct drvmgr_dev *dev, uint8_t contrast)
{
	struct display_driver *api =
		(struct display_driver *)device_get_operations(dev);

	return api->set_contrast(dev, contrast);
}

/**
 * @brief Get display capabilities
 *
 * @param dev Pointer to device structure
 * @param capabilities Pointer to capabilities structure to populate
 */
static inline void display_get_capabilities(struct drvmgr_dev *dev,
					    struct display_capabilities *
					    capabilities)
{
	struct display_driver *api =
		(struct display_driver *)device_get_operations(dev);

	api->get_capabilities(dev, capabilities);
}

/**
 * @brief Set pixel format used by the display
 *
 * @param dev Pointer to device structure
 * @param pixel_format Pixel format to be used by display
 *
 * @retval 0 on success else negative errno code.
 */
static inline int
display_set_pixel_format(struct drvmgr_dev *dev,
			 const enum display_pixel_format pixel_format)
{
	struct display_driver *api =
		(struct display_driver *)device_get_operations(dev);

	return api->set_pixel_format(dev, pixel_format);
}

/**
 * @brief Set display orientation
 *
 * @param dev Pointer to device structure
 * @param orientation Orientation to be used by display
 *
 * @retval 0 on success else negative errno code.
 */
static inline int display_set_orientation(struct drvmgr_dev *dev,
					  const enum display_orientation
					  orientation)
{
	struct display_driver *api =
		(struct display_driver *)device_get_operations(dev);

	return api->set_orientation(dev, orientation);
}

/**
 * @brief Register display event callback
 *
 * @param dev Pointer to device structure
 * @param callback Pointer to structure containing the event callbacks
 *
 * @retval 0 on success else negative errno code.
 */
static inline int display_register_callback(struct drvmgr_dev *dev,
				const struct display_callback *callback)
{
	struct display_driver *api =
		(struct display_driver *)device_get_operations(dev);
	if (api->register_callback)
		return api->register_callback(dev, callback);
	return -ENOSYS;
}

/**
 * @brief Unregister display event callback
 *
 * @param dev Pointer to device structure
 * @param callback Pointer to structure containing the event callbacks
 *
 * @retval 0 on success else negative errno code.
 */
static inline int display_unregister_callback(struct drvmgr_dev *dev,
				const struct display_callback *callback)
{
	struct display_driver *api =
		(struct display_driver *)dev->priv;
	if (api->unregister_callback) 
		return api->unregister_callback(dev, callback);
	return -ENOSYS;
}

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_DISPLAY_H_ */
