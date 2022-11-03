/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Copyright (c) 2022 wtcat
 */

#ifndef DRIVER_DMA_H_
#define DRIVER_DMA_H_

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>

#include <rtems/thread.h>

#include "drivers/devbase.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uintptr_t dma_addr_t;

/* magic code to identify context content */
#define DMA_MAGIC 0x47494749

enum dma_channel_direction {
	MEMORY_TO_MEMORY = 0x0,
	MEMORY_TO_PERIPHERAL,
	PERIPHERAL_TO_MEMORY,
	PERIPHERAL_TO_PERIPHERAL /*only supported in NXP EDMA*/
};

/** Valid values for @a source_addr_adj and @a dest_addr_adj */
enum dma_addr_adj {
	DMA_ADDR_ADJ_INCREMENT,
	DMA_ADDR_ADJ_DECREMENT,
	DMA_ADDR_ADJ_NO_CHANGE,
};

/* channel attributes */
enum dma_channel_filter {
	DMA_CHANNEL_NORMAL, /* normal DMA channel */
	DMA_CHANNEL_PERIODIC, /* can be triggerred by periodic sources */
};

/**
 * @struct dma_block_config
 * @brief DMA block configuration structure.
 *
 * @param source_address is block starting address at source
 * @param source_gather_interval is the address adjustment at gather boundary
 * @param dest_address is block starting address at destination
 * @param dest_scatter_interval is the address adjustment at scatter boundary
 * @param dest_scatter_count is the continuous transfer count between scatter
 *                    boundaries
 * @param source_gather_count is the continuous transfer count between gather
 *                     boundaries
 *
 * @param block_size is the number of bytes to be transferred for this block.
 *
 * @param config is a bit field with the following parts:
 *
 *     source_gather_en   [ 0 ]       - 0-disable, 1-enable.
 *     dest_scatter_en    [ 1 ]       - 0-disable, 1-enable.
 *     source_addr_adj    [ 2 : 3 ]   - 00-increment, 01-decrement,
 *                                      10-no change.
 *     dest_addr_adj      [ 4 : 5 ]   - 00-increment, 01-decrement,
 *                                      10-no change.
 *     source_reload_en   [ 6 ]       - reload source address at the end of
 *                                      block transfer
 *                                      0-disable, 1-enable.
 *     dest_reload_en     [ 7 ]       - reload destination address at the end
 *                                      of block transfer
 *                                      0-disable, 1-enable.
 *     fifo_mode_control  [ 8 : 11 ]  - How full  of the fifo before transfer
 *                                      start. HW specific.
 *     flow_control_mode  [ 12 ]      - 0-source request served upon data
 *                                        availability.
 *                                      1-source request postponed until
 *                                        destination request happens.
 *     reserved           [ 13 : 15 ]
 */
struct dma_block_config {
	struct dma_block_config *next_block;
	dma_addr_t source_address;
	dma_addr_t dest_address;
	uint32_t source_gather_interval;
	uint32_t dest_scatter_interval;
	uint16_t dest_scatter_count;
	uint16_t source_gather_count;
	uint32_t block_size;
	uint16_t  source_gather_en :  1;
	uint16_t  dest_scatter_en :   1;
	uint16_t  source_addr_adj :   2;
	uint16_t  dest_addr_adj :     2;
	uint16_t  source_reload_en :  1;
	uint16_t  dest_reload_en :    1;
	uint16_t  fifo_mode_control : 4;
	uint16_t  flow_control_mode : 1;
	uint16_t  reserved :          3;
};

/**
 * @struct dma_config
 * @brief DMA configuration structure.
 *
 * @param dma_slot             [ 0 : 6 ]   - which peripheral and direction
 *                                        (HW specific)
 * @param channel_direction    [ 7 : 9 ]   - 000-memory to memory,
 *                                        001-memory to peripheral,
 *                                        010-peripheral to memory,
 *                                        011-peripheral to peripheral,
 *                                        ...
 * @param complete_callback_en [ 10 ]       - 0-callback invoked at completion only
 *                                        1-callback invoked at completion of
 *                                          each block
 * @param error_callback_en    [ 11 ]      - 0-error callback enabled
 *                                        1-error callback disabled
 * @param source_handshake     [ 12 ]      - 0-HW, 1-SW
 * @param dest_handshake       [ 13 ]      - 0-HW, 1-SW
 * @param channel_priority     [ 14 : 17 ] - DMA channel priority
 * @param source_chaining_en   [ 18 ]      - enable/disable source block chaining
 *                                        0-disable, 1-enable
 * @param dest_chaining_en     [ 19 ]      - enable/disable destination block
 *                                        chaining.
 *                                        0-disable, 1-enable
 * @param linked_channel       [ 20 : 26 ] - after channel count exhaust will
 *                                        initiate a channel service request
 *                                        at this channel
 * @param reserved             [ 27 : 31 ]
 * @param source_data_size    [ 0 : 15 ]   - width of source data (in bytes)
 * @param dest_data_size      [ 16 : 31 ]  - width of dest data (in bytes)
 * @param source_burst_length [ 0 : 15 ]   - number of source data units
 * @param dest_burst_length   [ 16 : 31 ]  - number of destination data units
 * @param block_count  is the number of blocks used for block chaining, this
 *     depends on availability of the DMA controller.
 * @param user_data  private data from DMA client.
 * @param dma_callback see dma_callback_t for details
 */
typedef void (*dma_transfer_cb_t)(struct drvmgr_dev *dev, void *user_data, 
	uint32_t channel, int status);
struct dma_config {
	uint32_t  dma_slot :             7;
	uint32_t  channel_direction :    3;
	uint32_t  complete_callback_en : 1;
	uint32_t  error_callback_en :    1;
	uint32_t  source_handshake :     1;
	uint32_t  dest_handshake :       1;
	uint32_t  channel_priority :     4;
	uint32_t  source_chaining_en :   1;
	uint32_t  dest_chaining_en :     1;
	uint32_t  linked_channel   :     7;
	uint32_t  reserved :             5;
	uint32_t  source_data_size :    16;
	uint32_t  dest_data_size :      16;
	uint32_t  source_burst_length : 16;
	uint32_t  dest_burst_length :   16;
	uint32_t block_count;
	struct dma_block_config *head_block;
	dma_transfer_cb_t dma_callback;
    void *user_data;
};

struct scatterlist {
	dma_addr_t dma_address;
	size_t length;
};

struct dma_chan {
	struct drvmgr_dev *dev;
	struct dma_config config;
	int channel;
};

struct dma_async_tx_descriptor {
	struct dma_chan *chan;
	uint32_t flags;
	void (*callback)(void *param);
	void *param;
};

/* DMA memcpy descriptor */
struct dma_mem_descriptor {
    struct drvmgr_dev *mdma;
    void (*release)(struct dma_mem_descriptor *);
    rtems_id thread;
    int error;
    size_t length;
    size_t nblks;
    struct dma_config head;
    struct dma_block_config blks[];
};

/**
 * DMA runtime status structure
 *
 * busy 			- is current DMA transfer busy or idle
 * dir				- DMA transfer direction
 * pending_length 		- data length pending to be transferred in bytes
 * 					or platform dependent.
 *
 */
struct dma_status {
	bool busy;
	enum dma_channel_direction dir;
	uint32_t pending_length;
};

/**
 * DMA context structure
 * Note: the dma_context shall be the first member
 *       of DMA client driver Data, got by dev->data
 *
 * magic			- magic code to identify the context
 * dma_channels		- dma channels
 * atomic			- driver atomic_t pointer
 *
 */
struct dma_context {
#define DMA_CHANNEL_SIZE(channels) \
	(sizeof(long) * BITS_TO_LONGS(channels))
	rtems_mutex lock;
	uint32_t magic;
	uint32_t dma_channels;
	long *dma_bitmaps;
#ifdef CONFIG_OFW
	struct dma_chan *chans;
#endif
};

/* DMA operations */
struct dma_operations {
    int (*configure)(struct drvmgr_dev *dev, uint32_t channel,
			    struct dma_config *config); 
    int (*reload)(struct drvmgr_dev *dev, uint32_t channel,
                dma_addr_t src, dma_addr_t dst, size_t size);
    int (*start)(struct drvmgr_dev *dev, uint32_t channel);
    int (*stop)(struct drvmgr_dev *dev, uint32_t channel);
    int (*get_status)(struct drvmgr_dev *dev, uint32_t channel,
				struct dma_status *status);
    bool (*chan_filter)(struct drvmgr_dev *dev, int channel, 
                void *filter_param);
	struct dma_mem_descriptor *(*memcpy_prepare)(struct drvmgr_dev *dev, void *dst, 
		const void *src, size_t size);
	
};

#define dmad_get_operations(dev) \
	(struct dma_operations *)device_get_operations(dev)
#define dmad_get_context(dev) \
	(struct dma_context *)((dev)->priv)

/**
 * @brief Configure individual channel for DMA transfer.
 *
 * @param dev     Pointer to the device structure for the driver instance.
 * @param channel Numeric identification of the channel to configure
 * @param config  Data structure containing the intended configuration for the
 *                selected channel
 *
 * @retval 0 if successful.
 * @retval Negative errno code if failure.
 */
static inline int dma_configure(struct drvmgr_dev *dev, uint32_t channel,
    struct dma_config *config) {
    _Assert(dev != NULL);
    _Assert(dev->priv != NULL);
	const struct dma_operations *ops = dmad_get_operations(dev);
	return ops->configure(dev, channel, config);
}

/**
 * @brief Reload buffer(s) for a DMA channel
 *
 * @param dev     Pointer to the device structure for the driver instance.
 * @param channel Numeric identification of the channel to configure
 *                selected channel
 * @param src     source address for the DMA transfer
 * @param dst     destination address for the DMA transfer
 * @param size    size of DMA transfer
 *
 * @retval 0 if successful.
 * @retval Negative errno code if failure.
 */
static inline int dma_reload(struct drvmgr_dev *dev, uint32_t channel,
	dma_addr_t src, dma_addr_t dst, size_t size) {
    _Assert(dev != NULL);
	const struct dma_operations *ops = device_get_operations(dev);
	if (ops->reload)
		return ops->reload(dev, channel, src, dst, size);
	return -ENOSYS;
}

/**
 * @brief Enables DMA channel and starts the transfer, the channel must be
 *        configured beforehand.
 *
 * Implementations must check the validity of the channel ID passed in and
 * return -EINVAL if it is invalid.
 *
 * @param dev     Pointer to the device structure for the driver instance.
 * @param channel Numeric identification of the channel where the transfer will
 *                be processed
 *
 * @retval 0 if successful.
 * @retval Negative errno code if failure.
 */
static inline int dma_start(struct drvmgr_dev *dev, uint32_t channel) {
    _Assert(dev != NULL);
	const struct dma_operations *ops = dmad_get_operations(dev);
	return ops->start(dev, channel);
}

/**
 * @brief Stops the DMA transfer and disables the channel.
 *
 * Implementations must check the validity of the channel ID passed in and
 * return -EINVAL if it is invalid.
 *
 * @param dev     Pointer to the device structure for the driver instance.
 * @param channel Numeric identification of the channel where the transfer was
 *                being processed
 *
 * @retval 0 if successful.
 * @retval Negative errno code if failure.
 */
static inline int dma_stop(struct drvmgr_dev *dev, uint32_t channel) {
    _Assert(dev != NULL);
	const struct dma_operations *ops = dmad_get_operations(dev);
	return ops->stop(dev, channel);
}

/**
 * @brief DMA channel filter.
 *
 * filter channel by attribute
 *
 * @param dev  Pointer to the device structure for the driver instance.
 * @param channel  channel number
 * @param filter_param filter attribute
 *
 * @retval Negative errno code if not support
 *
 */
static inline int dma_chan_filter(struct drvmgr_dev *dev,
    int channel, void *filter_param) {
    _Assert(dev != NULL);
    const struct dma_operations *ops = dmad_get_operations(dev);
	if (ops->chan_filter) 
		return ops->chan_filter(dev, channel, filter_param);
	return -ENOSYS;
}

/**
 * @brief get current runtime status of DMA transfer
 *
 * Implementations must check the validity of the channel ID passed in and
 * return -EINVAL if it is invalid or -ENOSYS if not supported.
 *
 * @param dev     Pointer to the device structure for the driver instance.
 * @param channel Numeric identification of the channel where the transfer was
 *                being processed
 * @param stat   a non-NULL dma_status object for storing DMA status
 *
 * @retval non-negative if successful.
 * @retval Negative errno code if failure.
 */
static inline int dma_get_status(struct drvmgr_dev *dev, uint32_t channel,
	struct dma_status *stat) {
    _Assert(dev != NULL);
    const struct dma_operations *ops = dmad_get_operations(dev);
	if (ops->get_status) 
		return ops->get_status(dev, channel, stat);
	return -ENOSYS;
}


static inline struct dma_mem_descriptor *dma_memcpy_prepare(struct drvmgr_dev *dev, 
	void *dst, const void *src, size_t size) {
    _Assert(dev != NULL);
    const struct dma_operations *ops = dmad_get_operations(dev);
	return ops->memcpy_prepare(dev, dst, src, size);
}

ssize_t dma_memcpy_sync(void *dst, const void *src, size_t size);

ssize_t dma_memcpy_async(void *dst, const void *src, size_t size, 
    dma_transfer_cb_t cb, void *arg);

int dma_mdev_register(struct drvmgr_dev *mdma);
/**
 * @brief request DMA channel.
 *
 * request DMA channel resources
 * return -EINVAL if there is no valid channel available.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param filter_param filter function parameter
 *
 * @retval dma channel if successful.
 * @retval Negative errno code if failure.
 */
int dma_request_channel(struct drvmgr_dev *dev, void *filter_param);
    
/**
 * @brief release DMA channel.
 *
 * release DMA channel resources
 *
 * @param dev  Pointer to the device structure for the driver instance.
 * @param channel  channel number
 *
 */
void dma_release_channel(struct drvmgr_dev *dev, uint32_t channel);

/**
 * @brief Initialize DMA context.
 *
 * @param dev  Pointer to the device context structure for the driver instance.
 * @param bitmaps  Pointer to the bitmap buffer.
 * @param channel  channel number
 *
 */
int dma_context_init(struct dma_context *ctx, uint32_t max_channels);
	
/*
 * DMA Channel public interface
 */
static inline int dma_chan_configure(struct dma_chan *chan, struct dma_config *config) {
	return dma_configure(chan->dev, chan->channel, config);
}

static inline int dma_chan_reload(struct dma_chan *chan, dma_addr_t src, 
	dma_addr_t dst, size_t size) {
	return dma_reload(chan->dev, chan->channel, src, dst, size);
}

static inline int dma_chan_start(struct dma_chan *chan) {
	return dma_start(chan->dev, chan->channel);
}

static inline int dma_chan_stop(struct dma_chan *chan) {
	return dma_stop(chan->dev, chan->channel);
}

static inline int dma_chan_get_status(struct dma_chan *chan, 
	struct dma_status *stat) {
	return dma_get_status(chan->dev, chan->channel, stat);
}

static inline bool dma_chan_do_filter(struct dma_chan *chan, void *param) {
	return dma_chan_filter(chan->dev, chan->channel, param);
}

static inline void dma_chan_release(struct dma_chan *chan) {
	dma_release_channel(chan->dev, chan->channel);
}

/*
 * OFW public inteface
 */
#ifdef CONFIG_OFW
struct dma_chan *ofw_dma_chan_request(phandle_t np, const char *name, 
    pcell_t *pcell, size_t maxsize);
#endif

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_DMA_H_ */
