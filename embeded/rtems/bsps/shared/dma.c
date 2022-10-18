/*
 * Copyright 2022 wtcat
 */
#include <rtems/thread.h>
#include <rtems/malloc.h>

#include "base/bitops.h"
#include "drivers/dma.h"

void dma_context_init(struct dma_context *ctx, void *bitmaps, 
    uint32_t channel) {
    _Assert(ctx != NULL);
    _Assert(bitmaps != NULL);
    rtems_mutex_init(&ctx->lock, "dma");
    ctx->dma_channels = channel;
    ctx->magic = DMA_MAGIC;
    ctx->dma_bitmaps = bitmaps;
}

int dma_request_channel(struct drvmgr_dev *dev, void *filter_param) {
	uint32_t channel;
    _Assert(dev != NULL);
    const struct dma_operations *ops = dmad_get_operations(dev);
	struct dma_context *ctx = dmad_get_context(dev);
	if (ctx->magic != DMA_MAGIC)
		return -EINVAL;
    rtems_mutex_lock(&ctx->lock);
	channel = (int)find_first_zero_bit((void *)ctx->dma_bitmaps, 
    ctx->dma_channels);
	if (channel >= ctx->dma_channels) {
        rtems_mutex_unlock(&ctx->lock);
		return -EBUSY;
    }
	if (ops->chan_filter &&
		!ops->chan_filter(dev, channel, filter_param)) {
        rtems_mutex_unlock(&ctx->lock);
		return -EINVAL;
    }
	set_bit(channel, ctx->dma_bitmaps);
    rtems_mutex_unlock(&ctx->lock);
	return (int)channel;
}

void dma_release_channel(struct drvmgr_dev *dev, uint32_t channel) {
    _Assert(dev != NULL);
	struct dma_context *ctx = dmad_get_context(dev);
	if (ctx->magic != DMA_MAGIC) 
		return;
    rtems_mutex_lock(&ctx->lock);
	if (channel < ctx->dma_channels) 
		clear_bit(channel, ctx->dma_bitmaps);
    rtems_mutex_unlock(&ctx->lock);
}
