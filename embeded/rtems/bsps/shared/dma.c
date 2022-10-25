/*
 * Copyright 2022 wtcat
 */
#include <rtems/thread.h>
#include <rtems/malloc.h>
#include <rtems/bspIo.h>

#include "base/bitops.h"
#include "drivers/dma.h"


#define dmg_dbg(fmt, ...) printk(fmt, ##__VA_ARGS__)

int dma_context_init(struct dma_context *ctx, uint32_t max_channels) {
    _Assert(ctx != NULL);
    _Assert(bitmaps != NULL);
    size_t alloc_size = 0;
    char *p;
#ifdef CONFIG_OFW
    alloc_size += sizeof(struct dma_chan) * max_channels;
#endif
    p = rtems_calloc(1, alloc_size + DMA_CHANNEL_SIZE(max_channels));
    if (!p)
        return -ENOMEM;
    rtems_mutex_init(&ctx->lock, "dma");
    ctx->dma_channels = max_channels;
    ctx->magic = DMA_MAGIC;
    ctx->dma_bitmaps = (void *)(p + alloc_size);
#ifdef CONFIG_OFW
    ctx->chans = (struct dma_chan *)p;
#endif
    return 0;
}

int dma_request_channel(struct drvmgr_dev *dev, void *filter_param) {
	uint32_t channel;
    _Assert(dev != NULL);
    const struct dma_operations *ops = dmad_get_operations(dev);
	struct dma_context *ctx = dmad_get_context(dev);
	if (ctx->magic != DMA_MAGIC) {
        dmg_dbg("%s: dma context is not ready!\n", __func__);
		return -EINVAL;
    }
    rtems_mutex_lock(&ctx->lock);
	channel = (int)find_first_zero_bit((void *)ctx->dma_bitmaps, 
    ctx->dma_channels);
	if (channel >= ctx->dma_channels) {
        rtems_mutex_unlock(&ctx->lock);
        dmg_dbg("%s: channel is busy!\n", __func__);
		return -EBUSY;
    }
	if (ops->chan_filter &&
		!ops->chan_filter(dev, channel, filter_param)) {
        rtems_mutex_unlock(&ctx->lock);
        dmg_dbg("%s: channel was disabled!\n", __func__);
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
