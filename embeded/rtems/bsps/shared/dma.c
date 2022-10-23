/*
 * Copyright 2022 wtcat
 */
#include <rtems/thread.h>
#include <rtems/malloc.h>
#include <rtems/bspIo.h>

#include "base/bitops.h"
#include "drivers/dma.h"

#ifdef CONFIG_OFW
#include <string.h>
#include "drivers/ofw_platform_bus.h"
#endif

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

#ifdef CONFIG_OFW
struct dma_chan *ofw_dma_chan_request(phandle_t np, const char *name, 
    pcell_t *pcell, size_t maxsize) {
    struct drvmgr_dev *dev;
    struct dma_context *ctx;
    char buffer[128];
    pcell_t args[16];
    phandle_t dmanp;
    pcell_t dma_cells;
    int count, nargs, ofs;
    size_t bytes;
    int chan;
    
    nargs = rtems_ofw_get_enc_prop(np, "dmas", args, sizeof(args));
    if (nargs < 0) {
        errno = -ENODATA;
        return NULL;
    }
    count = ofw_property_count_strings(np, "dma-names", 
        buffer, sizeof(buffer));
    ofs = 0;
    for (int i = 0; i < count; i++) {
        const char *output = NULL;
        dev = ofw_device_get_by_phandle(args[ofs]);
        if (dev == NULL) {
            errno = -ENODEV;
            return NULL;
        }
        dmanp = ofw_phandle_get(dev);
        if (rtems_ofw_get_enc_prop(dmanp, "#dma-cells", 
            &dma_cells, sizeof(dma_cells)) < 0) {
            errno = -ENOSTR;
            return NULL;
        }

        ofw_property_read_string_index(np, "dma-names", i, 
            &output, buffer, sizeof(buffer));
        if (!strcmp(output, name))
            goto _next;
    
        ofs += 1 + dma_cells;
    }
    errno = -EINVAL;
    return NULL;

_next:
    chan = dma_request_channel(dev, NULL);
    if (chan >= 0) {
        bytes = dma_cells * sizeof(pcell_t);
        memcpy(pcell, &args[ofs+1], min_t(size_t, maxsize, bytes));     
        ctx = dmad_get_context(dev);
        ctx->chans[chan].channel = chan;
        ctx->chans[chan].dev = dev;
        return &ctx->chans[chan];
    }
    return NULL;
}
#endif /* CONFIG_OFW */
