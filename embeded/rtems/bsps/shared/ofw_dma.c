/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <string.h>

#include "drivers/dma.h"
#include "drivers/ofw_platform_bus.h"


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
