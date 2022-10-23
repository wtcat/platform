/*
 * Copyright 2022 wtcat
 */
#include <rtems.h>
#include <rtems/bspIo.h>

#include "drivers/dma.h"


static struct drvmgr_dev *mdma_device;

static void dma_memcpy_cb(struct drvmgr_dev *dev, void *arg, 
    uint32_t channel, int status) {
    (void) dev;
    (void) channel;
    struct mdma_desc *desc = (struct mdma_desc *)arg;
    desc->error = status;
    rtems_event_transient_send(desc->thread);
}

static int dma_wait_transfer_complete(struct mdma_desc *desc, int chan) {
    int ret;
    desc->error = -1;
    desc->thread = rtems_task_self();
    ret = dma_start(desc->mdma, chan);
    if (!ret) {
        rtems_event_transient_receive(RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        ret = desc->error;
    }
    return ret;
}

ssize_t dma_memcpy_sync(void *dst, const void *src, size_t size) {
    struct mdma_desc *desc;
    int chan, ret;

    chan = dma_request_channel(mdma_device, NULL);
    if (chan < 0) {
        printk("Request dma channel failed!\n");
        return -ENODEV;
    }
    desc = dma_memcpy_prepare(mdma_device, dst, src, size);
    if (!desc) {
        ret = -ENOMEM;
        goto _free_ch;
    }
    desc->head.dma_callback = dma_memcpy_cb;
    desc->head.user_data = desc;

    ret = dma_configure(desc->mdma, chan, &desc->head);
    if (ret) {
        ret = -EIO;
        goto _end;
    }
    ret = dma_wait_transfer_complete(desc, chan);
_end:
    if (desc->release)
        desc->release(desc);
_free_ch:
    dma_release_channel(mdma_device, chan);
    return ret == 0? (int)size: ret;
}

ssize_t dma_memcpy_async(void *dst, const void *src, size_t size, 
    dma_transfer_cb_t cb, void *arg) {
    struct mdma_desc *desc;
    int chan, ret;

    chan = dma_request_channel(mdma_device, NULL);
    if (chan < 0) {
        printk("Request dma channel failed!\n");
        return -ENODEV;
    }
    desc = dma_memcpy_prepare(mdma_device, dst, src, size);
    if (!desc) {
        ret = -ENOMEM;
        goto _free_ch;
    }

    desc->head.dma_callback = cb;
    desc->head.user_data = arg;
    ret = dma_configure(desc->mdma, chan, &desc->head);
    if (ret) {
        ret = -EIO;
        goto _end;
    }
    ret = dma_start(desc->mdma, chan);
_end:
    if (desc->release)
        desc->release(desc);
_free_ch:
    dma_release_channel(mdma_device, chan);
    return ret == 0? (int)size: ret;
}

int dma_mdev_register(struct drvmgr_dev *mdma) {
    if (mdma) {
        mdma_device = mdma;
        return 0;
    }
    return -EINVAL;
}
