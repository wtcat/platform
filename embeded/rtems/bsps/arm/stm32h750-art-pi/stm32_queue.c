/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <stdlib.h>
#include <rtems/rtems/cache.h>
#include <rtems/malloc.h>

#include "drivers/dma.h"

#include "stm32/stm32_queue.h"

struct dma_circle_queue *dma_circle_queue_create(uint16_t size) {
    struct dma_circle_queue *cq;
    if (!size)
        return NULL;
    cq = rtems_malloc(sizeof(struct dma_circle_queue));
    if (cq) {
        cq->buffer = rtems_cache_coherent_allocate(size, 4, 0);
        if (!cq->buffer) {
            free(cq);
            return NULL;
        }
        cq->size = size;
        dma_circle_queue_reset(cq);
    }
    return cq;
}

int dma_circle_queue_delete(struct dma_circle_queue *cq) {
    if (cq == NULL)
        return -EINVAL;
    if (cq->buffer) {
        rtems_cache_coherent_free(cq->buffer);
        cq->buffer = NULL;
        free(cq);
    }
    return 0;
}

int dma_circle_queue_update(struct dma_circle_queue *cq, struct dma_chan *chan) {
    struct dma_status stat;
    uint16_t remain;
    uint16_t rxbytes;

    dma_get_status(chan->dev, chan->channel, &stat);
    remain = (uint16_t)stat.pending_length;
    if (cq->ndtr >= remain)
        rxbytes = cq->ndtr - remain;
    else
        rxbytes = cq->ndtr + cq->size - remain;
    cq->ndtr = remain;
    cq->count += remain;
    if (cq->count <= cq->size)
        cq->in = (cq->in + rxbytes) % cq->size;
    else
        return -EOVERFLOW;
    return 0;
}
