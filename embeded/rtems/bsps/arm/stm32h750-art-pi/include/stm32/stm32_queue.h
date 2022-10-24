/*
 * Copyright 2022 wtcat
 */
#ifndef STM32_QUEUE_H_
#define STM32_QUEUE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct dma_chan;
struct dma_circle_queue {
    uint16_t in;
    uint16_t out;
    uint16_t count;
    uint16_t size;
    uint16_t ndtr;
    char *buffer;
};

struct dma_circle_queue *dma_circle_queue_create(uint16_t size);
int dma_circle_queue_delete(struct dma_circle_queue *cq);

/* Called by DMA ISR */
int dma_circle_queue_update(struct dma_circle_queue *cq, struct dma_chan *chan);

static inline void dma_circle_queue_reset(struct dma_circle_queue *cq) {
    cq->in = cq->out = cq->count = 0;
    cq->ndtr = cq->size;
}

#ifdef __cplusplus
}
#endif
#endif /* STM32_QUEUE_H_ */
