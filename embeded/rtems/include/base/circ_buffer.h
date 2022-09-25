/*
 * Copyright 2022  wtcat
 */
#ifndef BASE_CIRC_BUFFER_H_
#define BASE_CIRC_BUFFER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif

struct circ_buffer {
	int head;
	int tail;
    union {
	    uint8_t  b_buf[0];
        uint16_t h_buf[0];
        uint32_t w_buf[0];
    };
#define TO_CIRC(buf) ((struct circ_buffer *)(buf))
#define CIRC_BSIZE(size) (sizeof(struct circ_buffer) + (size))
#define CIRC_CNT(head, tail, size) (((head) - (tail)) & ((size)-1))
#define CIRC_SPACE(head, tail, size) CIRC_CNT((tail),((head)+1),(size))
#define CIRC_CNT_TO_END(head, tail, size) \
	({int end = (size) - (tail); \
	  int n = ((head) + end) & ((size)-1); \
	  n < end ? n : end;})
#define CIRC_SPACE_TO_END(head, tail, size) \
	({int end = (size) - 1 - (head); \
	  int n = (end + (tail)) & ((size)-1); \
	  n <= end ? n : end+1;})
};

static inline void circ_buffer_reset(struct circ_buffer *cbuf) {
    cbuf->head = cbuf->tail = 0;
}

#ifdef __cplusplus
}
#endif
#endif /* BASE_CIRC_BUFFER_H_ */
