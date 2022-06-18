/*
 * Copyright(c) 2004 - 2006 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called COPYING.
 */

/*
 * Copyright (c) 2022 wtcat
 */
 
#ifndef VIRT_DMA_H_
#define VIRT_DMA_H_

#include <errno.h>
#include <string.h>

#include <rtems/thread.h>
#include <rtems/score/assert.h>
#include <rtems/rtems/intr.h>
#include <rtems/bspIo.h>

#include "bsp/platform_bus.h"
#undef LIST_HEAD
#include "component/list.h"

#ifdef __cplusplus
extern "C"{
#endif

#ifdef DMAENGINE_DEBUG
#define dev_vdbg(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define dev_vdbg(...)
#endif


#define DMA_MIN_COOKIE 1
typedef uint16_t  dma_cookie_t;
typedef uintptr_t dma_addr_t;
struct dma_chan;

struct scatterlist {
	dma_addr_t dma_address;
	unsigned int length;
};

enum dmaengine_tx_result {
	DMA_TRANS_NOERROR = 0,		/* SUCCESS */
	DMA_TRANS_READ_FAILED,		/* Source DMA read failed */
	DMA_TRANS_WRITE_FAILED,		/* Destination DMA write failed */
	DMA_TRANS_ABORTED,		/* Op never submitted / aborted */
};

struct dmaengine_result {
	enum dmaengine_tx_result result;
};

/**
 * enum dma_status - DMA transaction status
 * @DMA_COMPLETE: transaction completed
 * @DMA_IN_PROGRESS: transaction not yet processed
 * @DMA_PAUSED: transaction is paused
 * @DMA_ERROR: transaction failed
 */
enum dma_status {
	DMA_COMPLETE,
	DMA_IN_PROGRESS,
	DMA_PAUSED,
	DMA_ERROR,
	DMA_OUT_OF_ORDER,
};

/**
 * enum dma_ctrl_flags - DMA flags to augment operation preparation,
 *  control completion, and communicate status.
 * @DMA_PREP_INTERRUPT - trigger an interrupt (callback) upon completion of
 *  this transaction
 * @DMA_CTRL_ACK - if clear, the descriptor cannot be reused until the client
 *  acknowledges receipt, i.e. has a chance to establish any dependency
 *  chains
 * @DMA_PREP_PQ_DISABLE_P - prevent generation of P while generating Q
 * @DMA_PREP_PQ_DISABLE_Q - prevent generation of Q while generating P
 * @DMA_PREP_CONTINUE - indicate to a driver that it is reusing buffers as
 *  sources that were the result of a previous operation, in the case of a PQ
 *  operation it continues the calculation with new sources
 * @DMA_PREP_FENCE - tell the driver that subsequent operations depend
 *  on the result of this operation
 * @DMA_CTRL_REUSE: client can reuse the descriptor and submit again till
 *  cleared or freed
 * @DMA_PREP_CMD: tell the driver that the data passed to DMA API is command
 *  data and the descriptor should be in different format from normal
 *  data descriptors.
 * @DMA_PREP_REPEAT: tell the driver that the transaction shall be automatically
 *  repeated when it ends until a transaction is issued on the same channel
 *  with the DMA_PREP_LOAD_EOT flag set. This flag is only applicable to
 *  interleaved transactions and is ignored for all other transaction types.
 * @DMA_PREP_LOAD_EOT: tell the driver that the transaction shall replace any
 *  active repeated (as indicated by DMA_PREP_REPEAT) transaction when the
 *  repeated transaction ends. Not setting this flag when the previously queued
 *  transaction is marked with DMA_PREP_REPEAT will cause the new transaction
 *  to never be processed and stay in the issued queue forever. The flag is
 *  ignored if the previous transaction is not a repeated transaction.
 */
enum dma_ctrl_flags {
	DMA_PREP_INTERRUPT = (1 << 0),
	DMA_CTRL_ACK = (1 << 1),
	DMA_PREP_PQ_DISABLE_P = (1 << 2),
	DMA_PREP_PQ_DISABLE_Q = (1 << 3),
	DMA_PREP_CONTINUE = (1 << 4),
	DMA_PREP_FENCE = (1 << 5),
	DMA_CTRL_REUSE = (1 << 6),
	DMA_PREP_CMD = (1 << 7),
	DMA_PREP_REPEAT = (1 << 8),
	DMA_PREP_LOAD_EOT = (1 << 9),
};

/**
 * enum dma_transfer_direction - dma transfer mode and direction indicator
 * @DMA_MEM_TO_MEM: Async/Memcpy mode
 * @DMA_MEM_TO_DEV: Slave mode & From Memory to Device
 * @DMA_DEV_TO_MEM: Slave mode & From Device to Memory
 * @DMA_DEV_TO_DEV: Slave mode & From Device to Device
 */
enum dma_transfer_direction {
	DMA_MEM_TO_MEM,
	DMA_MEM_TO_DEV,
	DMA_DEV_TO_MEM,
	DMA_DEV_TO_DEV,
	DMA_TRANS_NONE,
};

/**
 * Interleaved Transfer Request
 * ----------------------------
 * A chunk is collection of contiguous bytes to be transfered.
 * The gap(in bytes) between two chunks is called inter-chunk-gap(ICG).
 * ICGs may or maynot change between chunks.
 * A FRAME is the smallest series of contiguous {chunk,icg} pairs,
 *  that when repeated an integral number of times, specifies the transfer.
 * A transfer template is specification of a Frame, the number of times
 *  it is to be repeated and other per-transfer attributes.
 *
 * Practically, a client driver would have ready a template for each
 *  type of transfer it is going to need during its lifetime and
 *  set only 'src_start' and 'dst_start' before submitting the requests.
 *
 *
 *  |      Frame-1        |       Frame-2       | ~ |       Frame-'numf'  |
 *  |====....==.===...=...|====....==.===...=...| ~ |====....==.===...=...|
 *
 *    ==  Chunk size
 *    ... ICG
 */

/**
 * struct data_chunk - Element of scatter-gather list that makes a frame.
 * @size: Number of bytes to read from source.
 *	  size_dst := fn(op, size_src), so doesn't mean much for destination.
 * @icg: Number of bytes to jump after last src/dst address of this
 *	 chunk and before first src/dst address for next chunk.
 *	 Ignored for dst(assumed 0), if dst_inc is true and dst_sgl is false.
 *	 Ignored for src(assumed 0), if src_inc is true and src_sgl is false.
 * @dst_icg: Number of bytes to jump after last dst address of this
 *	 chunk and before the first dst address for next chunk.
 *	 Ignored if dst_inc is true and dst_sgl is false.
 * @src_icg: Number of bytes to jump after last src address of this
 *	 chunk and before the first src address for next chunk.
 *	 Ignored if src_inc is true and src_sgl is false.
 */
struct data_chunk {
	size_t size;
	size_t icg;
	size_t dst_icg;
	size_t src_icg;
};

/**
 * struct dma_interleaved_template - Template to convey DMAC the transfer pattern
 *	 and attributes.
 * @src_start: Bus address of source for the first chunk.
 * @dst_start: Bus address of destination for the first chunk.
 * @dir: Specifies the type of Source and Destination.
 * @src_inc: If the source address increments after reading from it.
 * @dst_inc: If the destination address increments after writing to it.
 * @src_sgl: If the 'icg' of sgl[] applies to Source (scattered read).
 *		Otherwise, source is read contiguously (icg ignored).
 *		Ignored if src_inc is false.
 * @dst_sgl: If the 'icg' of sgl[] applies to Destination (scattered write).
 *		Otherwise, destination is filled contiguously (icg ignored).
 *		Ignored if dst_inc is false.
 * @numf: Number of frames in this template.
 * @frame_size: Number of chunks in a frame i.e, size of sgl[].
 * @sgl: Array of {chunk,icg} pairs that make up a frame.
 */
struct dma_interleaved_template {
	dma_addr_t src_start;
	dma_addr_t dst_start;
	enum dma_transfer_direction dir;
	bool src_inc;
	bool dst_inc;
	bool src_sgl;
	bool dst_sgl;
	size_t numf;
	size_t frame_size;
	struct data_chunk sgl[0];
};


/**
 * enum dma_slave_buswidth - defines bus width of the DMA slave
 * device, source or target buses
 */
enum dma_slave_buswidth {
	DMA_SLAVE_BUSWIDTH_UNDEFINED = 0,
	DMA_SLAVE_BUSWIDTH_1_BYTE = 1,
	DMA_SLAVE_BUSWIDTH_2_BYTES = 2,
	DMA_SLAVE_BUSWIDTH_3_BYTES = 3,
	DMA_SLAVE_BUSWIDTH_4_BYTES = 4,
	DMA_SLAVE_BUSWIDTH_8_BYTES = 8,
	DMA_SLAVE_BUSWIDTH_16_BYTES = 16,
	DMA_SLAVE_BUSWIDTH_32_BYTES = 32,
	DMA_SLAVE_BUSWIDTH_64_BYTES = 64,
};

/**
 * struct dma_tx_state - filled in to report the status of
 * a transfer.
 * @last: last completed DMA cookie
 * @used: last issued DMA cookie (i.e. the one in progress)
 * @residue: the remaining number of bytes left to transmit
 *	on the selected transfer for states DMA_IN_PROGRESS and
 *	DMA_PAUSED if this is implemented in the driver, else 0
 * @in_flight_bytes: amount of data in bytes cached by the DMA.
 */
struct dma_tx_state {
	dma_cookie_t last;
	dma_cookie_t used;
	uint32_t residue;
	uint32_t in_flight_bytes;
};

/**
 * struct dma_slave_map - associates slave device and it's slave channel with
 * parameter to be used by a filter function
 * @slave: slave channel name
 * @param: opaque parameter to pass to struct dma_filter.fn
 */
struct dma_slave_map {
	const char *slave;
	void *param;
};

#define DMA_SLAVE_MAP(_name, _param) \
	{.slave = _name, .param = (void *)(_param)}

/**
 * struct dma_filter - information for slave device/channel to filter_fn/param
 * mapping
 * @fn: filter function callback
 * @mapcnt: number of slave device/channel in the map
 * @map: array of channel to filter mapping data
 */
struct dma_filter {
	bool (*fn)(struct dma_chan *chan, void *param);
	int mapcnt;
	const struct dma_slave_map *map;
};

/**
 * struct dma_slave_config - dma slave channel runtime config
 * @direction: whether the data shall go in or out on this slave
 * channel, right now. DMA_MEM_TO_DEV and DMA_DEV_TO_MEM are
 * legal values. DEPRECATED, drivers should use the direction argument
 * to the device_prep_slave_sg and device_prep_dma_cyclic functions or
 * the dir field in the dma_interleaved_template structure.
 * @src_addr: this is the physical address where DMA slave data
 * should be read (RX), if the source is memory this argument is
 * ignored.
 * @dst_addr: this is the physical address where DMA slave data
 * should be written (TX), if the source is memory this argument
 * is ignored.
 * @src_addr_width: this is the width in bytes of the source (RX)
 * register where DMA data shall be read. If the source
 * is memory this may be ignored depending on architecture.
 * Legal values: 1, 2, 3, 4, 8, 16, 32, 64.
 * @dst_addr_width: same as src_addr_width but for destination
 * target (TX) mutatis mutandis.
 * @src_maxburst: the maximum number of words (note: words, as in
 * units of the src_addr_width member, not bytes) that can be sent
 * in one burst to the device. Typically something like half the
 * FIFO depth on I/O peripherals so you don't overflow it. This
 * may or may not be applicable on memory sources.
 * @dst_maxburst: same as src_maxburst but for destination target
 * mutatis mutandis.
 * @src_port_window_size: The length of the register area in words the data need
 * to be accessed on the device side. It is only used for devices which is using
 * an area instead of a single register to receive the data. Typically the DMA
 * loops in this area in order to transfer the data.
 * @dst_port_window_size: same as src_port_window_size but for the destination
 * port.
 * @device_fc: Flow Controller Settings. Only valid for slave channels. Fill
 * with 'true' if peripheral should be flow controller. Direction will be
 * selected at Runtime.
 * @slave_id: Slave requester id. Only valid for slave channels. The dma
 * slave peripheral will have unique id as dma requester which need to be
 * pass as slave config.
 * @peripheral_config: peripheral configuration for programming peripheral
 * for dmaengine transfer
 * @peripheral_size: peripheral configuration buffer size
 *
 * This struct is passed in as configuration data to a DMA engine
 * in order to set up a certain channel for DMA transport at runtime.
 * The DMA device/engine has to provide support for an additional
 * callback in the dma_device structure, device_config and this struct
 * will then be passed in as an argument to the function.
 *
 * The rationale for adding configuration information to this struct is as
 * follows: if it is likely that more than one DMA slave controllers in
 * the world will support the configuration option, then make it generic.
 * If not: if it is fixed so that it be sent in static from the platform
 * data, then prefer to do that.
 */
struct dma_slave_config {
	dma_addr_t src_addr;
	dma_addr_t dst_addr;
	enum dma_transfer_direction direction;
	enum dma_slave_buswidth src_addr_width;
	enum dma_slave_buswidth dst_addr_width;
	uint32_t src_maxburst;
	uint32_t dst_maxburst;
	uint32_t src_port_window_size;
	uint32_t dst_port_window_size;
	bool device_fc;
	unsigned int slave_id;
	void *peripheral_config;
	size_t peripheral_size;
};

struct dma_device {
	struct dma_filter filter;
	unsigned int chancnt;
	uint32_t max_burst;

	/* protected by dma_device.lock */
	struct list_head channels;
	rtems_mutex lock;
	int (*device_alloc_chan_resources)(struct dma_chan *chan);
	void (*device_free_chan_resources)(struct dma_chan *chan);
	struct dma_async_tx_descriptor *(*device_prep_dma_memcpy)(
		struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
		size_t len, unsigned long flags);
	struct dma_async_tx_descriptor *(*device_prep_dma_memset)(
		struct dma_chan *chan, dma_addr_t dest, int value, size_t len,
		unsigned long flags);
	struct dma_async_tx_descriptor *(*device_prep_slave_sg)(
		struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context);
	struct dma_async_tx_descriptor *(*device_prep_dma_cyclic)(
		struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
		size_t period_len, enum dma_transfer_direction direction,
		unsigned long flags);
	struct dma_async_tx_descriptor *(*device_prep_interleaved_dma)(
		struct dma_chan *chan, struct dma_interleaved_template *xt,
		unsigned long flags);

	int (*device_config)(struct dma_chan *chan, struct dma_slave_config *config);
	int (*device_pause)(struct dma_chan *chan);
	int (*device_resume)(struct dma_chan *chan);
	int (*device_terminate_all)(struct dma_chan *chan);
	void (*device_synchronize)(struct dma_chan *chan);
	enum dma_status (*device_tx_status)(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate);
	void (*device_issue_pending)(struct dma_chan *chan);
};


struct dma_chan {
	struct dma_device *device;
	const char *name;
	struct list_head node;
	dma_cookie_t cookie;
	dma_cookie_t completed_cookie;
	uint16_t refcnt;
};

/**
 * struct dma_async_tx_descriptor - async transaction descriptor
 * ---dma generic offload fields---
 * @cookie: tracking cookie for this transaction, set to -EBUSY if
 *	this tx is sitting on a dependency list
 * @flags: flags to augment operation preparation, control completion, and
 *	communicate status
 * @phys: physical address of the descriptor
 * @chan: target channel for this operation
 * @tx_submit: accept the descriptor, assign ordered cookie and mark the
 * descriptor pending. To be pushed on .issue_pending() call
 * @callback: routine to call after this operation is complete
 * @callback_param: general parameter to pass to the callback routine
 * @desc_metadata_mode: core managed metadata mode to protect mixed use of
 *	DESC_METADATA_CLIENT or DESC_METADATA_ENGINE. Otherwise
 *	DESC_METADATA_NONE
 * @metadata_ops: DMA driver provided metadata mode ops, need to be set by the
 *	DMA driver if metadata mode is supported with the descriptor
 * ---async_tx api specific fields---
 * @next: at completion submit this descriptor
 * @parent: pointer to the next level up in the dependency chain
 * @lock: protect the parent and next pointers
 */
struct dma_async_tx_descriptor {
	dma_cookie_t cookie;
	uint16_t flags;
	struct dma_chan *chan;
	void (*callback)(void *param);
	void *callback_param;
};

struct virt_dma_desc {
	struct dma_async_tx_descriptor tx;
	struct dmaengine_result tx_result;
	/* protected by vc.lock */
	struct list_head node;
};

struct virt_dma_chan {
	struct dma_chan	chan;
	rtems_interrupt_lock lock;

	/* protected by vc.lock */
	struct list_head desc_allocated;
	struct list_head desc_submitted;
	struct list_head desc_terminated;
	void (*desc_free)(struct virt_dma_desc *);
};

static inline struct virt_dma_chan *to_virt_chan(struct dma_chan *chan) {
	return container_of(chan, struct virt_dma_chan, chan);
}

static inline struct virt_dma_desc *to_virt_desc(struct dma_async_tx_descriptor *tx) {
	return container_of(tx, struct virt_dma_desc, tx);
}

static inline void dmaengine_desc_notify(struct dma_async_tx_descriptor *tx) {
	if (tx->callback)
		tx->callback(tx->callback_param);
}

static inline void dmaengine_desc_clear_reuse(struct dma_async_tx_descriptor *tx) {
	tx->flags &= ~DMA_CTRL_REUSE;
}

static inline bool dmaengine_desc_test_reuse(struct dma_async_tx_descriptor *tx) {
	return (tx->flags & DMA_CTRL_REUSE) == DMA_CTRL_REUSE;
}

static inline void vchan_dma_desc_free_list(struct virt_dma_chan *vc, 
	struct list_head *head) {
	struct virt_dma_desc *vd, *_vd;
	list_for_each_entry_safe(vd, _vd, head, node) {
		if (dmaengine_desc_test_reuse(&vd->tx)) {
			list_move_tail(&vd->node, &vc->desc_allocated);
		} else {
			dev_vdbg("txd %p: freeing\n", vd);
			list_del(&vd->node);
			vc->desc_free(vd);
		}
	}
}

static inline struct virt_dma_desc *vchan_desc_reclaim(struct virt_dma_chan *vc) {
	struct virt_dma_desc *vd = NULL;
	rtems_interrupt_lock_context flags;
	rtems_interrupt_lock_acquire(&vc->lock, &flags);
	if (!list_empty(&vc->desc_terminated)) {
		vd = container_of(vc->desc_terminated.next, struct virt_dma_desc, node);
		list_del(&vd->node);
	}
	rtems_interrupt_lock_release(&vc->lock, &flags);
	return vd;
}

/**
 * dma_cookie_init - initialize the cookies for a DMA channel
 * @chan: dma channel to initialize
 */
static inline void dma_cookie_init(struct dma_chan *chan) {
	chan->cookie = DMA_MIN_COOKIE;
}

/**
 * dma_cookie_assign - assign a DMA engine cookie to the descriptor
 * @tx: descriptor needing cookie
 *
 * Assign a unique non-zero per-channel cookie to the descriptor.
 * Note: caller is expected to hold a lock to prevent concurrency.
 */
static inline dma_cookie_t dma_cookie_assign(struct dma_async_tx_descriptor *tx) {
	struct dma_chan *chan = tx->chan;
	dma_cookie_t cookie;
	cookie = chan->cookie + 1;
	if (cookie < DMA_MIN_COOKIE)
		cookie = DMA_MIN_COOKIE;
	tx->cookie = chan->cookie = cookie;
	return cookie;
}

/**
 * vchan_vdesc_fini - Free or reuse a descriptor
 * @vd: virtual descriptor to free/reuse
 */
static inline void vchan_vdesc_fini(struct virt_dma_chan *vc,
	struct virt_dma_desc *vd) {
	if (dmaengine_desc_test_reuse(&vd->tx))
		list_move(&vd->node, &vc->desc_allocated);
	else
		list_move(&vd->node, &vc->desc_terminated);
}

/**
 * vchan_tx_desc_free - free a reusable descriptor
 * @tx: the transfer
 *
 * This function frees a previously allocated reusable descriptor. The only
 * other way is to clear the DMA_CTRL_REUSE flag and submit one last time the
 * transfer.
 *
 * Returns 0 upon success
 */
static inline int vchan_tx_desc_free(struct dma_async_tx_descriptor *tx) {
	struct virt_dma_chan *vc = to_virt_chan(tx->chan);
	struct virt_dma_desc *vd = to_virt_desc(tx);
	rtems_interrupt_lock_context flags;
	rtems_interrupt_lock_acquire(&vc->lock, &flags);
	list_del(&vd->node);
	rtems_interrupt_lock_release(&vc->lock, &flags);
	dev_vdbg("vchan %p: txd %p[%x]: freeing\n", vc, vd, vd->tx.cookie);
	vc->desc_free(vd);
	return 0;
}

static inline void vchan_init(struct virt_dma_chan *vc, struct dma_device *dmadev) {
	dma_cookie_init(&vc->chan);
	rtems_interrupt_lock_initialize(&vc->lock, "VDMA");
	INIT_LIST_HEAD(&vc->desc_allocated);
	INIT_LIST_HEAD(&vc->desc_submitted);
	INIT_LIST_HEAD(&vc->desc_terminated);
	vc->chan.device = dmadev;
	list_add_tail(&vc->chan.node, &dmadev->channels);
}

static inline struct virt_dma_desc *vchan_find_desc(struct virt_dma_chan *vc,
	dma_cookie_t cookie) {
	struct virt_dma_desc *vd;
	list_for_each_entry(vd, &vc->desc_submitted, node)
		if (vd->tx.cookie == cookie)
			return vd;
	return NULL;
}

static inline dma_cookie_t vchan_tx_submit(struct dma_async_tx_descriptor *tx) {
	struct virt_dma_chan *vc = to_virt_chan(tx->chan);
	struct virt_dma_desc *vd = to_virt_desc(tx);
	rtems_interrupt_lock_context flags;
	rtems_interrupt_lock_acquire(&vc->lock, &flags);
	dma_cookie_t cookie = dma_cookie_assign(tx);
	list_move_tail(&vd->node, &vc->desc_submitted);
	rtems_interrupt_lock_release(&vc->lock, &flags);
	dev_vdbg("vchan %p: txd %p[%x]: submitted\n", vc, vd, cookie);
	return cookie;
}
	
static inline void dma_async_tx_descriptor_init(struct dma_async_tx_descriptor *tx,
	struct dma_chan *chan) {
	tx->chan = chan;
}

/**
 * vchan_tx_prep - prepare a descriptor
 * @vc: virtual channel allocating this descriptor
 * @vd: virtual descriptor to prepare
 * @tx_flags: flags argument passed in to prepare function
 */
static inline struct dma_async_tx_descriptor *vchan_tx_prep(struct virt_dma_chan *vc,
	struct virt_dma_desc *vd, unsigned long tx_flags) {
	rtems_interrupt_lock_context flags;
	dma_async_tx_descriptor_init(&vd->tx, &vc->chan);
	vd->tx.flags = tx_flags;
	vd->tx_result.result = DMA_TRANS_NOERROR;
	rtems_interrupt_lock_acquire(&vc->lock, &flags);
	list_add_tail(&vd->node, &vc->desc_allocated);
	rtems_interrupt_lock_release(&vc->lock, &flags);
	return &vd->tx;
}

/**
 * vchan_issue_pending - move submitted descriptors to issued list
 * @vc: virtual channel to update
 *
 * vc.lock must be held by caller
 */
static inline bool vchan_issue_pending(struct virt_dma_chan *vc) {
	return !list_empty(&vc->desc_submitted);
}

/**
 * vchan_terminate_vdesc - Disable pending cyclic callback
 * @vd: virtual descriptor to be terminated
 *
 * vc.lock must be held by caller
 */
static inline void vchan_terminate_vdesc(struct virt_dma_desc *vd) {
	struct virt_dma_chan *vc = to_virt_chan(vd->tx.chan);
	vchan_vdesc_fini(vc, vd);
}

/**
 * vchan_next_desc - peek at the next descriptor to be processed
 * @vc: virtual channel to obtain descriptor from
 *
 * vc.lock must be held by caller
 */
static inline struct virt_dma_desc *vchan_next_desc(struct virt_dma_chan *vc) {
	return list_first_entry_or_null(&vc->desc_submitted,
					struct virt_dma_desc, node);
}

/**
 * vchan_get_all_descriptors - obtain all submitted and issued descriptors
 * @vc: virtual channel to get descriptors from
 * @head: list of descriptors found
 *
 * vc.lock must be held by caller
 *
 * Removes all submitted and issued descriptors from internal lists, and
 * provides a list of all descriptors found
 */
static inline void vchan_get_all_descriptors(struct virt_dma_chan *vc,
	struct list_head *head) {
	list_splice_tail_init(&vc->desc_allocated, head);
	list_splice_tail_init(&vc->desc_submitted, head);
	list_splice_tail_init(&vc->desc_terminated, head);
}

static inline void vchan_free_chan_resources(struct virt_dma_chan *vc) {
	struct virt_dma_desc *vd;
	rtems_interrupt_lock_context flags;
	LIST_HEAD(head);
	rtems_interrupt_lock_acquire(&vc->lock, &flags);
	vchan_get_all_descriptors(vc, &head);
	list_for_each_entry(vd, &head, node) {
		dmaengine_desc_clear_reuse(&vd->tx);
	}
	rtems_interrupt_lock_release(&vc->lock, &flags);
	vchan_dma_desc_free_list(vc, &head);
}

/**
 * vchan_synchronize() - synchronize callback execution to the current context
 * @vc: virtual channel to synchronize
 *
 * Makes sure that all scheduled or active callbacks have finished running. For
 * proper operation the caller has to ensure that no new callbacks are scheduled
 * after the invocation of this function started.
 * Free up the terminated cyclic descriptor to prevent memory leakage.
 */
static inline void vchan_synchronize(struct virt_dma_chan *vc) {
	LIST_HEAD(head);
	rtems_interrupt_lock_context flags;
	rtems_interrupt_lock_acquire(&vc->lock, &flags);
	vchan_get_all_descriptors(vc, &head);
	list_splice_tail_init(&vc->desc_terminated, &head);
	rtems_interrupt_lock_release(&vc->lock, &flags);
	vchan_dma_desc_free_list(vc, &head);
}

/**
 * vchan_cyclic_callback - report the completion of a period
 * @vd: virtual descriptor
 */
static inline void vchan_cyclic_callback(struct virt_dma_desc *vd) {
	dmaengine_desc_notify(&vd->tx);
}

/**
 * dma_cookie_complete - complete a descriptor
 * @tx: descriptor to complete
 *
 * Mark this descriptor complete by updating the channels completed
 * cookie marker.  Zero the descriptors cookie to prevent accidental
 * repeated completions.
 *
 * Note: caller is expected to hold a lock to prevent concurrency.
 */
static inline void dma_cookie_complete(struct dma_async_tx_descriptor *tx) {
	_Assert(tx->cookie >= DMA_MIN_COOKIE);
	tx->chan->completed_cookie = tx->cookie;
	tx->cookie = 0;
}

/**
 * dma_async_is_complete - test a cookie against chan state
 * @cookie: transaction identifier to test status of
 * @last_complete: last know completed transaction
 * @last_used: last cookie value handed out
 *
 * dma_async_is_complete() is used in dma_async_is_tx_complete()
 * the test logic is separated for lightweight testing of multiple cookies
 */
static inline enum dma_status dma_async_is_complete(dma_cookie_t cookie,
	dma_cookie_t last_complete, dma_cookie_t last_used) {
	if (last_complete <= last_used) {
		if ((cookie <= last_complete) || (cookie > last_used))
			return DMA_COMPLETE;
	} else {
		if ((cookie <= last_complete) && (cookie > last_used))
			return DMA_COMPLETE;
	}
	return DMA_IN_PROGRESS;
}
			
/**
 * vchan_cookie_complete - report completion of a descriptor
 * @vd: virtual descriptor to update
 *
 * vc.lock must be held by caller
 */
static inline void vchan_cookie_complete(struct virt_dma_desc *vd) {
	struct virt_dma_chan *vc = to_virt_chan(vd->tx.chan);
	dev_vdbg("txd %p[%x]: marked complete\n", vd, vd->tx.cookie);
	dma_cookie_complete(&vd->tx);
	vchan_vdesc_fini(vc, vd);
	dmaengine_desc_notify(&vd->tx);
}

static inline size_t dmaengine_get_icg(bool inc, bool sgl, size_t icg,
	size_t dir_icg) {
	if (inc) {
		if (dir_icg)
			return dir_icg;
		else if (sgl)
			return icg;
	}
	return 0;
}

static inline size_t dmaengine_get_dst_icg(struct dma_interleaved_template *xt,
	struct data_chunk *chunk) {
	return dmaengine_get_icg(xt->dst_inc, xt->dst_sgl, chunk->icg, chunk->dst_icg);
}

static inline size_t dmaengine_get_src_icg(struct dma_interleaved_template *xt,
	struct data_chunk *chunk) {
	return dmaengine_get_icg(xt->src_inc, xt->src_sgl, chunk->icg, chunk->src_icg);			 
}

static inline int dmaengine_slave_config(struct dma_chan *chan,
	struct dma_slave_config *config) {
	_Assert(chan != NULL);
	_Assert(chan->device != NULL);
	_Assert(chan->device->device_config != NULL);
	return chan->device->device_config(chan, config);
}

static inline struct dma_async_tx_descriptor *dmaengine_prep_slave_single(
	struct dma_chan *chan, dma_addr_t buf, size_t len,
	enum dma_transfer_direction dir, unsigned long flags) {
	_Assert(chan != NULL);
	_Assert(chan->device != NULL);
	_Assert(chan->device->device_prep_slave_sg != NULL);
	struct scatterlist sg;
	sg.dma_address = buf;
	sg.length = len;
	return chan->device->device_prep_slave_sg(chan, &sg, 1,
						  dir, flags, NULL);
}

static inline struct dma_async_tx_descriptor *dmaengine_prep_slave_sg(
	struct dma_chan *chan, struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction dir, unsigned long flags) {
	_Assert(chan != NULL);
	_Assert(chan->device != NULL);
	_Assert(chan->device->device_prep_slave_sg != NULL);
	return chan->device->device_prep_slave_sg(chan, sgl, sg_len,
						  dir, flags, NULL);
}

static inline struct dma_async_tx_descriptor *dmaengine_prep_dma_cyclic(
	struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction dir,
	unsigned long flags) {
	_Assert(chan != NULL);
	_Assert(chan->device != NULL);
	_Assert(chan->device->device_prep_dma_cyclic != NULL);
	return chan->device->device_prep_dma_cyclic(chan, buf_addr, buf_len,
						period_len, dir, flags);
}

static inline struct dma_async_tx_descriptor *dmaengine_prep_dma_memset(
	struct dma_chan *chan, dma_addr_t dest, int value, size_t len,
	unsigned long flags) {
	_Assert(chan != NULL);
	_Assert(chan->device != NULL);
	_Assert(chan->device->device_prep_dma_memset != NULL);
	return chan->device->device_prep_dma_memset(chan, dest, value,
							len, flags);
}

static inline struct dma_async_tx_descriptor *dmaengine_prep_dma_memcpy(
	struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
	size_t len, unsigned long flags) {
	_Assert(chan != NULL);
	_Assert(chan->device != NULL);
	_Assert(chan->device->device_prep_dma_memcpy != NULL);
	return chan->device->device_prep_dma_memcpy(chan, dest, src,
							len, flags);
}

static inline int dmaengine_pause(struct dma_chan *chan) {
	if (chan->device->device_pause)
		return chan->device->device_pause(chan);
	return -ENOSYS;
}

static inline int dmaengine_resume(struct dma_chan *chan) {
	if (chan->device->device_resume)
		return chan->device->device_resume(chan);
	return -ENOSYS;
}

static inline enum dma_status dmaengine_tx_status(struct dma_chan *chan,
	dma_cookie_t cookie, struct dma_tx_state *state) {
	return chan->device->device_tx_status(chan, cookie, state);
}

static inline dma_cookie_t dmaengine_submit(struct dma_async_tx_descriptor *desc) {
	_Assert(desc != NULL);
	return vchan_tx_submit(desc);
}

static inline void dma_async_issue_pending(struct dma_chan *chan) {
	_Assert(chan != NULL);
	_Assert(chan->device != NULL);
	_Assert(chan->device->device_issue_pending != NULL);
	chan->device->device_issue_pending(chan);
}

static inline int dmaengine_terminate_async(struct dma_chan *chan) {
	if (chan->device->device_terminate_all)
		return chan->device->device_terminate_all(chan);
	return -EINVAL;
}

static inline int dmaengine_terminate_all(struct dma_chan *chan) {
	if (chan->device->device_terminate_all)
		return chan->device->device_terminate_all(chan);
	return -ENOSYS;
}

static inline void dmaengine_synchronize(struct dma_chan *chan) {
	if (chan->device->device_synchronize)
		chan->device->device_synchronize(chan);
}

static inline int dmaengine_terminate_sync(struct dma_chan *chan) {
	int ret = dmaengine_terminate_async(chan);
	if (ret)
		return ret;
	dmaengine_synchronize(chan);
	return 0;
}

static inline enum dma_status dma_cookie_status(struct dma_chan *chan,
	dma_cookie_t cookie, struct dma_tx_state *state) {
	dma_cookie_t used, complete;
	used = chan->cookie;
	complete = chan->completed_cookie;
	barrier();
	if (state) {
		state->last = complete;
		state->used = used;
		state->residue = 0;
		state->in_flight_bytes = 0;
	}
	return dma_async_is_complete(cookie, complete, used);
}

static inline bool is_slave_direction(enum dma_transfer_direction direction) {
	return (direction == DMA_MEM_TO_DEV) || (direction == DMA_DEV_TO_MEM);
}
							
struct dma_chan *dma_request_chan(struct drvmgr_dev *dev, 
	const char *name);
void dma_release_chan(struct dma_chan *chan);

#ifdef __cplusplus
}
#endif
#endif /* VIRT_DMA_H_ */
