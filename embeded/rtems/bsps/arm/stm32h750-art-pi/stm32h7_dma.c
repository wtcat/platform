/*
 * Copyright 2022 wtcat
 */
/*
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2019 Song Qiang <songqiang1304521@gmail.com>
 * Copyright (c) 2022 wtcat
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Common part of DMA drivers for stm32.
 * @note  Functions named with stm32_dma_* are SoCs related functions
 *        implemented in dma_stm32_v*.c
 */
#include <errno.h>
#include <stdlib.h>
#include <rtems/bspIo.h>
#include <rtems/malloc.h>

#include "base/macros.h"
#include "drivers/dma.h"
#include "drivers/clock.h"
#include "drivers/ofw_platform_bus.h"

#include "stm32/stm32_clock.h"
#include "stm32/stm32_dma.h"
#include "stm32/stm32_com.h"

#define DMA_STREAM_COUNT 8
#define STREAM_OFFSET 1
#define STM32_DMA_HAL_OVERRIDE      0x7F

#define LOG_ERR(fmt, ...) printk(fmt, ##__VA_ARGS__)
#define LOG_WRN LOG_ERR

struct stm32_dma {
    DMA_TypeDef *dma;
    struct drvmgr_dev *clk;
    int clkid;
    bool support_m2m;
	uint32_t max_streams;
	long channels;
#ifdef CONFIG_DMAMUX_STM32
	uint8_t offset; /* position in the list of dmamux channel list */
#endif
	struct dma_stm32_stream streams[DMA_STREAM_COUNT];
};

static uint32_t table_m_size[] = {
	LL_DMA_MDATAALIGN_BYTE,
	LL_DMA_MDATAALIGN_HALFWORD,
	LL_DMA_MDATAALIGN_WORD,
};

static uint32_t table_p_size[] = {
	LL_DMA_PDATAALIGN_BYTE,
	LL_DMA_PDATAALIGN_HALFWORD,
	LL_DMA_PDATAALIGN_WORD,
};

static void _stm32_dma_dump_stream_irq(struct drvmgr_dev *dev, uint32_t id) {
	struct stm32_dma *priv = dev->priv;
	stm32_dma_dump_stream_irq(priv->dma, id);
}

static void _stm32_dma_clear_stream_irq(struct drvmgr_dev *dev, uint32_t id) {
	struct stm32_dma *priv = dev->priv;
	dma_stm32_clear_tc(priv->dma, id);
	dma_stm32_clear_ht(priv->dma, id);
	stm32_dma_clear_stream_irq(priv->dma, id);
}

static void stm32_dma_irq_handler(struct drvmgr_dev *dev, uint32_t id) {
	struct stm32_dma *priv = dev->priv;
	DMA_TypeDef *dma = priv->dma;
	struct dma_stm32_stream *stream;
	uint32_t callback_arg;

	_Assert(id < priv->max_streams);
	stream = &priv->streams[id];
#ifdef CONFIG_DMAMUX_STM32
	callback_arg = stream->mux_channel;
#else
	callback_arg = id + STREAM_OFFSET;
#endif /* CONFIG_DMAMUX_STM32 */

	if (!IS_ENABLED(CONFIG_DMAMUX_STM32)) {
		stream->busy = false;
	}

	/* the dma stream id is in range from STREAM_OFFSET..<dma-requests> */
	if (stm32_dma_is_ht_irq_active(dma, id)) {
		/* Let HAL DMA handle flags on its own */
		if (!stream->hal_override) {
			dma_stm32_clear_ht(dma, id);
		}
		stream->dma_callback(dev, stream->user_data, callback_arg, 0);
	} else if (stm32_dma_is_tc_irq_active(dma, id)) {
#ifdef CONFIG_DMAMUX_STM32
		stream->busy = false;
#endif
		/* Let HAL DMA handle flags on its own */
		if (!stream->hal_override) {
			dma_stm32_clear_tc(dma, id);
		}
		stream->dma_callback(dev, stream->user_data, callback_arg, 0);
	} else if (stm32_dma_is_unexpected_irq_happened(dma, id)) {
		LOG_ERR("Unexpected irq happened.");
		stream->dma_callback(dev, stream->user_data,
				     callback_arg, -EIO);
	} else {
		LOG_ERR("Transfer Error.");
		_stm32_dma_dump_stream_irq(dev, id);
		_stm32_dma_clear_stream_irq(dev, id);
		stream->dma_callback(dev, stream->user_data,
				     callback_arg, -EIO);
	}
}

static int stm32_dma_get_priority(uint8_t priority, uint32_t *ll_priority) {
	switch (priority) {
	case 0x0:
		*ll_priority = LL_DMA_PRIORITY_LOW;
		break;
	case 0x1:
		*ll_priority = LL_DMA_PRIORITY_MEDIUM;
		break;
	case 0x2:
		*ll_priority = LL_DMA_PRIORITY_HIGH;
		break;
	case 0x3:
		*ll_priority = LL_DMA_PRIORITY_VERYHIGH;
		break;
	default:
		LOG_ERR("Priority error. %d", priority);
		return -EINVAL;
	}
	return 0;
}

static int stm32_dma_get_direction(enum dma_channel_direction direction,
	uint32_t *ll_direction) {
	switch (direction) {
	case MEMORY_TO_MEMORY:
		*ll_direction = LL_DMA_DIRECTION_MEMORY_TO_MEMORY;
		break;
	case MEMORY_TO_PERIPHERAL:
		*ll_direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
		break;
	case PERIPHERAL_TO_MEMORY:
		*ll_direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
		break;
	default:
		LOG_ERR("Direction error. %d", direction);
		return -EINVAL;
	}
	return 0;
}

static int stm32_dma_get_memory_increment(enum dma_addr_adj increment,
	uint32_t *ll_increment) {
	switch (increment) {
	case DMA_ADDR_ADJ_INCREMENT:
		*ll_increment = LL_DMA_MEMORY_INCREMENT;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		*ll_increment = LL_DMA_MEMORY_NOINCREMENT;
		break;
	case DMA_ADDR_ADJ_DECREMENT:
		return -ENOTSUP;
	default:
		LOG_ERR("Memory increment error. %d", increment);
		return -EINVAL;
	}
	return 0;
}

static int stm32_dma_get_periph_increment(enum dma_addr_adj increment,
    uint32_t *ll_increment) {
	switch (increment) {
	case DMA_ADDR_ADJ_INCREMENT:
		*ll_increment = LL_DMA_PERIPH_INCREMENT;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		*ll_increment = LL_DMA_PERIPH_NOINCREMENT;
		break;
	case DMA_ADDR_ADJ_DECREMENT:
		return -ENOTSUP;
	default:
		LOG_ERR("Periph increment error. %d", increment);
		return -EINVAL;
	}
	return 0;
}

DMA_STM32_EXPORT_API int stm32_dma_disable_stream(DMA_TypeDef *dma, uint32_t id) {
	int count = 0;
	while (true) {
		if (_stm32_dma_disable_stream(dma, id) == 0)
			return 0;
		/* After trying for 5 seconds, give up */
		if (count++ > (5 * 1000)) 
			return -EBUSY;
		rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(1));
	}
	return 0;
}

DMA_STM32_EXPORT_API int dma_stm32_configure(struct drvmgr_dev *dev, uint32_t id,
    struct dma_config *config) {
	struct stm32_dma *priv = dev->priv;
	struct dma_stm32_stream *stream =
				&priv->streams[id - STREAM_OFFSET];
	DMA_TypeDef *dma = priv->dma;
	LL_DMA_InitTypeDef DMA_InitStruct = {0};
	int ret;

	/* give channel from index 0 */
	id = id - STREAM_OFFSET;

	/* Check potential DMA override */
	if (config->linked_channel == STM32_DMA_HAL_OVERRIDE) {
		/* DMA channel is overridden by HAL DMA
		 * Retain that the channel is busy and proceed to the minimal
		 * configuration to properly route the IRQ
		 */
		stream->busy = true;
		stream->hal_override = true;
		stream->dma_callback = config->dma_callback;
		stream->user_data = config->user_data;
		return 0;
	}

	if (id >= priv->max_streams) {
		LOG_ERR("cannot configure the dma stream %d.", id);
		return -EINVAL;
	}

	if (stream->busy) {
		LOG_ERR("dma stream %d is busy.", id);
		return -EBUSY;
	}

	if (stm32_dma_disable_stream(dma, id) != 0) {
		LOG_ERR("could not disable dma stream %d.", id);
		return -EBUSY;
	}

	_stm32_dma_clear_stream_irq(dev, id);

	if (config->head_block->block_size > DMA_STM32_MAX_DATA_ITEMS) {
		LOG_ERR("Data size too big: %d\n",
		       config->head_block->block_size);
		return -EINVAL;
	}

#ifdef CONFIG_DMA_STM32_V1
	if ((config->channel_direction == MEMORY_TO_MEMORY) &&
		(!priv->support_m2m)) {
		LOG_ERR("Memcopy not supported for device %s",
			dev->name);
		return -ENOTSUP;
	}
#endif /* CONFIG_DMA_STM32_V1 */

	/* support only the same data width for source and dest */
	if ((config->dest_data_size != config->source_data_size)) {
		LOG_ERR("source and dest data size differ.");
		return -EINVAL;
	}

	if (config->source_data_size != 4U &&
	    config->source_data_size != 2U &&
	    config->source_data_size != 1U) {
		LOG_ERR("source and dest unit size error, %d",
			config->source_data_size);
		return -EINVAL;
	}

	/*
	 * STM32's circular mode will auto reset both source address
	 * counter and destination address counter.
	 */
	if (config->head_block->source_reload_en !=
		config->head_block->dest_reload_en) {
		LOG_ERR("source_reload_en and dest_reload_en must "
			"be the same.");
		return -EINVAL;
	}

	stream->busy		= true;
	stream->dma_callback	= config->dma_callback;
	stream->direction	= config->channel_direction;
	stream->user_data       = config->user_data;
	stream->src_size	= config->source_data_size;
	stream->dst_size	= config->dest_data_size;

	/* check dest or source memory address, warn if 0 */
	if ((config->head_block->source_address == 0)) {
		LOG_WRN("source_buffer address is null.");
	}

	if ((config->head_block->dest_address == 0)) {
		LOG_WRN("dest_buffer address is null.");
	}

	if (stream->direction == MEMORY_TO_PERIPHERAL) {
		DMA_InitStruct.MemoryOrM2MDstAddress =
					config->head_block->source_address;
		DMA_InitStruct.PeriphOrM2MSrcAddress =
					config->head_block->dest_address;
	} else {
		DMA_InitStruct.PeriphOrM2MSrcAddress =
					config->head_block->source_address;
		DMA_InitStruct.MemoryOrM2MDstAddress =
					config->head_block->dest_address;
	}

	uint16_t memory_addr_adj = 0, periph_addr_adj = 0;

	ret = stm32_dma_get_priority(config->channel_priority,
				     &DMA_InitStruct.Priority);
	if (ret < 0) {
		return ret;
	}

	ret = stm32_dma_get_direction(config->channel_direction,
				      &DMA_InitStruct.Direction);
	if (ret < 0) {
		return ret;
	}

	switch (config->channel_direction) {
	case MEMORY_TO_MEMORY:
	case PERIPHERAL_TO_MEMORY:
		memory_addr_adj = config->head_block->dest_addr_adj;
		periph_addr_adj = config->head_block->source_addr_adj;
		break;
	case MEMORY_TO_PERIPHERAL:
		memory_addr_adj = config->head_block->source_addr_adj;
		periph_addr_adj = config->head_block->dest_addr_adj;
		break;
	/* Direction has been asserted in stm32_dma_get_direction. */
	default:
		LOG_ERR("Channel direction error (%d).",
				config->channel_direction);
		return -EINVAL;
	}

	ret = stm32_dma_get_memory_increment(memory_addr_adj,
					&DMA_InitStruct.MemoryOrM2MDstIncMode);
	if (ret < 0) {
		return ret;
	}
	ret = stm32_dma_get_periph_increment(periph_addr_adj,
					&DMA_InitStruct.PeriphOrM2MSrcIncMode);
	if (ret < 0) {
		return ret;
	}

	if (config->head_block->source_reload_en) {
		DMA_InitStruct.Mode = LL_DMA_MODE_CIRCULAR;
	} else {
		DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
	}

	stream->source_periph = (stream->direction == PERIPHERAL_TO_MEMORY);

	/* set the data width, when source_data_size equals dest_data_size */
	int index = ffs(config->source_data_size) - 1;
	DMA_InitStruct.PeriphOrM2MSrcDataSize = table_p_size[index];
	index = ffs(config->dest_data_size) - 1;
	DMA_InitStruct.MemoryOrM2MDstDataSize = table_m_size[index];

#if defined(CONFIG_DMA_STM32_V1)
	DMA_InitStruct.MemBurst = stm32_dma_get_mburst(config,
						       stream->source_periph);
	DMA_InitStruct.PeriphBurst = stm32_dma_get_pburst(config,
							stream->source_periph);

#if !defined(CONFIG_SOC_SERIES_STM32H7X)
	if (config->channel_direction != MEMORY_TO_MEMORY) {
		if (config->dma_slot >= 8) {
			LOG_ERR("dma slot error.");
			return -EINVAL;
		}
	} else {
		if (config->dma_slot >= 8) {
			LOG_ERR("dma slot is too big, using 0 as default.");
			config->dma_slot = 0;
		}
	}

	DMA_InitStruct.Channel = dma_stm32_slot_to_channel(config->dma_slot);
#endif

	DMA_InitStruct.FIFOThreshold = stm32_dma_get_fifo_threshold(
					config->head_block->fifo_mode_control);

	if (stm32_dma_check_fifo_mburst(&DMA_InitStruct)) {
		DMA_InitStruct.FIFOMode = LL_DMA_FIFOMODE_ENABLE;
	} else {
		DMA_InitStruct.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
	}
#endif
	if (stream->source_periph) {
		DMA_InitStruct.NbData = config->head_block->block_size /
					config->source_data_size;
	} else {
		DMA_InitStruct.NbData = config->head_block->block_size /
					config->dest_data_size;
	}

	/*
	 * the with dma V2 and dma mux,
	 * the request ID is stored in the dma_slot
	 */
	DMA_InitStruct.PeriphRequest = config->dma_slot;

	LL_DMA_Init(dma, dma_stm32_id_to_stream(id), &DMA_InitStruct);

	LL_DMA_EnableIT_TC(dma, dma_stm32_id_to_stream(id));

	/* Enable Half-Transfer irq if circular mode is enabled */
	if (config->head_block->source_reload_en) {
		LL_DMA_EnableIT_HT(dma, dma_stm32_id_to_stream(id));
	}

#if defined(CONFIG_DMA_STM32_V1)
	if (DMA_InitStruct.FIFOMode == LL_DMA_FIFOMODE_ENABLE) {
		LL_DMA_EnableFifoMode(dma, dma_stm32_id_to_stream(id));
		LL_DMA_EnableIT_FE(dma, dma_stm32_id_to_stream(id));
	} else {
		LL_DMA_DisableFifoMode(dma, dma_stm32_id_to_stream(id));
		LL_DMA_DisableIT_FE(dma, dma_stm32_id_to_stream(id));
	}
#endif
	return ret;
}

DMA_STM32_EXPORT_API int dma_stm32_reload(struct drvmgr_dev *dev, uint32_t id,
	dma_addr_t src, dma_addr_t dst, size_t size) {
    struct stm32_dma *priv = dev->priv;
	DMA_TypeDef *dma = priv->dma;
	struct dma_stm32_stream *stream;

	/* give channel from index 0 */
	id = id - STREAM_OFFSET;

	if (id >= priv->max_streams) {
		return -EINVAL;
	}

	stream = &priv->streams[id];

	if (stm32_dma_disable_stream(dma, id) != 0) {
		return -EBUSY;
	}

	switch (stream->direction) {
	case MEMORY_TO_PERIPHERAL:
		LL_DMA_SetMemoryAddress(dma, dma_stm32_id_to_stream(id), src);
		LL_DMA_SetPeriphAddress(dma, dma_stm32_id_to_stream(id), dst);
		break;
	case MEMORY_TO_MEMORY:
	case PERIPHERAL_TO_MEMORY:
		LL_DMA_SetPeriphAddress(dma, dma_stm32_id_to_stream(id), src);
		LL_DMA_SetMemoryAddress(dma, dma_stm32_id_to_stream(id), dst);
		break;
	default:
		return -EINVAL;
	}

	if (stream->source_periph) {
		LL_DMA_SetDataLength(dma, dma_stm32_id_to_stream(id),
				     size / stream->src_size);
	} else {
		LL_DMA_SetDataLength(dma, dma_stm32_id_to_stream(id),
				     size / stream->dst_size);
	}

	stm32_dma_enable_stream(dma, id);
	return 0;
}

DMA_STM32_EXPORT_API int dma_stm32_start(struct drvmgr_dev *dev, uint32_t id) {
    struct stm32_dma *priv = dev->priv;
	DMA_TypeDef *dma = priv->dma;

	/* give channel from index 0 */
	id = id - STREAM_OFFSET;

	/* Only M2P or M2M mode can be started manually. */
	if (id >= DMA_STREAM_COUNT) 
		return -EINVAL;
	
	_stm32_dma_clear_stream_irq(dev, id);
	stm32_dma_enable_stream(dma, id);
	return 0;
}

DMA_STM32_EXPORT_API int dma_stm32_stop(struct drvmgr_dev *dev, uint32_t id) {
	struct stm32_dma *priv = dev->priv;
	struct dma_stm32_stream *stream = &priv->streams[id - STREAM_OFFSET];
	DMA_TypeDef *dma = priv->dma;

	/* give channel from index 0 */
	id = id - STREAM_OFFSET;

	if (id >= DMA_STREAM_COUNT) 
		return -EINVAL;
	
#if !defined(CONFIG_DMAMUX_STM32) || defined(CONFIG_SOC_SERIES_STM32H7X)
	LL_DMA_DisableIT_TC(dma, dma_stm32_id_to_stream(id));
#endif /* CONFIG_DMAMUX_STM32 */

#if defined(CONFIG_DMA_STM32_V1)
	stm32_dma_disable_fifo_irq(dma, id);
#endif
	stm32_dma_disable_stream(dma, id);
	_stm32_dma_clear_stream_irq(dev, id);

	/* Finally, flag stream as free */
	stream->busy = false;

	return 0;
}

DMA_STM32_EXPORT_API int dma_stm32_get_status(struct drvmgr_dev *dev,
	uint32_t id, struct dma_status *stat) {
	struct stm32_dma *priv = dev->priv;
	DMA_TypeDef *dma = priv->dma;
	struct dma_stm32_stream *stream;

	/* give channel from index 0 */
	id = id - STREAM_OFFSET;
	if (id >= DMA_STREAM_COUNT) {
		return -EINVAL;
	}

	stream = &priv->streams[id];
	stat->pending_length = LL_DMA_GetDataLength(dma, dma_stm32_id_to_stream(id));
	stat->dir = stream->direction;
	stat->busy = stream->busy;

	return 0;
}

static const struct dma_operations stm32_dma_ops = {
	.reload		 = dma_stm32_reload,
	.configure   = dma_stm32_configure,
	.start		 = dma_stm32_start,
	.stop		 = dma_stm32_stop,
	.get_status	 = dma_stm32_get_status,
};

static void stm32_dma_stream0_isr(void *arg) {
	stm32_dma_irq_handler(arg, 0);
}

static void stm32_dma_stream1_isr(void *arg) {
	stm32_dma_irq_handler(arg, 1);
}

static void stm32_dma_stream2_isr(void *arg) {
	stm32_dma_irq_handler(arg, 2);
}

static void stm32_dma_stream3_isr(void *arg) {
	stm32_dma_irq_handler(arg, 3);
}

static void stm32_dma_stream4_isr(void *arg) {
	stm32_dma_irq_handler(arg, 4);
}

static void stm32_dma_stream5_isr(void *arg) {
	stm32_dma_irq_handler(arg, 5);
}

static void stm32_dma_stream6_isr(void *arg) {
	stm32_dma_irq_handler(arg, 6);
}

static void stm32_dma_stream7_isr(void *arg) {
	stm32_dma_irq_handler(arg, 7);
}

static int stm32_dma_preprobe(struct drvmgr_dev *dev) {
	struct dev_private *devp = device_get_private(dev);
	rtems_ofw_memory_area reg;
    struct stm32_dma *priv;
	pcell_t offset;
    
	if (rtems_ofw_get_reg(devp->np, &reg, sizeof(reg)) < 0) 
		return -ENOSTR;
	if (rtems_ofw_get_enc_prop(devp->np, "dma-offset", &offset, sizeof(offset)) < 0)
		return -ENOSTR;
	priv = rtems_calloc(1, sizeof(*priv));
	if (!priv)
		return -ENOMEM;
	priv->dma = (void *)reg.start; 
	priv->offset = offset;
    priv->support_m2m = rtems_ofw_has_prop(devp->np, "st,mem2mem");
	dev->priv = priv;
    devp->devops = &stm32_dma_ops;
    return 0;
}

static int stm32_dma_probe(struct drvmgr_dev *dev) {
	struct dev_private *devp = device_get_private(dev);
	struct stm32_dma *priv = dev->priv;
	rtems_vector_number irqs[DMA_STREAM_COUNT];
	int ret;

	drvmgr_isr isrs[] = {
		stm32_dma_stream0_isr,
		stm32_dma_stream1_isr,
		stm32_dma_stream2_isr,
		stm32_dma_stream3_isr,
		stm32_dma_stream4_isr,
		stm32_dma_stream5_isr,
		stm32_dma_stream6_isr,
		stm32_dma_stream7_isr
	};

	ret = rtems_ofw_get_interrupts(devp->np, irqs, sizeof(irqs));
	if (ret != DMA_STREAM_COUNT) {
		ret = -EINVAL;
		goto _failed;
	}
    if (stm32_ofw_get_clkdev(devp->np, &priv->clk, &priv->clkid)) {
        ret = -ENODEV;
		goto _failed;
    }
	for (uint32_t i = 0; i < DMA_STREAM_COUNT; i++) {
		ret = drvmgr_interrupt_register(dev, IRQF_HARD(irqs[i]), 
			dev->name, isrs[i], dev);
		if (ret)
			goto _failed;
		priv->streams[i].busy = false;
#ifdef CONFIG_DMAMUX_STM32
		/* each further stream->mux_channel is fixed here */
		priv->streams[i].mux_channel = i + priv->offset;
#endif /* CONFIG_DMAMUX_STM32 */
	}
	clk_enable(priv->clk, &priv->clkid);
	return 0;
_failed:
	free(priv);
	return ret;
}

static const struct dev_id id_table[] = {
    {.compatible = "st,stm32-dma", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops stm32_dma_driver = {
	.init = {
        stm32_dma_preprobe,
		stm32_dma_probe
	},
};
		
OFW_PLATFORM_DRIVER(stm32_dma) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "dma",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &stm32_dma_driver
	},
	.ids = id_table
};
