/*
 * Copyright (c) 2020 STMicroelectronics
 * Copyright (c) 2022 wtcat
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Common part of DMAMUX drivers for stm32.
 * @note  api functions named dmamux_stm32_
 *        are calling the dma_stm32 corresponding function
 *        implemented in dma_stm32.c
 */
#include <stdlib.h>
#include <rtems/malloc.h>
#include <rtems/bspIo.h>

#include "base/macros.h"
#include "drivers/dma.h"
#include "drivers/clock.h"
#include "drivers/ofw_platform_bus.h"

#include "stm32/stm32_dma.h"
#include "stm32/stm32_com.h"

/* this is the configuration of one dmamux channel */
struct dmamux_stm32_channel {
	/* pointer to the associated dma instance */
	struct drvmgr_dev *dev_dma;
	/* ref of the associated dma stream for this instance */
	uint8_t dma_id;
};

/* the table of all the dmamux channel */
struct dmamux_stm32_data {
	void *callback_arg;
	void (*dmamux_callback)(void *arg, uint32_t id,
				int error_code);
};

struct stm32h7_dmamux {
	struct dma_context context;
    DMAMUX_Channel_TypeDef *dmamux;
    struct drvmgr_dev *clk;
    int clkid;
	uint8_t channel_nb;	/* total nb of channels */
	uint8_t gen_nb;	/* total nb of Request generator */
	uint8_t req_nb;	/* total nb of Peripheral Request inputs */
	struct dmamux_stm32_channel mux_channels[];
};

#ifdef DMAMUX_DEBUG
#define dma_dbg(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define dma_dbg(...) 
#endif

static int dmamux_stm32_configure(struct drvmgr_dev *dev, uint32_t id,
    struct dma_config *config) {
    struct stm32h7_dmamux *priv = dev->priv;
	/*
	 * request line ID for this mux channel is stored
	 * in the dma_slot parameter
	 */
	int request_id = config->dma_slot;

	if (request_id > priv->req_nb + priv->gen_nb) {
		printk("request ID %d is not valid.", request_id);
		return -EINVAL;
	}

	/* check if this channel is valid */
	if (id >= priv->channel_nb) {
		printk("channel ID %d is too big.", id);
		return -EINVAL;
	}

	/*
	 * Also configures the corresponding dma channel
	 * instance is given by the dev_dma
	 * stream is given by the index i
	 * config is directly this dma_config
	 */

	/*
	 * This dmamux channel 'id' is now used for this peripheral request
	 * It gives this mux request ID to the dma through the config.dma_slot
	 */
	dma_dbg("\ndmamux_stm32_configure: %s stream(%d) channel(%d) request_id(%d)\n\n", 
			priv->mux_channels[id].dev_dma->name, 
			priv->mux_channels[id].dma_id,
			id, request_id);
	if (dma_stm32_configure(priv->mux_channels[id].dev_dma,
			priv->mux_channels[id].dma_id, config) != 0) {
		printk("cannot configure the dmamux.");
		return -EINVAL;
	}

	/* set the Request Line ID to this dmamux channel i */
	LL_DMAMUX_SetRequestID(priv->dmamux, id, request_id);
	return 0;
}

static int __fastcode dmamux_stm32_start(struct drvmgr_dev *dev, uint32_t id) {
	struct stm32h7_dmamux *priv = dev->priv;

	/* check if this channel is valid */
	if (id >= priv->channel_nb) {
		printk("channel ID %d is too big.", id);
		return -EINVAL;
	}
	if (dma_stm32_start(priv->mux_channels[id].dev_dma,
		priv->mux_channels[id].dma_id) != 0) {
		printk("cannot start the dmamux channel %d.", id);
		return -EINVAL;
	}

	return 0;
}

static int __fastcode dmamux_stm32_stop(struct drvmgr_dev *dev, uint32_t id) {
	struct stm32h7_dmamux *priv = dev->priv;

	/* check if this channel is valid */
	if (id >= priv->channel_nb) {
		printk("channel ID %d is too big.", id);
		return -EINVAL;
	}

	if (dma_stm32_stop(priv->mux_channels[id].dev_dma,
		priv->mux_channels[id].dma_id) != 0) {
		printk("cannot stop the dmamux channel %d.", id);
		return -EINVAL;
	}

	return 0;
}

static int __fastcode dmamux_stm32_reload(struct drvmgr_dev *dev, uint32_t id,
    dma_addr_t src, dma_addr_t dst, size_t size) {
	struct stm32h7_dmamux *priv = dev->priv;

	/* check if this channel is valid */
	if (id >= priv->channel_nb) {
		printk("channel ID %d is too big.", id);
		return -EINVAL;
	}

	if (dma_stm32_reload(priv->mux_channels[id].dev_dma,
		priv->mux_channels[id].dma_id, src, dst, size) != 0) {
		printk("cannot reload the dmamux channel %d.", id);
		return -EINVAL;
	}

	return 0;
}

static int __fastcode dmamux_stm32_get_status(struct drvmgr_dev *dev, uint32_t id,
    struct dma_status *stat) {
	struct stm32h7_dmamux *priv = dev->priv;

	/* check if this channel is valid */
	if (id >= priv->channel_nb) {
		printk("channel ID %d is too big.", id);
		return -EINVAL;
	}

	if (dma_stm32_get_status(priv->mux_channels[id].dev_dma,
		priv->mux_channels[id].dma_id, stat) != 0) {
		printk("cannot get the status of dmamux channel %d.", id);
		return -EINVAL;
	}

	return 0;
}

static struct dma_mem_descriptor *dmamux_memcpy_prepare(struct drvmgr_dev *dev, void *dst, 
	const void *src, size_t size) {
    struct dma_config *cfg;
    struct dma_block_config *blk;
    struct dma_mem_descriptor *desc;
    size_t vsize;

	if (size > 0xFFFF)
		return NULL;
    vsize = sizeof(struct dma_block_config);
    desc = rtems_calloc(1, sizeof(struct dma_mem_descriptor) + vsize);
    if (desc == NULL)
        return NULL;

    blk = &desc->blks[0];
    blk->block_size = size & 0xFFFF;
    blk->dest_address = (dma_addr_t)dst;
    blk->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
    blk->source_address = (dma_addr_t)src;
    blk->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	blk->fifo_mode_control = 3;
	
    cfg = &desc->head;
    cfg->channel_direction = MEMORY_TO_MEMORY;
    cfg->dma_slot = 0; /* m2m */
    cfg->dest_data_size = 1;
    cfg->dest_burst_length = 4;
    cfg->source_burst_length = 4;
    cfg->source_data_size = 1;
    cfg->block_count = 1;
    cfg->head_block = blk;
    desc->release = (void (*)(struct dma_mem_descriptor *))free;
    desc->length = size;
	desc->mdma = dev;
    return desc;
}

static const struct dma_operations stm32h7_dma_ops = {
	.reload		 = dmamux_stm32_reload,
	.configure	 = dmamux_stm32_configure,
	.start		 = dmamux_stm32_start,
	.stop		 = dmamux_stm32_stop,
	.get_status	 = dmamux_stm32_get_status,
	.memcpy_prepare = dmamux_memcpy_prepare
};


static int dmamux_stm32_preprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct stm32h7_dmamux *priv;
	rtems_ofw_memory_area reg;
    pcell_t chan, req, gen;
	int err;
    
	if (rtems_ofw_get_reg(devp->np, &reg, sizeof(reg)) < 0) 
		return -ENOSTR;	
	if (rtems_ofw_get_enc_prop(devp->np, "dma-channels", &chan, sizeof(chan)) < 0)
		return -ENOSTR;
	if (rtems_ofw_get_enc_prop(devp->np, "dma-requests", &req, sizeof(req)) < 0)
		return -ENOSTR;
	if (rtems_ofw_get_enc_prop(devp->np, "dma-generators", &gen, sizeof(gen)) < 0)
		return -ENOSTR;
    priv = rtems_calloc(1, sizeof(*priv) + chan * sizeof(struct dmamux_stm32_channel));
	if (!priv)
		return -ENOMEM;
	err = dma_context_init(&priv->context, chan);
	if (!err) {
		priv->channel_nb = (uint8_t)chan;
		priv->gen_nb = (uint8_t)gen;
		priv->req_nb = (uint8_t)req;
		priv->dmamux = (DMAMUX_Channel_TypeDef *)reg.start;
		devp->devops = &stm32h7_dma_ops;
		dev->priv = priv;
	}
	return err;
}

static int dmamux_stm32_probe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct stm32h7_dmamux *priv = dev->priv;
    struct drvmgr_dev *dma1, *dma2;
    pcell_t masters[2];
    int ret;

    if (stm32_ofw_get_clkdev(devp->np, &priv->clk, &priv->clkid)) {
        ret = -ENODEV;
		goto _failed;
    }
	if (rtems_ofw_get_enc_prop(devp->np, "dma-masters", masters, sizeof(masters)) < 0) {
        ret = -ENOSTR;
        goto _free;
    }
    dma1 = ofw_device_get_by_phandle(masters[0]);
    if (!dma1) {
        ret = -ENODEV;
        goto _free;
    }
    dma2 = ofw_device_get_by_phandle(masters[1]);
    if (!dma2) {
        ret = -ENODEV;
        goto _free;
    }
    for (int i = 0; i < (int)priv->channel_nb; i++) {
        priv->mux_channels[i].dev_dma = (i < 8)? dma1: dma2;
        priv->mux_channels[i].dma_id = i + 1;
    }
    clk_enable(priv->clk, &priv->clkid);
	dma_mdev_register(dev);
	return 0;

_free:
    free(priv);
_failed:
    return ret;
}


static const struct dev_id id_table[] = {
    {.compatible = "st,stm32h7-dmamux", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops stm32h7_dmamux_driver = {
	.init = {
        dmamux_stm32_preprobe,
		dmamux_stm32_probe
	},
};
		
OFW_PLATFORM_DRIVER(stm32h7_dmamux) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "dmamux",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &stm32h7_dmamux_driver
	},
	.ids = id_table
};
