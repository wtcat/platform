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
	long dma_channel[1];
	struct dmamux_stm32_channel mux_channels[];
};

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
	if (dma_stm32_configure(priv->mux_channels[id].dev_dma,
			priv->mux_channels[id].dma_id, config) != 0) {
		printk("cannot configure the dmamux.");
		return -EINVAL;
	}

	/* set the Request Line ID to this dmamux channel i */
	LL_DMAMUX_SetRequestID(priv->dmamux, id, request_id);

	return 0;
}

static int dmamux_stm32_start(struct drvmgr_dev *dev, uint32_t id) {
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

int dmamux_stm32_stop(struct drvmgr_dev *dev, uint32_t id) {
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

int dmamux_stm32_reload(struct drvmgr_dev *dev, uint32_t id,
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

int dmamux_stm32_get_status(struct drvmgr_dev *dev, uint32_t id,
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

static const struct dma_operations stm32h7_dma_ops = {
	.reload		 = dmamux_stm32_reload,
	.configure	 = dmamux_stm32_configure,
	.start		 = dmamux_stm32_start,
	.stop		 = dmamux_stm32_stop,
	.get_status	 = dmamux_stm32_get_status
};


static int dmamux_stm32_preprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct stm32h7_dmamux *priv;
    pcell_t chan, req, gen;

	if (rtems_ofw_get_enc_prop(devp->np, "dma-channels", &chan, sizeof(chan)) < 0)
		return -ENOSTR;
	if (rtems_ofw_get_enc_prop(devp->np, "dma-requests", &req, sizeof(req)) < 0)
		return -ENOSTR;
	if (rtems_ofw_get_enc_prop(devp->np, "dma-generators", &gen, sizeof(gen)) < 0)
		return -ENOSTR;
    priv = rtems_calloc(1, sizeof(*priv) + chan * sizeof(struct dmamux_stm32_channel));
	if (!priv)
		return -ENOMEM;
	dma_context_init(&priv->context, priv->dma_channel, chan);
    priv->channel_nb = (uint8_t)chan;
    priv->gen_nb = (uint8_t)gen;
    priv->req_nb = (uint8_t)req;
    dev->priv = priv;
	return 0;
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
	devp->devops = &stm32h7_dma_ops;
    clk_enable(priv->clk, &priv->clkid);
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
