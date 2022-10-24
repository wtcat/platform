/*
 * Copyright 2022 wtcat
 */
#include <stdlib.h>
#include <rtems/malloc.h>

#include "drivers/devbase.h"
#include "drivers/clock.h"
#include "drivers/dma.h"
#include "drivers/spi.h"
#include "drivers/ofw_platform_bus.h"

#include "stm32/stm32_com.h"
#include "stm32h7xx_ll_spi.h"

struct stm32h7_spi {
    spi_bus bus;
    rtems_id thread;
    const spi_ioc_transfer *msg;
    struct drvmgr_dev *dev;
    size_t msg_todo;
    size_t todo;
    int in_transfer;
    uint8_t *rxbuf;
    const uint8_t *txbuf;
    SPI_TypeDef *reg;
    struct dma_chan *tx;
    struct dma_chan *rx;
    struct drvmgr_dev *clk;
    int clkid;
    int irq;
};


static void stm32h7_spi_done(struct stm32h7_spi *priv) {
    writel_relaxed(0, priv->base + AM335X_SPI_IRQENABLE);
    rtems_event_transient_send(priv->thread);
}

static int stm32h7_spi_setup(struct stm32h7_spi *priv,
    uint32_t speed_hz, uint32_t mode, uint8_t cs) {
    (void) mode;
    (void) cs;
    uint32_t div = priv->bus.max_speed_hz / speed_hz;
    if (div && (div & (div - 1)) == 0) {

        return 0;
    }
    return -EINVAL;
}

static void stm32h7_spi_next_message(struct stm32h7_spi *spi) {
    if (spi->msg_todo > 0) {
        const spi_ioc_transfer *msg = spi->msg;
        spi_bus *bus = &spi->bus;
        if (msg->speed_hz != bus->speed_hz || 
            msg->mode != bus->mode || 
            msg->cs != bus->cs) {
            stm32h7_spi_setup(spi, msg->speed_hz, msg->mode, msg->cs);
        }
        spi->todo = msg->len;
        spi->txbuf = msg->tx_buf;
        spi->rxbuf = msg->rx_buf;

        /* xxxxxxxxxxxx */
    } else {
        stm32h7_spi_done(spi);
    }
}

static void stm32h7_spi_isr(void *arg) {
    struct stm32h7_spi *spi = arg;

    if (spi->todo > 0) {
        am437x_spi_push(priv);
    } else if (spi->in_transfer > 0) {
        writel_relaxed(AM335X_SPI_IRQENABLE_RX0_FULL, 
            priv->base + AM335X_SPI_IRQENABLE);
    } else {
        spi->msg_todo--;
        spi->msg++;
        stm32h7_spi_next_message(priv);
    }
}

static int stm32h7_spi_transfer(spi_bus *bus, const spi_ioc_transfer *msgs, 
    uint32_t n) {
    struct stm32h7_spi *spi = (struct stm32h7_spi *)bus;
    spi->msg_todo = n;
    spi->msg = &msgs[0];
    spi->thread = rtems_task_self();
    stm32h7_spi_next_message(spi);
    rtems_event_transient_receive(RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    return 0;
}

static int stm32_spi_bus_unite(struct drvmgr_drv *drv, struct drvmgr_dev *dev) {
	return ofw_platform_bus_match(drv, dev, DRVMGR_BUS_TYPE_SPI);
}

static struct drvmgr_bus_ops stm32h7_spi_bus = {
	.init = {
		ofw_platform_bus_populate_device,
	},
	.unite = stm32_spi_bus_unite
};

static int stm32_spi_preprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    rtems_ofw_memory_area reg;
    rtems_vector_number irq;
    struct stm32h7_spi *spi;
    int ret;

    ret = rtems_ofw_get_reg(devp->np, &reg, sizeof(reg));
    if (ret < 0) 
        return -ENOSTR;
    ret = rtems_ofw_get_interrupts(devp->np, &irq, sizeof(irq));
    if (ret < 0) 
        return -ENOSTR;
    spi = rtems_calloc(1, sizeof(struct stm32h7_spi));
    if (spi == NULL) 
        return -ENOMEM;
    spi->reg = (void *)reg.start;
    spi->irq = (int)irq;
    spi->dev = dev;
    dev->priv = spi;
    return ofw_platform_bus_device_register(dev, &stm32h7_spi_bus, 
    DRVMGR_BUS_TYPE_SPI);
}

static int stm32_spi_probe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct stm32h7_spi *spi = dev->priv;
    int ret;

    ret = stm32_ofw_get_clkdev(devp->np, &spi->clk, &spi->clkid);
    if (ret)
        return ret;

    clk_enable(spi->clk, &spi->clkid);
    return 0;
}

static int stm32h7_spi_extprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct stm32h7_spi *spi = dev->priv;
    pcell_t specs[3];

    spi->tx = ofw_dma_chan_request(devp->np, "tx", 
        specs, sizeof(specs));
    if (spi->tx) 
        spi->tx->config.dma_slot = specs[0];

    spi->rx = ofw_dma_chan_request(devp->np, "rx", 
        specs, sizeof(specs));
    if (spi->rx) {
        spi->rx->config.dma_slot = specs[0];

    }

    return 0;
}

static struct drvmgr_drv_ops stm32h7_spi_driver = {
	.init = {
        stm32_spi_preprobe,
		stm32_spi_probe,
        stm32h7_spi_extprobe
	},
};

OFW_PLATFORM_DRIVER(stm32h7_spi) = {
	.drv = {
		.drv_id   = DRIVER_SPI_ID,
		.name     = "spi",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &stm32h7_spi_driver
	}
};
