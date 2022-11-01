/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <rtems/thread.h>
#include <rtems/malloc.h>
#include <rtems/blkdev.h>

#include "drivers/ofw_platform_bus.h"
#include "drivers/spi.h"
#include "linux/spi/spidev.h"


struct flash_info {
    uint32_t start;
    uint32_t size;
    uint32_t pagesz;
};

struct flash_private {
    rtems_mutex lock;
    struct drvmgr_dev *spi_master;
    uint32_t max_freq;
    const struct flash_info *info;
};

static const struct flash_info w25q128 = {
    .start = 0,
    .size = 16*1024*1024u,
    .pagesz = 4096
};

static const struct dev_id id_table[] = {
    {.compatible = "winbond,w25q128", (void *)&w25q128},
    {NULL, NULL}
};


static int flash_spi_xfer(struct flash_private *priv, const void *tx_buf, 
    void *rx_buf, size_t len) {
    spi_ioc_transfer msg = {
        .len = (uint16_t)len,
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .cs_change = true,
        .cs = 0,
        .bits_per_word = 8,
        .mode = SPI_MODE_0,
        .speed_hz = priv->max_freq,
        .delay_usecs = 0
    };
    return spi_master_transfer(priv->spi_master, &msg, 1);
}

static int flash_spi_write_and_read(struct flash_private *priv, const void *tx_buf, 
    size_t tx_len, void *rx_buf, size_t rx_len) {
    spi_ioc_transfer msgs[] = {
        {
            .len = (uint16_t)tx_len,
            .tx_buf = tx_buf,
            .rx_buf = NULL,
            .cs_change = true,
            .cs = 0,
            .bits_per_word = 8,
            .mode = SPI_MODE_0,
            .speed_hz = priv->max_freq,
            .delay_usecs = 0
        },{
            .len = (uint16_t)rx_len,
            .tx_buf = NULL,
            .rx_buf = rx_buf,
            .cs_change = true,
            .cs = 0,
            .bits_per_word = 8,
            .mode = SPI_MODE_0,
            .speed_hz = priv->max_freq,
            .delay_usecs = 0
        }
    };
    return spi_master_transfer(priv->spi_master, msgs, 2);
}

static uint32_t flash_read_id(struct flash_private *priv) {
    uint8_t cmd[] = {0x90, 0, 0, 0};
    uint16_t id = 0;

    flash_spi_write_and_read()
}

static int flash_read(struct flash_private *priv, rtems_blkdev_request *r) {
    rtems_blkdev_request_done(r, 0);
}

static int flash_write(struct flash_private *priv, rtems_blkdev_request *r) {
    rtems_blkdev_request_done(r, 0);
}

static int flash_ioctl(rtems_disk_device *dd, uint32_t req, void *arg) {
	if (req == RTEMS_BLKIO_REQUEST) {
		struct flash_private *priv = rtems_disk_get_driver_data(dd);
		rtems_blkdev_request *r = (rtems_blkdev_request *)arg;
		switch (r->req) {
        case RTEMS_BLKDEV_REQ_READ:
            return flash_read(priv, r);
        case RTEMS_BLKDEV_REQ_WRITE:
            return flash_write(priv, r);
        default:
            return -EINVAL;
		}
	} else if (req == RTEMS_BLKIO_CAPABILITIES) {
		*(uint32_t *) arg = RTEMS_BLKDEV_CAP_MULTISECTOR_CONT;
		return 0;
	} else {
		return rtems_blkdev_ioctl(dd, req, arg);
	}
}

static int flash_preprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct flash_private *priv;
    const struct dev_id *id;
    pcell_t prop;

    if (rtems_ofw_get_enc_prop(devp->np, "spi-max-frequency", 
        &prop, sizeof(prop)) < 0)
        return -EINVAL;
    priv = rtems_calloc(1, sizeof(*priv));
    if (!priv)
        return -ENOMEM;

    id = ofw_device_match(dev, id_table);
    priv->max_freq = (uint32_t)prop;
    priv->spi_master = dev->parent->dev;
    priv->info = id->data;
    dev->priv = priv;
    return 0;
}

static int flash_probe(struct drvmgr_dev *dev) {
    struct flash_private *priv = dev->priv;
    rtems_status_code sc;

    sc = rtems_blkdev_create(dev->name, priv->blksize, 
    priv->phy_size / priv->blksize, flash_ioctl, priv);
    if (sc != RTEMS_SUCCESSFUL) {
        printk("%s: create blkdev(%s) failed(%s)\n", __func__, 
            dev->name, rtems_status_text(sc));
        return -EFAULT;
    }

    return 0;
}

static struct drvmgr_drv_ops flash_driver = {
	.init = {
        [0] = flash_preprobe, 
        [3] = flash_probe
	}
};

OFW_PLATFORM_DRIVER(spi_flash) = {
	.drv = {
		.drv_id   = DRIVER_SPI_ID,
		.name     = "w25q128",
		.bus_type = DRVMGR_BUS_TYPE_SPI,
		.ops      = &flash_driver
	},
    .ids = id_table
};
