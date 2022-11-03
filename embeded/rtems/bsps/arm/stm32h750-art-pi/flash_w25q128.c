/*
 * Copyright 2022 wtcat
 */
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <rtems/thread.h>
#include <rtems/malloc.h>
#include <rtems/blkdev.h>
#include <sys/errno.h>

#include "drivers/ofw_platform_bus.h"
#include "drivers/spi.h"

/* 
 * chip command definition 
 */
#define W25QXX_COMMAND_WRITE_ENABLE                      0x06        /**< write enable */
#define W25QXX_COMMAND_VOLATILE_SR_WRITE_ENABLE          0x50        /**< sr write enable */
#define W25QXX_COMMAND_WRITE_DISABLE                     0x04        /**< write disable */
#define W25QXX_COMMAND_READ_STATUS_REG1                  0x05        /**< read status register-1 */
#define W25QXX_COMMAND_READ_STATUS_REG2                  0x35        /**< read status register-2 */
#define W25QXX_COMMAND_READ_STATUS_REG3                  0x15        /**< read status register-3 */
#define W25QXX_COMMAND_WRITE_STATUS_REG1                 0x01        /**< write status register-1 */
#define W25QXX_COMMAND_WRITE_STATUS_REG2                 0x31        /**< write status register-2 */
#define W25QXX_COMMAND_WRITE_STATUS_REG3                 0x11        /**< write status register-3 */
#define W25QXX_COMMAND_CHIP_ERASE                        0xC7        /**< chip erase */
#define W25QXX_COMMAND_ERASE_PROGRAM_SUSPEND             0x75        /**< erase suspend */
#define W25QXX_COMMAND_ERASE_PROGRAM_RESUME              0x7A        /**< erase resume */
#define W25QXX_COMMAND_POWER_DOWN                        0xB9        /**< power down */
#define W25QXX_COMMAND_RELEASE_POWER_DOWN                0xAB        /**< release power down */
#define W25QXX_COMMAND_READ_MANUFACTURER                 0x90        /**< manufacturer */
#define W25QXX_COMMAND_JEDEC_ID                          0x9F        /**< jedec id */
#define W25QXX_COMMAND_GLOBAL_BLOCK_SECTOR_LOCK          0x7E        /**< global block lock */
#define W25QXX_COMMAND_GLOBAL_BLOCK_SECTOR_UNLOCK        0x98        /**< global block unlock */
#define W25QXX_COMMAND_ENTER_QSPI_MODE                   0x38        /**< enter spi mode */
#define W25QXX_COMMAND_ENABLE_RESET                      0x66        /**< enable reset */
#define W25QXX_COMMAND_RESET_DEVICE                      0x99        /**< reset device */
#define W25QXX_COMMAND_READ_UNIQUE_ID                    0x4B        /**< read unique id */
#define W25QXX_COMMAND_PAGE_PROGRAM                      0x02        /**< page program */
#define W25QXX_COMMAND_QUAD_PAGE_PROGRAM                 0x32        /**< quad page program */
#define W25QXX_COMMAND_SECTOR_ERASE_4K                   0x20        /**< sector erase */
#define W25QXX_COMMAND_BLOCK_ERASE_32K                   0x52        /**< block erase */
#define W25QXX_COMMAND_BLOCK_ERASE_64K                   0xD8        /**< block erase */
#define W25QXX_COMMAND_READ_DATA                         0x03        /**< read data */
#define W25QXX_COMMAND_FAST_READ                         0x0B        /**< fast read */
#define W25QXX_COMMAND_FAST_READ_DUAL_OUTPUT             0x3B        /**< fast read dual output */
#define W25QXX_COMMAND_FAST_READ_QUAD_OUTPUT             0x6B        /**< fast read quad output */
#define W25QXX_COMMAND_READ_SFDP_REGISTER                0x5A        /**< read SFDP register */
#define W25QXX_COMMAND_ERASE_SECURITY_REGISTER           0x44        /**< erase security register */
#define W25QXX_COMMAND_PROGRAM_SECURITY_REGISTER         0x42        /**< program security register */
#define W25QXX_COMMAND_READ_SECURITY_REGISTER            0x48        /**< read security register */
#define W25QXX_COMMAND_INDIVIDUAL_BLOCK_LOCK             0x36        /**< individual block lock */
#define W25QXX_COMMAND_INDIVIDUAL_BLOCK_UNLOCK           0x39        /**< individual block unlock */
#define W25QXX_COMMAND_READ_BLOCK_LOCK                   0x3D        /**< read block lock */
#define W25QXX_COMMAND_FAST_READ_DUAL_IO                 0xBB        /**< fast read dual I/O */
#define W25QXX_COMMAND_DEVICE_ID_DUAL_IO                 0x92        /**< device id dual I/O */
#define W25QXX_COMMAND_SET_BURST_WITH_WRAP               0x77        /**< set burst with wrap */
#define W25QXX_COMMAND_FAST_READ_QUAD_IO                 0xEB        /**< fast read quad I/O */
#define W25QXX_COMMAND_WORD_READ_QUAD_IO                 0xE7        /**< word read quad I/O */
#define W25QXX_COMMAND_OCTAL_WORD_READ_QUAD_IO           0xE3        /**< octal word read quad I/O */
#define W25QXX_COMMAND_DEVICE_ID_QUAD_IO                 0x94        /**< device id quad I/O */


#define PROGRAM_PAGE_SIZE 256

struct flash_info {
    uint32_t devid;
    uint32_t capacity;
    uint32_t blksz;
};

struct flash_private {
    rtems_mutex lock;
    struct drvmgr_dev *master;
    uint32_t max_freq;
    const struct flash_info *info;
    uint8_t buffer[256+6];
};

static const struct flash_info w25q128 = {
    .devid = 0xEF17,
    .capacity = 16*1024*1024u,
    .blksz = 4096
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
    return spi_master_transfer(priv->master, &msg, 1);
}

static int flash_spi_write_and_read(struct flash_private *priv, const void *tx_buf, 
    size_t tx_len, void *rx_buf, size_t rx_len) {
    spi_ioc_transfer msgs[] = {
        {
            .len = (uint16_t)tx_len,
            .tx_buf = tx_buf,
            .rx_buf = NULL,
            .cs_change = false,
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
    return spi_master_transfer(priv->master, msgs, 2);
}

static uint32_t flash_read_id(struct flash_private *priv) {
    uint8_t cmd[] = {0x90, 0, 0, 0};
    uint16_t id = 0;

    if (flash_spi_write_and_read(priv, cmd, sizeof(cmd), &id, 2) > 0)
        return id;
    return 0;
}

static int flash_write_enable(struct flash_private *priv) {
    uint8_t cmd = W25QXX_COMMAND_WRITE_ENABLE;
    int err;

    /* Write enable */
    err = flash_spi_xfer(priv, &cmd, NULL, 1);
    if (err)
        printk("%s: flash write enable error\n", __func__);
    return err;
}

static int flash_wait_program_complete(struct flash_private *priv) {
    uint8_t cmd = W25QXX_COMMAND_READ_STATUS_REG1;
    uint8_t sta = 0x1;
    int trycnt = 3;
    do {
        int ret = flash_spi_write_and_read(priv, &cmd, 1, &sta, 1);
        if (ret) {
            printk("%s: read status register failed\n", __func__);
            return -EIO;
        }
        if (!(sta & 0x1))
            return 0;
        rtems_task_wake_after(1);
    } while (--trycnt > 0);
    printk("%s: read status register timeout\n", __func__);
    return -ETIMEDOUT;
}

static int flash_erase_sector(struct flash_private *priv, off_t offset) {
    uint cmds[4];

    /* Write enable */
    int err = flash_write_enable(priv);
    if (err)
        return err;

    cmds[0] = W25QXX_COMMAND_SECTOR_ERASE_4K;
    cmds[1] = (offset >> 16) & 0xFF;
    cmds[2] = (offset >> 8) & 0xFF;
    cmds[3] = (offset >> 0) & 0xFF;
    err = flash_spi_xfer(priv, cmds, NULL, 4);
    if (err) {
        printk("%s: flash erase sector failed\n", __func__);
        return err;
    }

    return flash_wait_program_complete(priv);
}

static int flash_program_page(struct flash_private *priv, const void *buf, 
    size_t size, off_t offset) {

    /* Write enable */
    int err = flash_write_enable(priv);
    if (err)
        return err;

    /* Program page */
    priv->buffer[0] = W25QXX_COMMAND_PAGE_PROGRAM;
    priv->buffer[1] = (offset >> 16) & 0xFF;
    priv->buffer[2] = (offset >> 8) & 0xFF;
    priv->buffer[3] = (offset >> 0) & 0xFF;
    memcpy(&priv->buffer[4], buf, size); 
    err = flash_spi_xfer(priv, priv->buffer, NULL, size + 4);
    if (err) {
        printk("%s: program page failed\n", __func__);
        return -EIO;
    }
    return flash_wait_program_complete(priv);
}

static int flash_read_data(struct flash_private *priv, void *buf, size_t size, 
    off_t offset) {
    uint8_t cmds[6];
    cmds[0] = W25QXX_COMMAND_FAST_READ;
    cmds[1] = (offset >> 16) & 0xFF;
    cmds[2] = (offset >> 8) & 0xFF;
    cmds[3] = (offset >> 0) & 0xFF;
    cmds[4] = 0x00; 
    return flash_spi_write_and_read(priv, cmds, 5, buf, size);
}

static int flash_write_data(struct flash_private *priv, void *buf, size_t size, 
    off_t offset) {
    size_t remain;
    int err = 0;

    remain = PROGRAM_PAGE_SIZE - offset % PROGRAM_PAGE_SIZE;
    if (size <= remain)
        remain = size;

    while (size > 0) {
        err = flash_program_page(priv, buf, remain, offset);
        if (err)
            break;
        offset += remain;
        buf += remain;
        size -= remain;
        remain = min_t(size_t, PROGRAM_PAGE_SIZE, size);
    }
    return err;
}

static int flash_read(struct flash_private *priv, rtems_blkdev_request *r) {
    rtems_blkdev_sg_buffer *sg = r->bufs;
    off_t offset;
    int err;

    rtems_mutex_lock(&priv->lock);
    for (uint32_t n = 0; n < r->bufnum; n++) {
        offset = sg->block * priv->info->blksz;
        err = flash_read_data(priv, sg->buffer, sg->length, offset);
        if (err)
            break;
    }
    rtems_blkdev_request_done(r, err);
    rtems_mutex_unlock(&priv->lock);
    return err;
}

static int flash_write(struct flash_private *priv, rtems_blkdev_request *r) {
    rtems_blkdev_sg_buffer *sg;
    off_t offset;
    uint32_t n;
    int err;

    rtems_mutex_lock(&priv->lock);
    for (n = 0, sg = r->bufs; n < r->bufnum; n++) {
        offset = sg->block * priv->info->blksz;
        err = flash_erase_sector(priv, offset);
        if (err)
            break;
        err = flash_write_data(priv, sg->buffer, sg->length, offset);
        if (err)
            break;
    }
    rtems_blkdev_request_done(r, err);
    rtems_mutex_unlock(&priv->lock);
    return err;
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

    rtems_mutex_init(&priv->lock, "w25qxx");
    id = ofw_device_match(dev, id_table);
    priv->max_freq = (uint32_t)prop;
    priv->master = dev->parent->dev;
    priv->info = id->data;
    dev->priv = priv;
    return 0;
}

static int flash_probe(struct drvmgr_dev *dev) {
    struct flash_private *priv = dev->priv;
    const struct flash_info *info = priv->info;
    rtems_status_code sc;

    uint32_t id = flash_read_id(priv);
    if (info->devid != id) {
        printk("%s: invalid flash device id(0x%x)\n", __func__, id);
        goto _fault; 
    }

    sc = rtems_blkdev_create(dev->name, info->blksz, 
        info->capacity / info->blksz, flash_ioctl, priv);
    if (sc != RTEMS_SUCCESSFUL) {
        printk("%s: create blkdev(%s) failed(%s)\n", __func__, 
            dev->name, rtems_status_text(sc));
        return -EFAULT;
    }
    return 0;

_fault:
    free(priv);
    return -EFAULT;
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
