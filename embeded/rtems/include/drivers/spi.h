/*
 * CopyRight(c) 2022 wtcat
 */

#ifndef DRIVER_SPI_H_
#define DRIVER_SPI_H_

#include <rtems/thread.h>
#include <dev/spi/spi.h>

#include "drivers/platform_bus.h"

#ifdef __cplusplus
extern "C"{
#endif

static inline ssize_t spi_master_transfer(struct drvmgr_dev *dev, 
    spi_ioc_transfer *msgs, uint32_t n) {
    spi_bus *bus = (spi_bus *)dev->priv;
    int err;
    rtems_recursive_mutex_lock(&bus->mutex);
    err = bus->transfer(bus, msgs, n);
    rtems_recursive_mutex_unlock(&bus->mutex);
    return err == 0? msgs->len: -err;  
}

static inline ssize_t spi_master_write(struct drvmgr_dev *dev, 
    const void *buffer, size_t size) {
    spi_ioc_transfer msg = {
        .len = (uint16_t)size,
        .tx_buf = buffer
    };
    return spi_master_transfer(dev, &msg, 1);
}

static inline ssize_t spi_master_read(struct drvmgr_dev *dev, 
    void *buffer, size_t size) {
    spi_ioc_transfer msg = {
        .len = (uint16_t)size,
        .rx_buf = buffer
    };
    return spi_master_transfer(dev, &msg, 1);
}

#ifdef __cplusplus
}
#endif
#endif /* DRIVER_SPI_H_ */
