/*
 * CopyRight(c) 2022 wtcat
 */

#ifndef DRIVER_SPI_H_
#define DRIVER_SPI_H_

#include <rtems/thread.h>
#include <dev/spi/spi.h>

#include "drivers/devbase.h"

#ifdef __cplusplus
extern "C"{
#endif

static inline int spi_master_transfer(struct drvmgr_dev *dev, 
    spi_ioc_transfer *msgs, uint32_t n) {
    spi_bus *bus = (spi_bus *)device_get_operations(dev);
    int err;
    rtems_recursive_mutex_lock(&bus->mutex);
    err = bus->transfer(bus, msgs, n);
    rtems_recursive_mutex_unlock(&bus->mutex);
    return err;
}

static inline int spi_master_write(struct drvmgr_dev *dev, 
    const void *buffer, size_t size) {
    spi_bus *bus = (spi_bus *)device_get_operations(dev);
    spi_ioc_transfer msg = {
        .len = (uint16_t)size,
        .tx_buf = buffer,
        .cs_change = bus->cs_change,
        .cs = bus->cs,
        .bits_per_word = bus->bits_per_word,
        .mode = bus->mode,
        .speed_hz = bus->speed_hz,
        .delay_usecs = bus->delay_usecs
    };
    return spi_master_transfer(dev, &msg, 1);
}

static inline int spi_master_read(struct drvmgr_dev *dev, 
    void *buffer, size_t size) {
    spi_bus *bus = (spi_bus *)device_get_operations(dev);
    spi_ioc_transfer msg = {
        .len = (uint16_t)size,
        .rx_buf = buffer,
        .cs_change = bus->cs_change,
        .cs = bus->cs,
        .bits_per_word = bus->bits_per_word,
        .mode = bus->mode,
        .speed_hz = bus->speed_hz,
        .delay_usecs = bus->delay_usecs
    };
    return spi_master_transfer(dev, &msg, 1);
}

#ifdef __cplusplus
}
#endif
#endif /* DRIVER_SPI_H_ */
