/*
 * CopyRight(c) 2022 wtcat
 */

#ifndef DRIVER_I2C_H_
#define DRIVER_I2C_H_

#include <dev/i2c/i2c.h>
#include "drivers/devbase.h"

#ifdef __cplusplus
extern "C"{
#endif

static inline int i2c_master_set_clock(struct drvmgr_dev *dev, 
    unsigned long rate) {
    i2c_bus *bus = (i2c_bus *)dev->priv;
    i2c_bus_obtain(bus);
    int err = (*bus->set_clock)(bus, rate);
    i2c_bus_release(bus);
    return err;
}

static inline ssize_t i2c_master_transfer(struct drvmgr_dev *dev, i2c_msg *msgs,
    uint32_t n, uint32_t flags) {
    i2c_bus *bus = (i2c_bus *)dev->priv;
    return i2c_bus_do_transfer(bus, msgs, n, flags);
}

static inline ssize_t i2c_master_write(struct drvmgr_dev *dev, const void *buffer,
    size_t size, uint16_t addr) {
    i2c_bus *bus = (i2c_bus *)dev->priv;
    i2c_msg msg = {
        .addr = addr,
        .flags = 0,
        .len = (uint16_t)size,
        .buf = RTEMS_DECONST(void *, buffer)
    };
    if (bus->ten_bit_address)
        msg.flags |= I2C_M_TEN;
    int err = i2c_master_transfer(dev, &msg, 1, 0);
    return err == 0? msg.len: -err;
}

static inline ssize_t i2c_master_read(struct drvmgr_dev *dev, void *buffer,
    size_t size, uint16_t addr) {
    i2c_bus *bus = (i2c_bus *)dev->priv;
    i2c_msg msg = {
        .addr = addr,
        .flags = I2C_M_RD,
        .len = (uint16_t)size,
        .buf = buffer
    };
    if (bus->ten_bit_address)
        msg.flags |= I2C_M_TEN;
    int err = i2c_master_transfer(dev, &msg, 1, 0);
    return err == 0? msg.len: -err;
}

#ifdef __cplusplus
}
#endif
#endif /* DRIVER_I2C_H_ */
