extern "C" int flsl(long i);
#include "drivers/gpio.h"

#include "gtest/gtest.h"

extern "C" struct drvmgr_dev *drvmgr_dev_by_name(const char *name);

TEST(tpic2810, write) {
    struct drvmgr_dev *dev;
    int err;
    dev = drvmgr_dev_by_name("tpic2810");
    ASSERT_TRUE(dev != nullptr);
    ASSERT_TRUE(dev->priv != nullptr);
    int value = 0xFF;
    while (value) {
        err = gpiod_write(dev, 0xFF, value);
        ASSERT_TRUE(err > 0);
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(500));
        value >>= 1;
    }
    value = 7;
    err = gpiod_write(dev, 0xFF, 0xFF);
    ASSERT_TRUE(err > 0);
    while (value >= 0) {
        err = gpiod_setpin(dev, value, 0);
        ASSERT_TRUE(err > 0);
        rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(300));
        value--;
    }
}