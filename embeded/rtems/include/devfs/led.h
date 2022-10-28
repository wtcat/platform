/*
 * Copyright 2022 wtcat
 */
#ifndef IMFS_DEVFS_LED_H_
#define IMFS_DEVFS_LED_H_

#include <sys/ioccom.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif
#define LED_IOC_MAGIC 0x11


struct led_ioc {
    int channel;
    uint32_t delay_on;
    uint32_t delay_off;
};

#define LED_IOC_ON    _IOW(LED_IOC_MAGIC, 0, struct led_ioc)
#define LED_IOC_OFF   _IOW(LED_IOC_MAGIC, 1, struct led_ioc)
#define LED_IOC_BLINK _IOW(LED_IOC_MAGIC, 2, struct led_ioc)


#ifdef __cplusplus
}
#endif
#endif /* IMFS_DEVFS_LED_H_ */
