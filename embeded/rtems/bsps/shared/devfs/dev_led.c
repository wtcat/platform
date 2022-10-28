/*
 * Copyright 2022 wtcat
 */
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include <rtems/thread.h>
#include <rtems/imfs.h>
#include <rtems/malloc.h>

#include "devfs/led.h"
#include "drivers/led.h"

struct led_file {
    rtems_mutex lock;
    struct drvmgr_dev *dev;
};

static int led_parse_channel(const char *s, const char **sout) {
    const char *p = strchr(s, ':');
    int sum = 0;

    if (p++) {
        while (*p != '\0') {
            if (!isdigit((int)*p))
                break;
            sum = sum*10 + *p - '0';
            p++;
        }
    }
    if (sout) 
        *sout = (p != NULL)? p: s;
    return sum;
}

static bool led_parse_blink_args(const char *s, uint32_t *delay_on, 
    uint32_t *delay_off) {
    uint32_t t1 = 0, t2 = 0;
    int state = 0;

    if (s == NULL || *s == '\0')
        return false;

    while (*s != '\0' && isspace((int)*s))
        s++;
    while (*s != '\0' && isdigit((int)*s)) {
        if (state == 0)
            state = 1;
        t1 = t1 * 10 + *s - '0';
        s++;
    }

    while (*s != '\0' && isspace((int)*s))
        s++;
    while (*s != '\0' && isdigit((int)*s)) {
        if (state == 1)
            state = 2;
        t2 = t2 * 10 + *s - '0';
        s++;
    }
    if (state != 2)
        return false;
    *delay_on = t1;
    *delay_off = t2;
    return true;
}

static ssize_t led_devfs_write(rtems_libio_t *iop, const void *buffer, size_t count) {
    struct led_file *file = IMFS_generic_get_context_by_iop(iop);
    struct drvmgr_dev *dev = file->dev;
    const char *s;
    int led = 0;
    int err = -EINVAL;

    rtems_mutex_lock(&file->lock);
    if ((s = strstr(buffer, "on"))) {
        led = led_parse_channel(s, NULL);
        err = led_on(dev, led);
    } else if ((s = strstr(buffer, "off"))) {
        led = led_parse_channel(s, NULL);
        err = led_off(dev, led);
    } else if ((s = strstr(buffer, "blink"))) {
        uint32_t delay_on, delay_off;
        const char *endp;
        led = led_parse_channel(s, &endp);
        if (led_parse_blink_args(endp, &delay_on, &delay_off))
            err = led_blink(dev, led, delay_on, delay_off);
        else
            err = -EINVAL;
    }
    rtems_mutex_unlock(&file->lock);
    (void)count;
    return err;
}
  
static int led_devfs_ioctl(rtems_libio_t *iop, ioctl_command_t cmd, void *arg) {
    struct led_file *file = IMFS_generic_get_context_by_iop(iop);
    struct led_ioc *ioc = arg;
    int ch = 0;
    int err;

    rtems_mutex_lock(&file->lock);
    switch (cmd) {
    case LED_IOC_ON:
        if (ioc != NULL)
            ch = ioc->channel;
        err = led_on(file->dev, ch);
        break;
    case LED_IOC_OFF:
        if (ioc != NULL)
            ch = ioc->channel;
        err = led_off(file->dev, ch);
        break;
    case LED_IOC_BLINK:
        if (ioc != NULL) {
            err = led_blink(file->dev, ioc->channel, ioc->delay_on, 
                ioc->delay_off);
            break;
        }
        err = -EINVAL;
        break;
    default:
        err = -EINVAL;
    }
    rtems_mutex_unlock(&file->lock);
    return err;
}

static const rtems_filesystem_file_handlers_r led_file_handler = {
    .open_h = rtems_filesystem_default_open,
    .close_h = rtems_filesystem_default_close,
    .read_h = rtems_filesystem_default_read,
    .write_h = led_devfs_write,
    .ioctl_h = led_devfs_ioctl,
    .lseek_h = rtems_filesystem_default_lseek,
    .fstat_h = IMFS_stat,
    .ftruncate_h = rtems_filesystem_default_ftruncate,
    .fsync_h = rtems_filesystem_default_fsync_or_fdatasync,
    .fdatasync_h = rtems_filesystem_default_fsync_or_fdatasync,
    .fcntl_h = rtems_filesystem_default_fcntl,
    .kqfilter_h = rtems_filesystem_default_kqfilter,
    .mmap_h = rtems_filesystem_default_mmap,
    .poll_h = rtems_filesystem_default_poll,
    .readv_h = rtems_filesystem_default_readv,
    .writev_h = rtems_filesystem_default_writev
};

static IMFS_jnode_t *led_node_generic(IMFS_jnode_t *node, void *arg) {
    return IMFS_node_initialize_generic(node, arg);
}

static void led_node_destroy(IMFS_jnode_t *node) {
    struct led_file *led = IMFS_generic_get_context_by_node(node);
    rtems_mutex_destroy(&led->lock);
    free(led);
    IMFS_node_destroy_default(node);
}

static const IMFS_node_control led_node_control = IMFS_GENERIC_INITIALIZER(
    &led_file_handler,
    led_node_generic,
    led_node_destroy
);

int led_devfs_register(struct drvmgr_dev *dev) {
    struct led_file *led;
    if (dev == NULL)
        return -EINVAL;
    led = rtems_malloc(sizeof(*led));
    if (!led)
        return -ENOMEM;
    led->dev = dev;
    rtems_mutex_init(&led->lock, "ledfs");
    return IMFS_make_generic_node(dev->name,
        S_IFCHR | S_IRWXU | S_IRWXG | S_IRWXO,
        &led_node_control, led);
}
