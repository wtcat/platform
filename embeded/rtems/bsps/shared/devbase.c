/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <rtems/bspIo.h>
#include <sys/param.h>

#include "drivers/devbase.h"
#include "base/init.h"

static struct drv_posthook *drv_post_list;

struct drvmgr_dev *device_add(struct drvmgr_dev *parent, 
    const struct drvmgr_bus_ops *bus_ops, 
    int bustype, const char *name, size_t devp_size, size_t priv_size) {
    struct drvmgr_dev *child;
    char *pname;
    int err;

    if (parent == NULL || name == NULL) {
        errno = -EINVAL;
        return NULL;
    }
    if (parent->bus == NULL) {
        if (bus_ops == NULL) {
            errno = -EINVAL;
            return NULL;
        }
        drvmgr_alloc_bus(&parent->bus, 0);
        parent->bus->ops = (void *)bus_ops;
        parent->bus->dev = parent;
        parent->bus->bus_type = bustype;
        err = drvmgr_bus_register(parent->bus);
        if (err) {
            errno = err;
            printk("%s: register bus failed(%d)\n", __func__, err);
            return NULL;
        }
    }
    size_t len = strlen(name);
    devp_size = roundup(devp_size, sizeof(void *));
    drvmgr_alloc_dev(&child, devp_size + priv_size + len + 1);
    void *devp = device_get_private(child);
    if (priv_size > 0)
        child->priv = (char *)devp + devp_size;
    else
        child->priv = NULL;
    pname = (char *)devp + devp_size + priv_size;
    memcpy(pname, name, len);
    child->parent = parent->bus;
    child->name = pname;
    err = drvmgr_dev_register(child);
    if (err) {
        errno = err;
        printk("%s: register device(%s) failed\n", __func__, child->name);
        return NULL;
    }
    return child;
}

int device_delete(struct drvmgr_dev *parent, struct drvmgr_dev *dev) {
    int err = drvmgr_dev_unregister(dev);
    (void) parent;
    return err;
}

int driver_register_posthook(struct drv_posthook *hook) {
    struct drv_posthook **iter;

    if (!hook || !hook->run)
        return -EINVAL;
    iter = &drv_post_list;
    while (*iter)
        iter = &(*iter)->next;
    *iter = hook;
    hook->next = NULL;
    return 0;
}

static void driver_post_run(void) {
    struct drv_posthook *iter = drv_post_list;
    while (iter) {
        if (iter->run)
            iter->run(iter->arg);
        iter = iter->next;
    }
}

SYSINIT_ITEM(driver_post_run, 
    SYSINIT_INIT_BEGIN, RTEMS_SYSINIT_ORDER_MIDDLE);