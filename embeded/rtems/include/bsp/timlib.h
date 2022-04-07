/*
 * CopyRight 2022 wtcat (Borrow)
 */
#ifndef BSP_TIMLIB_H_
#define BSP_TIMLIB_H_

#include <rtems/score/assert.h>
#include <drvmgr/drvmgr.h>

#ifdef __cplusplus
extern "C"{
#endif

typedef void (*timlib_isr_t)(void *);

struct timlib_ops {
    void (*reset)(struct drvmgr_dev *dev);
    void (*start)(struct drvmgr_dev *dev);
    void (*stop)(struct drvmgr_dev *dev);
    void (*restart)(struct drvmgr_dev *dev);
    int	(*set_freq)(struct drvmgr_dev *dev, uint32_t tickrate);
    int (*get_freq)(struct drvmgr_dev *dev, uint32_t *basefreq,
        uint32_t *tickrate);
    int (*reg_intr)(struct drvmgr_dev *dev, timlib_isr_t isr, void *arg);
    int (*unreg_intr)(struct drvmgr_dev *dev, timlib_isr_t isr, void *arg);
    /* Down-forward count value */
    uint32_t (*get_counter)(struct drvmgr_dev *dev);
    uint32_t (*get_widthmask)(struct drvmgr_dev *dev);
    void (*ack)(struct drvmgr_dev *dev);
    void (*dump)(struct drvmgr_dev *dev);
};

struct timlib_priv {
    const struct timlib_ops *ops;
    timlib_isr_t isr;
    void *arg;
    uint8_t state; // 1: open; 0: closed
};

static inline void timlib_reset(struct drvmgr_dev *dev) {
    _Assert(dev != NULL);
    struct timlib_priv *t = (struct timlib_priv *)dev->priv;
    t->ops->reset(dev);
}

static inline void timlib_start(struct drvmgr_dev *dev) {
    _Assert(dev != NULL);
    struct timlib_priv *t = (struct timlib_priv *)dev->priv;
    t->ops->start(dev);
}

static inline void timlib_stop(struct drvmgr_dev *dev) {
    _Assert(dev != NULL);
    struct timlib_priv *t = (struct timlib_priv *)dev->priv;
    t->ops->stop(dev);
}

static inline void timlib_restart(struct drvmgr_dev *dev) {
    _Assert(dev != NULL);
    struct timlib_priv *t = (struct timlib_priv *)dev->priv;
    t->ops->restart(dev);
}

static inline int timlib_set_freq(struct drvmgr_dev *dev,
    uint32_t tickrate) {
    _Assert(dev != NULL);
    struct timlib_priv *t = (struct timlib_priv *)dev->priv;
    return t->ops->set_freq(dev, tickrate);
}

static inline int timlib_get_freq(struct drvmgr_dev *dev,
    uint32_t *basefreq, uint32_t *tickrate) {
    _Assert(dev != NULL);
    struct timlib_priv *t = (struct timlib_priv *)dev->priv;
    return t->ops->get_freq(dev, basefreq, tickrate);
}

static inline int timlib_unregister_intr(struct drvmgr_dev *dev) {
    _Assert(dev != NULL);
    struct timlib_priv *t = (struct timlib_priv *)dev->priv;
    return t->ops->unreg_intr(dev, t->isr, t->arg);
}

static inline int timlib_register_intr(struct drvmgr_dev *dev,
    void (*isr)(void *), void *arg) {
    _Assert(dev != NULL);
    _Assert(isr != NULL);
    struct timlib_priv *t = (struct timlib_priv *)dev->priv;
    t->ops->unreg_intr(dev, t->isr, t->arg);
    t->isr = isr;
    t->arg = arg;
    return t->ops->reg_intr(dev, isr, arg);
}

static inline int timlib_get_counter(struct drvmgr_dev *dev) {
    _Assert(dev != NULL);
    struct timlib_priv *t = (struct timlib_priv *)dev->priv;
    return t->ops->get_counter(dev);
}

static inline int timlib_get_widthmask(struct drvmgr_dev *dev) {
    _Assert(dev != NULL);
    struct timlib_priv *t = (struct timlib_priv *)dev->priv;
    return t->ops->get_widthmask(dev);
}

static inline void timlib_dump(struct drvmgr_dev *dev) {
    _Assert(dev != NULL);
    struct timlib_priv *t = (struct timlib_priv *)dev->priv;
    if (t->ops->dump)
        t->ops->dump(dev);
}

static inline void timlib_irq_ack(struct drvmgr_dev *dev) {
    _Assert(dev != NULL);
    struct timlib_priv *t = (struct timlib_priv *)dev->priv;
    t->ops->ack(dev);
}

static inline struct drvmgr_dev *timlib_open(const char *name) {
    struct drvmgr_dev *dev = drvmgr_dev_by_name(name);
    if (dev) {
        struct timlib_priv *t = (struct timlib_priv *)dev->priv;
        if (t->state)
            return dev;
        t->state = 1;
        t->ops->reset(dev);
    }
    return dev;
}

static inline void timlib_close(struct drvmgr_dev *dev) {
    struct timlib_priv *t = (struct timlib_priv *)dev->priv;
    if (t->state) {
        timlib_stop(dev);
        timlib_unregister_intr(dev);
        t->state = 0;
    }
}

#ifdef __cplusplus
}
#endif
#endif /* BSP_TIMLIB_H_ */
