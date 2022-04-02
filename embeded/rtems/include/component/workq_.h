#ifndef COMPONENT_WORKQ_H_
#define COMPONENT_WORKQ_H_

#include <rtems.h>
#include <rtems/scheduler.h>

#include "component/timer_ii.h"

#ifdef __cplusplus
extern "C"{
#endif

#if defined(RTEMS_SMP)
#define SMP_ALIGNMENT CPU_STRUCTURE_ALIGNMENT
#else
#define SMP_ALIGNMENT
#endif


struct workqueue {
#if defined(RTEMS_SMP)
    rtems_interrupt_lock lock;
#endif
    Chain_Control entries;
    rtems_id thread;
} SMP_ALIGNMENT;

struct work_struct {
    Chain_Node node;
    void (*handler)(struct work_struct *work);
};

struct delayed_work_struct {
    struct work_struct work;
    struct timer_ii timer;
    struct workqueue *wq;
};

extern struct workqueue _system_workqueue[];

#define _SYSTEM_WQ  (&_system_workqueue[rtems_scheduler_get_processor()])
#define WORK_INITIALIZER(_handler) \
    {{NULL, NULL}, _handler, 0}

void work_init(struct work_struct *work, 
    void (*handler)(struct work_struct *));
void delayed_work_init(struct delayed_work_struct *work,
    void (*handler)(struct work_struct *));
int schedule_work_to_queue(struct workqueue *wq, 
    struct work_struct *work);
int cancel_queue_work(struct workqueue *wq, struct work_struct *work, 
    bool wait);
int schedule_delayed_work_to_queue(struct workqueue *wq, 
    struct delayed_work_struct *work, uint32_t ticks);
int cancel_queue_delayed_work(struct workqueue *wq, 
    struct work_struct *work, bool wait);
int workqueue_create(struct workqueue *wq, int cpu_index, uint32_t prio, 
    size_t stksize, uint32_t modes, uint32_t attributes);
int workqueue_destory(struct workqueue *wq);
int workqueue_set_cpu_affinity(struct workqueue *wq, const cpu_set_t *affinity, 
    size_t size, uint32_t prio);


static inline int schedule_work(struct work_struct *work) {
    return schedule_work_to_queue(_SYSTEM_WQ, work);
}

static inline int schedule_delayed_work(struct work_struct *work, uint32_t ticks) {
    return schedule_delayed_work_to_queue(_SYSTEM_WQ, work, ticks);
}

static inline int cancel_work_sync(struct work_struct *work) {
    return cancel_queue_work(_SYSTEM_WQ, work, true);
}

static inline int cancel_work_async(struct work_struct *work) {
    return cancel_queue_work(_SYSTEM_WQ, work, false);
}

static inline int cancel_delayed_work_sync(struct work_struct *work) {
    return cancel_queue_delayed_work(_SYSTEM_WQ, work, true);
}

static inline int cancel_delayed_work_async(struct work_struct *work) {
    return cancel_queue_delayed_work(_SYSTEM_WQ, work, false);
}

#ifdef __cplusplus
}
#endif
#endif /* COMPONENT_WORKQ_H_ */

