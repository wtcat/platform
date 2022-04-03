/*
 * CopyRight 2022 wtcat
 */
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <rtems.h>
#include <rtems/chain.h>
#include <rtems/score/assert.h>
#include <rtems/sysinit.h>

#include "component/atomic.h"
#include "component/workqueue.h"
#include "bsp/sysconf.h"

struct helper_work_struct {
    struct work_struct work;
    rtems_id holder;
};

#define SYSWQ_PRIO  10
#define SYSWQ_STKSZ 4096

#define SYSWQ_ATTRIBUTE (RTEMS_LOCAL)
#define SYSWQ_MODES     (RTEMS_PREEMPT | RTEMS_NO_ASR | RTEMS_NO_TIMESLICE)

#define RTEMS_EVENT_WORKQUEUE_SUBMIT    RTEMS_EVENT_20
#define RTEMS_EVENT_WORKQUEUE_TERMINAL  RTEMS_EVENT_21
#define RTEMS_EVENT_WORK_CANCEL         RTEMS_EVENT_22

struct workqueue _system_workqueue[CONFIGURE_MAXIMUM_PROCESSORS];

static void cancel_helper_work(struct work_struct *work) {
    struct helper_work_struct *helper = RTEMS_CONTAINER_OF(work, 
        struct helper_work_struct, work);
    rtems_event_system_send(helper->holder, RTEMS_EVENT_WORK_CANCEL);
}

static void destory_helper_work(struct work_struct *work) {
    struct helper_work_struct *helper = RTEMS_CONTAINER_OF(work, 
        struct helper_work_struct, work);
    rtems_event_system_send(helper->holder, RTEMS_EVENT_WORKQUEUE_TERMINAL);
    rtems_task_exit();
}

static int request_event_helper(struct workqueue *wq, uint32_t events, 
    uint32_t timeout, void (*helper)(struct work_struct *), bool prepend) {
    struct helper_work_struct helper_work;
    rtems_interrupt_lock_context lock_context;
    rtems_status_code sc;
    rtems_event_set evtout;
    helper_work.holder = rtems_task_self();
    work_init(&helper_work.work, helper);
    rtems_interrupt_lock_acquire(&wq->lock, &lock_context);
    if (prepend)
        rtems_chain_prepend_unprotected(&wq->entries, &helper_work.work.node);
    else
        rtems_chain_append_unprotected(&wq->entries, &helper_work.work.node);
    rtems_interrupt_lock_release(&wq->lock, &lock_context);
    rtems_event_system_send(wq->thread, RTEMS_EVENT_WORKQUEUE_SUBMIT);
    sc = rtems_event_system_receive(events, RTEMS_EVENT_ALL | RTEMS_WAIT,
        timeout, &evtout);
    (void)evtout;
    return rtems_status_code_to_errno(sc);
}

static void schedule_worker(rtems_task_argument arg) {
    struct workqueue *wq = (struct workqueue *)arg;
    void (*handler)(struct work_struct *work);
    rtems_interrupt_lock_context lock_context;
    struct work_struct *work;
    for ( ; ; ) {
        rtems_event_set events;
        rtems_event_system_receive(RTEMS_EVENT_WORKQUEUE_SUBMIT,
            RTEMS_EVENT_ALL | RTEMS_WAIT, RTEMS_NO_TIMEOUT, &events);
        (void) events;
        do {
            rtems_interrupt_lock_acquire(&wq->lock, &lock_context);
            if (rtems_chain_is_empty(&wq->entries)) {
                rtems_interrupt_lock_release(&wq->lock, &lock_context);
                break;
            }
            work = (struct work_struct *)rtems_chain_get_first_unprotected(&wq->entries);
            rtems_chain_set_off_chain(&work->node);
            handler = work->handler;
            rtems_interrupt_lock_release(&wq->lock, &lock_context);
            _Assert(handler != NULL);
            /* Execute user work */
            handler(work);
        } while (true);
    }
}

static void schedule_delayed_timer_cb(struct timer_ii *timer) {
	struct delayed_work_struct *work = RTEMS_CONTAINER_OF(timer, 
		struct delayed_work_struct, timer);
	schedule_work_to_queue(work->wq, &work->work);
}

int schedule_work_to_queue(struct workqueue *wq, 
    struct work_struct *work) {
    rtems_interrupt_lock_context lock_context;
    _Assert(wq != NULL);
    _Assert(work != NULL);
    rtems_interrupt_lock_acquire(&wq->lock, &lock_context);
    if (RTEMS_PREDICT_FALSE(!rtems_chain_is_node_off_chain(&work->node))) {
        rtems_interrupt_lock_release(&wq->lock, &lock_context);
        return -EBUSY;
    }
    rtems_chain_append_unprotected(&wq->entries, &work->node);
    rtems_interrupt_lock_release(&wq->lock, &lock_context);
    rtems_event_system_send(wq->thread, RTEMS_EVENT_WORKQUEUE_SUBMIT);
    return 0;
}

int cancel_queue_work(struct workqueue *wq, struct work_struct *work, 
    bool wait) {
    _Assert(wq != NULL);
    _Assert(work != NULL);
    rtems_interrupt_lock_context lock_context;
    rtems_interrupt_lock_acquire(&wq->lock, &lock_context);
    if (rtems_chain_is_node_off_chain(&work->node)) {
        rtems_interrupt_lock_release(&wq->lock, &lock_context);
        return 0;
    }
    rtems_chain_extract_unprotected(&work->node);
    rtems_chain_set_off_chain(&work->node);
    rtems_interrupt_lock_release(&wq->lock, &lock_context);
    if (wait) {
        return request_event_helper(wq, RTEMS_EVENT_WORK_CANCEL, 
            RTEMS_NO_TIMEOUT, cancel_helper_work, true);
    }
    return 0;
}

int schedule_delayed_work_to_queue(struct workqueue *wq, 
    struct delayed_work_struct *work, uint32_t ticks) {
    _Assert(wq != NULL);
    _Assert(work != NULL);
    rtems_interrupt_lock_context lock_context;
    int ret;
    rtems_interrupt_lock_acquire(&wq->lock, &lock_context);
    if (RTEMS_PREDICT_FALSE(!rtems_chain_is_node_off_chain(&work->work.node))) {
        ret = -EBUSY;
        goto _err;
    }
    if (RTEMS_PREDICT_FALSE(ticks == 0)) {
        rtems_chain_append_unprotected(&wq->entries, &work->work.node);
    } else {
        work->wq = wq;
        ret = timer_ii_mod(&work->timer, ticks);
        goto _err;
    }
    rtems_interrupt_lock_release(&wq->lock, &lock_context);
    rtems_event_system_send(wq->thread, RTEMS_EVENT_WORKQUEUE_SUBMIT);
    return 0;
_err:
    rtems_interrupt_lock_release(&wq->lock, &lock_context);
    return ret;
}

int cancel_queue_delayed_work(struct workqueue *wq, 
    struct delayed_work_struct *work, bool wait) {
    _Assert(wq != NULL);
    _Assert(work != NULL);
    if (timer_ii_remove(&work->timer))
        return 0;
    return cancel_queue_work(wq, &work->work, wait);
}

void work_init(struct work_struct *work, 
    void (*handler)(struct work_struct *)) {
    rtems_chain_set_off_chain(&work->node);
    work->handler = handler;
}

void delayed_work_init(struct delayed_work_struct *work,
    void (*handler)(struct work_struct *)) {
    work_init(&work->work, handler);
    timer_ii_init(&work->timer, schedule_delayed_timer_cb);
}

int workqueue_create(struct workqueue *wq, int cpu_index, uint32_t prio, 
    size_t stksize, uint32_t modes, uint32_t attributes) {
    rtems_status_code sc;
#if defined(RTEMS_SMP)
    rtems_id scheduler;
    cpu_set_t cpu;
#endif
    if (wq == NULL)
        return -EINVAL;
    sc = rtems_task_create(rtems_build_name('W', 'Q', '@', 'X'),
        prio, stksize, modes, attributes, &wq->thread);
    if (sc != RTEMS_SUCCESSFUL)
        return -rtems_status_code_to_errno(sc);
    rtems_interrupt_lock_initialize(&wq->lock, "Workqueue");
    rtems_chain_initialize_empty(&wq->entries);
#if defined(RTEMS_SMP)
    sc = rtems_scheduler_ident_by_processor(cpu_index, &scheduler);
    if (sc == RTEMS_SUCCESSFUL) {
        sc = rtems_task_set_scheduler(wq->thread, scheduler, prio);
        _Assert(sc == RTEMS_SUCCESSFUL);
        CPU_ZERO(&cpu);
        CPU_SET(cpu_index, &cpu);
        (void) rtems_task_set_affinity(wq->thread, sizeof(cpu), &cpu);
    }
#else
    (void)cpu_index;
#endif
    sc = rtems_task_start(wq->thread, schedule_worker, (rtems_task_argument)wq);
    _Assert(sc == RTEMS_SUCCESSFUL);
    return -rtems_status_code_to_errno(sc);
}

int workqueue_destory(struct workqueue *wq) {
    return request_event_helper(wq, RTEMS_EVENT_WORKQUEUE_TERMINAL, 
        RTEMS_NO_TIMEOUT, destory_helper_work, false);
}

int workqueue_set_cpu_affinity(struct workqueue *wq, const cpu_set_t *affinity, 
    size_t size, uint32_t prio) {
    rtems_status_code sc;
    rtems_id scheduler;

    sc = rtems_scheduler_ident_by_processor_set(size, affinity, &scheduler);
    if (sc != RTEMS_SUCCESSFUL) 
        return -rtems_status_code_to_errno(sc);;
    sc = rtems_task_set_scheduler(wq->thread, scheduler, prio);
    if (sc != RTEMS_SUCCESSFUL) 
        return -rtems_status_code_to_errno(sc);
    sc = rtems_task_set_affinity(wq->thread, size, affinity);
    return -rtems_status_code_to_errno(sc);
}

static void workqueue_init(void) {
	int cpu_max = rtems_configuration_get_maximum_processors();
	for (int cpu_index = 0; cpu_index < cpu_max; cpu_index++) {
        int ret = workqueue_create(&_system_workqueue[cpu_index], cpu_index, 
            SYSWQ_PRIO, SYSWQ_STKSZ, SYSWQ_MODES, SYSWQ_ATTRIBUTE);
        if (ret)
            rtems_panic("Workqueue initialize failed: %d\n", ret);
	}
}

RTEMS_SYSINIT_ITEM(workqueue_init, 
	RTEMS_SYSINIT_BSP_PRE_DRIVERS, 
	RTEMS_SYSINIT_ORDER_MIDDLE);
