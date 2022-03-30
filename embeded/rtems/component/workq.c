#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <rtems.h>
#include <rtems/bspIo.h>
#include <rtems/chain.h>
#include <rtems/malloc.h>
#include <rtems/sysinit.h>

#include "component/workq.h"


#define SYSWQ_PRIO  10
#define SYSWQ_STKSZ 4096

struct workqueue *_system_workqueue;

int work_init(struct work_struct *work, 
	void (*worker)(struct work_struct *)) {
	rtems_status_code sc;
	sc = rtems_interrupt_server_request_initialize(_system_workqueue->queue.index, 
		&work->req, (rtems_interrupt_handler)worker, work);
	return rtems_status_code_to_errno(sc);
}

int work_submit_to_queue(struct workqueue *wq, struct work_struct *work) {
	if (!rtems_chain_is_node_off_chain(&work->req.entry.node))
		return -EBUSY;
	work->req.entry.server = &wq->queue;
	RTEMS_COMPILER_MEMORY_BARRIER();
	rtems_interrupt_server_request_submit(&work->req);
	return 0;
}

int work_cancel(struct work_struct *work) {
	if (rtems_chain_is_node_off_chain(&work->req.entry.node))
		return -EINVAL;
	rtems_interrupt_server_entry_destroy(&work->req.entry);
	return 0;
}

static void work_delayed_timer(struct timer_ii *timer) {
	struct work_delayed_struct *work = RTEMS_CONTAINER_OF(timer, 
		struct work_delayed_struct, timer);
	work_submit_to_queue(work->wq, &work->work);
}

int work_delayed_init(struct work_delayed_struct *work,
	void (*worker)(struct work_struct *)) {
	timer_ii_init(&work->timer, work_delayed_timer);
	work_init(&work->work, worker);
	return 0;
}

int work_delayed_sumbit_to_queue(struct workqueue *wq, 
	struct work_delayed_struct *work, uint32_t delay_ms) {
	uint32_t ticks = RTEMS_MILLISECONDS_TO_TICKS(delay_ms);
	if (RTEMS_PREDICT_TRUE(ticks > 0)) {
		work->wq = wq;
		RTEMS_COMPILER_MEMORY_BARRIER();
		timer_ii_mod(&work->timer, ticks);
	} else {
		work_submit_to_queue(wq, &work->work);
	}
	return 0;
}

int work_delayed_cancel(struct work_delayed_struct *work) {
	if (!timer_ii_remove(&work->timer))
		return work_cancel(&work->work);
	return 0;
}

int workqueue_create(struct workqueue **q, size_t stksz, int prio) {
	rtems_interrupt_server_config cfg;
	struct workqueue *wq;
	rtems_status_code sc;
	uint32_t index;

	wq = rtems_calloc(1, sizeof(struct workqueue));
	if (wq == NULL)
		return -ENOMEM;
	memset(&cfg, 0, sizeof(cfg));
	cfg.storage_size = stksz;
	cfg.priority = (rtems_task_priority)prio;
	cfg.name = rtems_build_name('w', 'k', 'q', '@');
	cfg.modes = RTEMS_PREEMPT | RTEMS_NO_ASR | RTEMS_NO_TIMESLICE;
	cfg.attributes = RTEMS_LOCAL;
	sc = rtems_interrupt_server_create(&wq->queue, &cfg, &index);
	if (sc != RTEMS_SUCCESSFUL) {
		free(wq);
		return rtems_status_code_to_errno(sc);
	}
	(void)index;
	*q = wq;
	return 0;
}

static void workq_init(void) {
	int ret;
	ret = workqueue_create(&_system_workqueue, SYSWQ_STKSZ, SYSWQ_PRIO);
	if (ret)
		rtems_panic("Create system workqueue failed!\n");
}

RTEMS_SYSINIT_ITEM(workq_init, 
	RTEMS_SYSINIT_BSP_PRE_DRIVERS, 
	RTEMS_SYSINIT_ORDER_MIDDLE);
