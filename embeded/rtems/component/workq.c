#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <rtems.h>
#include <rtems/bspIo.h>
#include <rtems/chain.h>
#include <rtems/malloc.h>
#include <rtems/sysinit.h>

#include "component/workq.h"
#include "bsp/sysconf.h"


#define SYSWQ_PRIO  10
#define SYSWQ_STKSZ 4096

struct workqueue _system_workqueue[CONFIGURE_MAXIMUM_PROCESSORS];

int work_init(struct work_struct *work, 
	void (*worker)(struct work_struct *)) {
	rtems_status_code sc;
	sc = rtems_interrupt_server_request_initialize(_system_workqueue[0].queue.index, 
		&work->req, (rtems_interrupt_handler)worker, work);
	return rtems_status_code_to_errno(sc);
}

int work_submit_to_queue(struct workqueue *wq, struct work_struct *work) {
	if (RTEMS_PREDICT_FALSE(wq == NULL || work == NULL))
		return -EINVAL; 
	work->req.entry.server = &wq->queue;
	rtems_interrupt_server_request_submit(&work->req);
	return 0;
}

int work_cancel(struct work_struct *work) {
	if (RTEMS_PREDICT_FALSE(work == NULL))
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
	if (RTEMS_PREDICT_FALSE(wq == NULL || work == NULL))
		return -EINVAL; 
	if (RTEMS_PREDICT_TRUE(ticks > 0)) {
		work->wq = wq;
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

int workqueue_create(struct workqueue *wq, size_t stksz, int prio) {
	rtems_interrupt_server_config cfg;
	rtems_status_code sc;
	uint32_t index;
	if (wq == NULL)
		return -EINVAL;
	memset(&cfg, 0, sizeof(cfg));
	cfg.storage_size = stksz;
	cfg.priority = (rtems_task_priority)prio;
	cfg.name = rtems_build_name('W', 'Q', '@', '0');
	cfg.modes = RTEMS_PREEMPT | RTEMS_NO_ASR | RTEMS_NO_TIMESLICE;
	cfg.attributes = RTEMS_LOCAL;
	sc = rtems_interrupt_server_create(&wq->queue, &cfg, &index);
	if (sc != RTEMS_SUCCESSFUL) 
		return rtems_status_code_to_errno(sc);
	(void)index;
	return 0;
}

int workqueue_destory(struct workqueue *wq) {
	if (wq != NULL) {
		uint32_t server = wq->queue.index;
		rtems_status_code sc;
		sc = rtems_interrupt_server_delete(server);
		return rtems_status_code_to_errno(sc);
	}
	return -EINVAL;
}

static void workq_init(void) {
	int cpu_max = rtems_configuration_get_maximum_processors();
	rtems_status_code sc;
	int ret;
	for (int cpu_index = 0; cpu_index < cpu_max; cpu_index++) {
		cpu_set_t cpu;
		ret = workqueue_create(&_system_workqueue[cpu_index], SYSWQ_STKSZ, SYSWQ_PRIO);
		if (!ret) {
		#if defined(RTEMS_SMP)
			rtems_id task = _system_workqueue[cpu_index].queue.server;
			rtems_id scheduler;
			sc = rtems_scheduler_ident_by_processor(cpu_index, &scheduler);
			if (sc != RTEMS_SUCCESSFUL)
				rtems_panic("Get scheduler failed: %s!\n", rtems_status_text(sc));
			sc = rtems_task_set_scheduler(task, scheduler, SYSWQ_PRIO);
			_Assert(sc == RTEMS_SUCCESSFUL);
			CPU_ZERO(&cpu);
			CPU_SET(cpu_index, &cpu);
			rtems_task_set_affinity(task, sizeof(cpu), &cpu);
			(void) sc;
		#endif
		} else {
			rtems_panic("Create system workqueue failed!\n");
		}
	}
}

RTEMS_SYSINIT_ITEM(workq_init, 
	RTEMS_SYSINIT_BSP_PRE_DRIVERS, 
	RTEMS_SYSINIT_ORDER_MIDDLE);
