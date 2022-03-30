#ifndef COMPONENT_WORKQ_H_
#define COMPONENT_WORKQ_H_

#include <rtems/irq-extension.h>

#include "component/timer_ii.h"

#ifdef __cplusplus
extern "C"{
#endif

struct workqueue {
	rtems_interrupt_server_control queue;
};

struct work_struct {
	rtems_interrupt_server_request req;
};

struct work_delayed_struct {
	struct work_struct work;
	struct timer_ii timer;
	struct workqueue *wq;
};

extern struct workqueue *_system_workqueue;

int work_init(struct work_struct *work, void (*worker)(struct work_struct *));
int work_submit_to_queue(struct workqueue *wq, struct work_struct *work);
int work_cancel(struct work_struct *work);
int work_delayed_init(struct work_delayed_struct *work,
	void (*worker)(struct work_struct *));
int work_delayed_sumbit_to_queue(struct workqueue *wq, 
	struct work_delayed_struct *work, uint32_t delay_ms);
int work_delayed_cancel(struct work_delayed_struct *work);
int workqueue_create(struct workqueue **q, size_t stksz, int prio);

static inline int work_submit(struct work_struct *work) {
	return work_submit_to_queue(_system_workqueue, work);
}

static inline int work_delayed_submit(struct work_delayed_struct *work, 
	uint32_t delay) {
	return work_delayed_sumbit_to_queue(_system_workqueue, work, delay);
}


#ifdef __cplusplus
}
#endif
#endif /* COMPONENT_WORKQ_H_ */

