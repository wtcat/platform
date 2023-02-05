/*
 * CopyRight 2022 wtcat
 */

#ifndef COMPONENT_TIMER_II_H_
#define COMPONENT_TIMER_II_H_

#include <rtems/score/watchdogimpl.h>

#ifdef __cplusplus
extern "C"{
#endif

struct timer_list {
	Watchdog_Control timer;
};

void timer_init(struct timer_list *timer, void (*adaptor)(struct timer_list *));
int timer_add(struct timer_list *timer, uint32_t expires);
int timer_mod(struct timer_list *timer, uint32_t expires);
int timer_del(struct timer_list *timer);
bool timer_is_pending(struct timer_list *timer);


#ifdef __cplusplus
}
#endif
#endif /* COMPONENT_TIMER_II_H_ */

