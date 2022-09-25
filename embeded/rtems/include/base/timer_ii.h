/*
 * CopyRight 2022 wtcat
 */

#ifndef COMPONENT_TIMER_II_H_
#define COMPONENT_TIMER_II_H_

#include <rtems/score/watchdogimpl.h>

#ifdef __cplusplus
extern "C"{
#endif

struct timer_ii {
	Watchdog_Control timer;
};

void timer_ii_init(struct timer_ii *timer, void (*adaptor)(struct timer_ii *));
int timer_ii_add(struct timer_ii *timer, uint32_t expires);
int timer_ii_mod(struct timer_ii *timer, uint32_t expires);
int timer_ii_remove(struct timer_ii *timer);
bool timer_ii_is_pending(struct timer_ii *timer);


#ifdef __cplusplus
}
#endif
#endif /* COMPONENT_TIMER_II_H_ */

