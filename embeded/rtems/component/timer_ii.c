/*
 * CopyRight 2022 wtcat
 */
#include "base/timer_ii.h"
#include "base/sections.h"

static inline 
Per_CPU_Control * __fastcode timer_ii_lock(struct timer_ii *timer,
	ISR_lock_Context *lock_context) {
	Per_CPU_Control *cpu;
	_ISR_lock_ISR_disable(lock_context);
	cpu = _Watchdog_Get_CPU(&timer->timer);
	_Watchdog_Per_CPU_acquire_critical(cpu, lock_context);
	return cpu;
}

static inline void __fastcode timer_ii_unlock(Per_CPU_Control *cpu,
  ISR_lock_Context *lock_context) {
	_Watchdog_Per_CPU_release_critical(cpu, lock_context);
	_ISR_lock_ISR_enable(lock_context);
}

static inline int __fastcode timer_ii_add_locked(Per_CPU_Control *cpu, 
	struct timer_ii *timer, uint32_t expires) {
	_Watchdog_Insert(&cpu->Watchdog.Header[PER_CPU_WATCHDOG_TICKS],
		&timer->timer, cpu->Watchdog.ticks + expires);
	return 0;
}

static inline int __fastcode timer_ii_remove_locked(Per_CPU_Control *cpu, 
	struct timer_ii *timer) {
	if (_Watchdog_Is_scheduled(&timer->timer)) {
		_Watchdog_Remove(&cpu->Watchdog.Header[PER_CPU_WATCHDOG_TICKS], 
			&timer->timer);
		return 1;
	}
	return 0;
}

void __fastcode timer_ii_init(struct timer_ii *timer, void (*adaptor)(struct timer_ii *)) {
	_Watchdog_Preinitialize(&timer->timer, _Per_CPU_Get_snapshot());
	_Watchdog_Initialize(&timer->timer, (Watchdog_Service_routine_entry)adaptor);
}

int __fastcode timer_ii_add(struct timer_ii *timer, uint32_t expires) {
	ISR_lock_Context lock_context;
	int ret;
	Per_CPU_Control *cpu = timer_ii_lock(timer, &lock_context);
	_Assert(cpu != NULL);
	ret = timer_ii_add_locked(cpu, timer, expires);
	timer_ii_unlock(cpu, &lock_context);
	return ret;
}

int __fastcode timer_ii_mod(struct timer_ii *timer, uint32_t expires) {
	ISR_lock_Context lock_context;
	int ret;
	Per_CPU_Control *cpu = timer_ii_lock(timer, &lock_context);
	_Assert(cpu != NULL);
	timer_ii_remove_locked(cpu, timer);
	ret = timer_ii_add_locked(cpu, timer, expires);
	timer_ii_unlock(cpu, &lock_context);
	return ret;	
}

int __fastcode timer_ii_remove(struct timer_ii *timer) {
	ISR_lock_Context lock_context;
	int ret;
	Per_CPU_Control *cpu = timer_ii_lock(timer, &lock_context);
	_Assert(cpu != NULL);
	ret = timer_ii_remove_locked(cpu, timer);
	timer_ii_unlock(cpu, &lock_context);
	return ret;
}

bool __fastcode timer_ii_is_pending(struct timer_ii *timer) {
	ISR_lock_Context lock_context;
	bool pending;
	Per_CPU_Control *cpu = timer_ii_lock(timer, &lock_context);
	_Assert(cpu != NULL);
	pending = _Watchdog_Is_scheduled(&timer->timer);
	timer_ii_unlock(cpu, &lock_context);
	return pending;
}
