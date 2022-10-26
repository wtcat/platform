/*
 * CopyRight(C) 2021-12
 * Author: wtcat
 */
#include <stddef.h>
#include <rtems/rtems/intr.h>
#include <rtems/sysinit.h>

#include "asm/cmsis_cortexm.h"
#include "asm/cortexm_dwt.h"

struct breakpoint_info {
	const char *file;
	int line;
};

#if defined(CORE_CM3) || defined(CORE_CM4) || defined(CORE_CM7)
struct watchpoint_regs {
	volatile uint32_t comp;
	volatile uint32_t mask;
	volatile uint32_t function;
	uint32_t reserved;
};

#elif defined(CORE_CM33) || defined(CORE_CM23)
struct watchpoint_regs {
	volatile uint32_t comp;
	uint32_t reserved_1;
	volatile uint32_t function;
	uint32_t reserved_2;
};

#else
#error "Unknown CPU architecture"
#endif /* CORE_CM_ */

#define _CPU_MUTEX_LOCK(_lock)   rtems_interrupt_enable(_lock)
#define _CPU_MUTEX_UNLOCK(_lock) rtems_interrupt_disable(_lock)

#define WATCHPOINT_REGADDR(member) \
	(struct watchpoint_regs *)(DWT_BASE + offsetof(DWT_Type, member));
#define WATCHPOINT_MAX_NR \
	((DWT->CTRL & DWT_CTRL_NUMCOMP_Msk) >> DWT_CTRL_NUMCOMP_Pos)
#define WATCHPOINT_REG() WATCHPOINT_REGADDR(COMP0)

static int _watchpoint_nums;
static struct breakpoint_info _breakpoint[MAX_BREAKPOINT_NR];

#if defined(CORE_CM7)
#ifndef DWT_LSR_Present_Msk
#define DWT_LSR_Present_Msk ITM_LSR_Present_Msk
#endif
#ifndef DWT_LSR_Access_Msk
#define DWT_LSR_Access_Msk ITM_LSR_Access_Msk
#endif

static void __core_debug_access(int ena) {
	uint32_t lsr = DWT->LSR;
	if ((lsr & DWT_LSR_Present_Msk) != 0) {
		if (!!ena) {
			if ((lsr & DWT_LSR_Access_Msk) != 0) 
				DWT->LAR = 0xC5ACCE55; /* unlock it */
		} else {
			if ((lsr & DWT_LSR_Access_Msk) == 0) 
				DWT->LAR = 0; /* Lock it */
		}
	}
}
#endif

static int cortexm_dwt_debug_init(void) {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(CORE_CM7)
	__core_debug_access(1);
#endif
	return 0;
}

static int cortexm_dwt_debug_enable(void) {
	 if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
	 	return -EBUSY;
#if defined(CONFIG_ARMV8_M_SE) && !defined(CONFIG_ARM_NONSECURE_FIRMWARE)
	if (!(CoreDebug->DEMCR & DCB_DEMCR_SDME_Msk))
		return -EBUSY;
#endif
	CoreDebug->DEMCR |= CoreDebug_DEMCR_MON_EN_Msk;
	return 0;
}

int cortexm_dwt_watchpoint_enable(int nr, void *addr, unsigned int mode, 
	const char *file, int line) {
	if (nr > _watchpoint_nums)
		return -EINVAL;
	struct watchpoint_regs *wp_regs = WATCHPOINT_REG();
	uint32_t func = (mode & 0xF) << DWT_FUNCTION_DATAVSIZE_Pos;
    uint32_t level;
	_CPU_MUTEX_LOCK(level);
	_breakpoint[nr].file = file;
	_breakpoint[nr].line = line;
#if defined(CORE_CM3) || defined(CORE_CM4) || defined(CORE_CM7)
	if (mode & MEM_READ)
		func |= (0x6 << DWT_FUNCTION_FUNCTION_Pos);
	if (mode & MEM_WRITE)
		func |= (0x5 << DWT_FUNCTION_FUNCTION_Pos);	
	wp_regs[nr].comp = (uint32_t)addr;
	wp_regs[nr].mask = 0;
	wp_regs[nr].function = func;
	
#elif defined(CORE_CM33) || defined(CORE_CM23)
	if (mode & MEM_READ)
		func |= (0x5 << DWT_FUNCTION_MATCH_Pos);
	if (mode & MEM_WRITE)
		func |= (0x6 << DWT_FUNCTION_MATCH_Pos);	
	func |= (0x1 << DWT_FUNCTION_ACTION_Pos);
	wp_regs[nr].comp = (uint32_t)addr;
	wp_regs[nr].function = func;
#endif
	_CPU_MUTEX_UNLOCK(level);
    return 0;
}

int cortexm_dwt_watchpoint_disable(int nr) {
	if (nr > _watchpoint_nums)
		return -EINVAL;
	struct watchpoint_regs *wp_regs = WATCHPOINT_REG();
    uint32_t level;
	_CPU_MUTEX_LOCK(level);
	wp_regs[nr].function = 0;
	_breakpoint[nr].file = NULL;
	_CPU_MUTEX_UNLOCK(level);
	return 0;
}

int _core_debug_watchpoint_busy(int nr) {
	struct watchpoint_regs *wp_regs = WATCHPOINT_REG();
	uint32_t func = wp_regs[nr].function;
	return func != 0;
}


int _core_debug_init_cycle_counter(void) {
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	return (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0;
}

uint32_t _core_debug_get_cycles(void) {
	return DWT->CYCCNT;
}

void _core_debug_cycle_count_start(void) {
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void cortexm_dwt_setup(void) {
	_watchpoint_nums = WATCHPOINT_MAX_NR;
	cortexm_dwt_debug_init();
	for (int i = 0; i < _watchpoint_nums; i++)
		cortexm_dwt_watchpoint_disable(i);
	cortexm_dwt_debug_enable();
}

RTEMS_SYSINIT_ITEM(cortexm_dwt_setup,
	RTEMS_SYSINIT_BSP_START,
	RTEMS_SYSINIT_ORDER_LAST
);
