/*
 * CopyRight(C) 2021-12
 * Author: wtcat
 */
#ifndef CORTEXM_DWT_H_
#define CORTEXM_DWT_H_

#include <errno.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif

#define MAX_BREAKPOINT_NR 4
/* 
 * Watchpoint mode 
 */
#define MEM_READ  0x0100  /* */
#define MEM_WRITE 0x0200
#define MEM_RW_MASK (MEM_READ | MEM_WRITE)
	
#define MEM_SIZE_1 0  /* 1 Byte */
#define MEM_SIZE_2 1  /* 2 Bytes */
#define MEM_SIZE_4 2  /* 4 Bytes */
#define MEM_SIZE_MASK 0xF

/*
 * Protected interface
 */
int _core_debug_watchpoint_enable(int nr, void *addr, unsigned int mode, 
	const char *file, int line);
int _core_debug_watchpoint_disable(int nr);
int _core_debug_watchpoint_busy(int nr);

/*
 * Public interface
 */
#define core_watchpoint_install(_nr, _addr, _mode) \
	_core_debug_watchpoint_enable(_nr, (void *)_addr, _mode, __FILE__, __LINE__)
	
#define core_watchpoint_uninstall(_nr) \
	_core_debug_watchpoint_disable(_nr)

int _core_debug_init_cycle_counter(void);
void _core_debug_cycle_count_start(void);
uint32_t _core_debug_get_cycles(void);

#ifdef __cplusplus
}
#endif
#endif /* CORTEXM_DWT_H_ */
