/*
 * Copyright(c) 2022 wtcat
 */
#include <stdint.h>
#include <rtems/fatal.h>

/*
 * Symbol referenced by GCC compiler generated code for canary value.
 */
const uintptr_t __stack_chk_guard = 0xDEADBEEF;

void __stack_chk_fail(void) {
    rtems_panic("StackCheckError***: Victim(%p)\n", __builtin_return_address(0));
	__builtin_unreachable();
}



