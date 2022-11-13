/*
 * This file is part of the CmBacktrace Library.
 *
 * Copyright (c) 2016-2019, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Initialize function and other general function.
 * Created on: 2016-12-15
 */
 
 /*
  * Copyright 2022 wtcat
  */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <rtems/score/armv7m.h>
#include <rtems/score/percpu.h>
#include <rtems/score/threadimpl.h>
#include <rtems/bspIo.h>
#include <rtems/sysinit.h>

#include <bsp/linker-symbols.h>
#include "base/compiler.h"
#include "asm/cortexm_trace.h"


#if defined(CORE_CM4)
#define CMB_CPU_PLATFORM_TYPE CMB_CPU_ARM_CORTEX_M4
#elif defined(CORE_CM7)
#define CMB_CPU_PLATFORM_TYPE CMB_CPU_ARM_CORTEX_M7
#else
#error "Invalid CPU type!!!"
#endif

// #define CMB_USING_DUMP_STACK_INFO 1

/* CPU platform list*/
#define CMB_CPU_ARM_CORTEX_M4  4
#define CMB_CPU_ARM_CORTEX_M7  7
#define CMB_CPU_ARM_CORTEX_M33 33

#ifndef CMB_CALL_STACK_MAX_DEPTH
#define CMB_CALL_STACK_MAX_DEPTH  16
#endif

#define CMB_ASSERT(EXPR)  _Assert(EXPR)
#define cmb_println(fmt, ...) \
do { \
    printk(fmt, ##__VA_ARGS__); \
    rtems_putc('\n'); \
} while (0)


/**
 * Cortex-M fault registers
 */
#define CMB_SYSHND_CTRL   (*(volatile unsigned int*)  (0xE000ED24u)) /* system handler control and state register */
#define CMB_NVIC_MFSR     (*(volatile unsigned char*) (0xE000ED28u)) /* memory management fault status register */
#define CMB_NVIC_BFSR     (*(volatile unsigned char*) (0xE000ED29u)) /* bus fault status register */
#define CMB_NVIC_UFSR     (*(volatile unsigned short*)(0xE000ED2Au)) /* usage fault status register */
#define CMB_NVIC_HFSR     (*(volatile unsigned int*)  (0xE000ED2Cu)) /* hard fault status register */
#define CMB_NVIC_DFSR     (*(volatile unsigned short*)(0xE000ED30u)) /* debug fault status register */
#define CMB_NVIC_MMAR     (*(volatile unsigned int*)  (0xE000ED34u)) /* memory management fault address register */
#define CMB_NVIC_BFAR     (*(volatile unsigned int*)  (0xE000ED38u)) /* bus fault manage address register */
#define CMB_NVIC_AFSR     (*(volatile unsigned short*)(0xE000ED3Cu)) /* auxiliary fault status register */

struct cmb_hard_fault_regs{
  struct {
    unsigned int r0;                     // Register R0
    unsigned int r1;                     // Register R1
    unsigned int r2;                     // Register R2
    unsigned int r3;                     // Register R3
    unsigned int r12;                    // Register R12
    unsigned int lr;                     // Link register
    unsigned int pc;                     // Program counter
    union {
      unsigned int value;
      struct {
#if (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M33)
        unsigned int IPSR : 9;           // Interrupt Program Status register (IPSR)
        unsigned int EPSR : 18;          // Execution Program Status register (EPSR)
        unsigned int APSR : 5;           // Application Program Status register (APSR)
#else
        unsigned int IPSR : 8;           // Interrupt Program Status register (IPSR)
        unsigned int EPSR : 19;          // Execution Program Status register (EPSR)
        unsigned int APSR : 5;           // Application Program Status register (APSR)
#endif
      } bits;
    } psr;                               // Program status register.
  } saved;

  union {
    unsigned int value;
    struct {
      unsigned int MEMFAULTACT    : 1;   // Read as 1 if memory management fault is active
      unsigned int BUSFAULTACT    : 1;   // Read as 1 if bus fault exception is active
#if (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M33)
      unsigned int HARDFAULTACT   : 1;   // Read as 1 if hardfault is active
#else
      unsigned int UnusedBits1    : 1;
#endif
      unsigned int USGFAULTACT    : 1;   // Read as 1 if usage fault exception is active
#if (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M33)
      unsigned int SECUREFAULTACT : 1;   // Read as 1 if secure fault exception is active
      unsigned int NMIACT         : 1;   // Read as 1 if NMI exception is active
      unsigned int UnusedBits2    : 1;
#else
      unsigned int UnusedBits2    : 3;
#endif
      unsigned int SVCALLACT      : 1;   // Read as 1 if SVC exception is active
      unsigned int MONITORACT     : 1;   // Read as 1 if debug monitor exception is active
      unsigned int UnusedBits3    : 1;
      unsigned int PENDSVACT      : 1;   // Read as 1 if PendSV exception is active
      unsigned int SYSTICKACT     : 1;   // Read as 1 if SYSTICK exception is active
      unsigned int USGFAULTPENDED : 1;   // Usage fault pended; usage fault started but was replaced by a higher-priority exception
      unsigned int MEMFAULTPENDED : 1;   // Memory management fault pended; memory management fault started but was replaced by a higher-priority exception
      unsigned int BUSFAULTPENDED : 1;   // Bus fault pended; bus fault handler was started but was replaced by a higher-priority exception
      unsigned int SVCALLPENDED   : 1;   // SVC pended; SVC was started but was replaced by a higher-priority exception
      unsigned int MEMFAULTENA    : 1;   // Memory management fault handler enable
      unsigned int BUSFAULTENA    : 1;   // Bus fault handler enable
      unsigned int USGFAULTENA    : 1;   // Usage fault handler enable
#if (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M33)
      unsigned int SECUREFAULTENA : 1;   // Secure fault handler enable
      unsigned int SECUREFAULTPENDED : 1;   // Secure fault pended; Secure fault handler was started but was replaced by a higher-priority exception
      unsigned int HARDFAULTPENDED   : 1;   // Hard fault pended; Hard fault handler was started but was replaced by a higher-priority exception
#else
      // None
#endif
    } bits;
  } syshndctrl;                          // System Handler Control and State Register (0xE000ED24)

  union {
    unsigned char value;
    struct {
      unsigned char IACCVIOL    : 1;     // Instruction access violation
      unsigned char DACCVIOL    : 1;     // Data access violation
      unsigned char UnusedBits  : 1;
      unsigned char MUNSTKERR   : 1;     // Unstacking error
      unsigned char MSTKERR     : 1;     // Stacking error
      unsigned char MLSPERR     : 1;     // Floating-point lazy state preservation (M4/M7)
      unsigned char UnusedBits2 : 1;
      unsigned char MMARVALID   : 1;     // Indicates the MMAR is valid
    } bits;
  } mfsr;                                // Memory Management Fault Status Register (0xE000ED28)
  unsigned int mmar;                     // Memory Management Fault Address Register (0xE000ED34)

  union {
    unsigned char value;
    struct {
      unsigned char IBUSERR    : 1;      // Instruction access violation
      unsigned char PRECISERR  : 1;      // Precise data access violation
      unsigned char IMPREISERR : 1;      // Imprecise data access violation
      unsigned char UNSTKERR   : 1;      // Unstacking error
      unsigned char STKERR     : 1;      // Stacking error
      unsigned char LSPERR     : 1;      // Floating-point lazy state preservation (M4/M7)
      unsigned char UnusedBits : 1;
      unsigned char BFARVALID  : 1;      // Indicates BFAR is valid
    } bits;
  } bfsr;                                // Bus Fault Status Register (0xE000ED29)
  unsigned int bfar;                     // Bus Fault Manage Address Register (0xE000ED38)

  union {
    unsigned short value;
    struct {
      unsigned short UNDEFINSTR : 1;     // Attempts to execute an undefined instruction
      unsigned short INVSTATE   : 1;     // Attempts to switch to an invalid state (e.g., ARM)
      unsigned short INVPC      : 1;     // Attempts to do an exception with a bad value in the EXC_RETURN number
      unsigned short NOCP       : 1;     // Attempts to execute a coprocessor instruction
#if (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M33)
      unsigned short STKOF      : 1;     // Indicates a stack overflow error has occured
      unsigned short UnusedBits : 3;
#else
      unsigned short UnusedBits : 4;
#endif
      unsigned short UNALIGNED  : 1;     // Indicates that an unaligned access fault has taken place
      unsigned short DIVBYZERO0 : 1;     // Indicates a divide by zero has taken place (can be set only if DIV_0_TRP is set)
    } bits;
  } ufsr;                                // Usage Fault Status Register (0xE000ED2A)

  union {
    unsigned int value;
    struct {
      unsigned int UnusedBits  : 1;
      unsigned int VECTBL      : 1;      // Indicates hard fault is caused by failed vector fetch
      unsigned int UnusedBits2 : 28;
      unsigned int FORCED      : 1;      // Indicates hard fault is taken because of bus fault/memory management fault/usage fault
      unsigned int DEBUGEVT    : 1;      // Indicates hard fault is triggered by debug event
    } bits;
  } hfsr;                                // Hard Fault Status Register (0xE000ED2C)

  union {
    unsigned int value;
    struct {
      unsigned int HALTED   : 1;         // Halt requested in NVIC
      unsigned int BKPT     : 1;         // BKPT instruction executed
      unsigned int DWTTRAP  : 1;         // DWT match occurred
      unsigned int VCATCH   : 1;         // Vector fetch occurred
      unsigned int EXTERNAL : 1;         // EDBGRQ signal asserted
    } bits;
  } dfsr;                                // Debug Fault Status Register (0xE000ED30)

  unsigned int afsr;                     // Auxiliary Fault Status Register (0xE000ED3C), Vendor controlled (optional)
};

enum {
    PRINT_MAIN_STACK_CFG_ERROR,
    PRINT_FIRMWARE_INFO,
    PRINT_ASSERT_ON_THREAD,
    PRINT_ASSERT_ON_HANDLER,
    PRINT_THREAD_STACK_INFO,
    PRINT_MAIN_STACK_INFO,
    PRINT_THREAD_STACK_OVERFLOW,
    PRINT_MAIN_STACK_OVERFLOW,
    PRINT_CALL_STACK_INFO,
    PRINT_CALL_STACK_ERR,
    PRINT_FAULT_ON_THREAD,
    PRINT_FAULT_ON_HANDLER,
    PRINT_REGS_TITLE,
    PRINT_HFSR_VECTBL,
    PRINT_MFSR_IACCVIOL,
    PRINT_MFSR_DACCVIOL,
    PRINT_MFSR_MUNSTKERR,
    PRINT_MFSR_MSTKERR,
    PRINT_MFSR_MLSPERR,
    PRINT_BFSR_IBUSERR,
    PRINT_BFSR_PRECISERR,
    PRINT_BFSR_IMPREISERR,
    PRINT_BFSR_UNSTKERR,
    PRINT_BFSR_STKERR,
    PRINT_BFSR_LSPERR,
    PRINT_UFSR_UNDEFINSTR,
    PRINT_UFSR_INVSTATE,
    PRINT_UFSR_INVPC,
    PRINT_UFSR_NOCP,
#if (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M33)
    PRINT_UFSR_STKOF,
#endif
    PRINT_UFSR_UNALIGNED,
    PRINT_UFSR_DIVBYZERO0,
    PRINT_DFSR_HALTED,
    PRINT_DFSR_BKPT,
    PRINT_DFSR_DWTTRAP,
    PRINT_DFSR_VCATCH,
    PRINT_DFSR_EXTERNAL,
    PRINT_MMAR,
    PRINT_BFAR,
};

static const char * const print_info[] = {
    [PRINT_MAIN_STACK_CFG_ERROR]  = "ERROR: Unable to get the main stack information, please check the configuration of the main stack",
    [PRINT_FIRMWARE_INFO]         = "Firmware name: %s, hardware version: %s, software version: %s",
    [PRINT_ASSERT_ON_THREAD]      = "Assert on thread %s",
    [PRINT_ASSERT_ON_HANDLER]     = "Assert on interrupt or bare metal(no OS) environment",
    [PRINT_THREAD_STACK_INFO]     = "===== Thread stack information =====",
    [PRINT_MAIN_STACK_INFO]       = "====== Main stack information ======",
    [PRINT_THREAD_STACK_OVERFLOW] = "Error: Thread stack(%08x) was overflow",
    [PRINT_MAIN_STACK_OVERFLOW]   = "Error: Main stack(%08x) was overflow",
    [PRINT_CALL_STACK_INFO]       = "Show more call stack info by run: addr2line -e %s%s -a -f %s",
    [PRINT_CALL_STACK_ERR]        = "Dump call stack has an error",
    [PRINT_FAULT_ON_THREAD]       = "Fault on thread %s",
    [PRINT_FAULT_ON_HANDLER]      = "Fault on interrupt or bare metal(no OS) environment",
    [PRINT_REGS_TITLE]            = "=================== Registers information ====================",
    [PRINT_HFSR_VECTBL]           = "Hard fault is caused by failed vector fetch",
    [PRINT_MFSR_IACCVIOL]         = "Memory management fault is caused by instruction access violation",
    [PRINT_MFSR_DACCVIOL]         = "Memory management fault is caused by data access violation",
    [PRINT_MFSR_MUNSTKERR]        = "Memory management fault is caused by unstacking error",
    [PRINT_MFSR_MSTKERR]          = "Memory management fault is caused by stacking error",
    [PRINT_MFSR_MLSPERR]          = "Memory management fault is caused by floating-point lazy state preservation",
    [PRINT_BFSR_IBUSERR]          = "Bus fault is caused by instruction access violation",
    [PRINT_BFSR_PRECISERR]        = "Bus fault is caused by precise data access violation",
    [PRINT_BFSR_IMPREISERR]       = "Bus fault is caused by imprecise data access violation",
    [PRINT_BFSR_UNSTKERR]         = "Bus fault is caused by unstacking error",
    [PRINT_BFSR_STKERR]           = "Bus fault is caused by stacking error",
    [PRINT_BFSR_LSPERR]           = "Bus fault is caused by floating-point lazy state preservation",
    [PRINT_UFSR_UNDEFINSTR]       = "Usage fault is caused by attempts to execute an undefined instruction",
    [PRINT_UFSR_INVSTATE]         = "Usage fault is caused by attempts to switch to an invalid state (e.g., ARM)",
    [PRINT_UFSR_INVPC]            = "Usage fault is caused by attempts to do an exception with a bad value in the EXC_RETURN number",
    [PRINT_UFSR_NOCP]             = "Usage fault is caused by attempts to execute a coprocessor instruction",
    #if (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M33)
        [PRINT_UFSR_STKOF]        = "Usage fault is caused by indicates that a stack overflow (hardware check) has taken place",
    #endif
    [PRINT_UFSR_UNALIGNED]        = "Usage fault is caused by indicates that an unaligned access fault has taken place",
    [PRINT_UFSR_DIVBYZERO0]       = "Usage fault is caused by Indicates a divide by zero has taken place (can be set only if DIV_0_TRP is set)",
    [PRINT_DFSR_HALTED]           = "Debug fault is caused by halt requested in NVIC",
    [PRINT_DFSR_BKPT]             = "Debug fault is caused by BKPT instruction executed",
    [PRINT_DFSR_DWTTRAP]          = "Debug fault is caused by DWT match occurred",
    [PRINT_DFSR_VCATCH]           = "Debug fault is caused by Vector fetch occurred",
    [PRINT_DFSR_EXTERNAL]         = "Debug fault is caused by EDBGRQ signal asserted",
    [PRINT_MMAR]                  = "The memory management fault occurred address is %08x",
    [PRINT_BFAR]                  = "The bus fault occurred address is %08x",
};

static const char *fw_name, *hw_ver, *sw_ver;
static uint32_t main_stack_start_addr = 0;
static size_t main_stack_size = 0;
static uint32_t code_start_addr = 0;
static size_t code_size = 0;
static bool init_ok = false;
static char call_stack_info[CMB_CALL_STACK_MAX_DEPTH * (8 + 1)] = { 0 };
static bool on_fault = false;
static bool stack_is_overflow = false;
static struct cmb_hard_fault_regs regs;

#if (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M4) || \
    (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M7) || \
    (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M33)
static bool statck_has_fpu_regs = false;
#endif
static bool on_thread_before_fault = false;

#define cmb_get_msp() _ARMV7M_Get_MSP()
#define cmb_get_psp() _ARMV7M_Get_PSP()

static __always_inline uint32_t cmb_get_sp(void) {
    register uint32_t result;
    __asm volatile ("MOV %0, sp\n" : "=r" (result) );
    return(result);
}

/**
 * library initialize
 */
static void cm_backtrace_init(const char *firmware_name, const char *hardware_ver, 
    const char *software_ver) {
    fw_name = firmware_name;
    hw_ver = hardware_ver;
    sw_ver = software_ver; 
    main_stack_start_addr = (uint32_t)_ISR_Stack_area_begin;
    main_stack_size = (uint32_t)_ISR_Stack_area_end - main_stack_start_addr;
    code_start_addr = (uint32_t)bsp_section_text_begin;
    code_size = (uint32_t)bsp_section_text_size;
    if (main_stack_size == 0) {
        cmb_println(print_info[PRINT_MAIN_STACK_CFG_ERROR]);
        return;
    }

    init_ok = true;
}

/**
 * print firmware information, such as: firmware name, hardware version, software version
 */
void cm_backtrace_firmware_info(void) {
    cmb_println(print_info[PRINT_FIRMWARE_INFO], 
        fw_name, 
        hw_ver? hw_ver: " ", 
        sw_ver? sw_ver: " "
    );
}

/**
 * Get current thread stack information
 *
 * @param sp stack current pointer
 * @param start_addr stack start address
 * @param size stack size
 */
static void get_cur_thread_stack_info(uint32_t sp, uint32_t *start_addr, 
    size_t *size) {
    struct _Thread_Control *the_thread = _Thread_Get_executing();
    Stack_Control *stack = &the_thread->Start.Initial_stack;
    *start_addr = (uint32_t)stack->area;
    *size = stack->size;
    (void) sp;
}

/**
 * Get current thread name
 */
static const char *get_cur_thread_name(void) {
    static char buffer[5];
    struct _Thread_Control *the_thread = _Thread_Get_executing();
    const char *name;
    name = the_thread->Join_queue.Queue.name;
    if (name != NULL && name[0] != '\0')
        return name;
    _Objects_Name_to_string(the_thread->Object.name,
        false, buffer, sizeof(buffer));
    return buffer;
}

#ifdef CMB_USING_DUMP_STACK_INFO
/**
 * dump current stack information
 */
static void dump_stack(uint32_t stack_start_addr, size_t stack_size, 
    uint32_t *stack_pointer) {
    if (stack_is_overflow) {
        if (on_thread_before_fault) {
            cmb_println(print_info[PRINT_THREAD_STACK_OVERFLOW], stack_pointer);
        } else {
            cmb_println(print_info[PRINT_MAIN_STACK_OVERFLOW], stack_pointer);
        }
        if ((uint32_t) stack_pointer < stack_start_addr) {
            stack_pointer = (uint32_t *) stack_start_addr;
        } else if ((uint32_t) stack_pointer > stack_start_addr + stack_size) {
            stack_pointer = (uint32_t *) (stack_start_addr + stack_size);
        }
    }
    cmb_println(print_info[PRINT_THREAD_STACK_INFO]);
    for (; (uint32_t) stack_pointer < stack_start_addr + stack_size; stack_pointer++) {
        cmb_println("  addr: %08x    data: %08x", (uint32_t)stack_pointer, *stack_pointer);
    }
    cmb_println("====================================");
}
#endif /* CMB_USING_DUMP_STACK_INFO */

/* check the disassembly instruction is 'BL' or 'BLX' */
static bool disassembly_ins_is_bl_blx(uint32_t addr) {
    uint16_t ins1 = *((uint16_t *)addr);
    uint16_t ins2 = *((uint16_t *)(addr + 2));

#define BL_INS_MASK         0xF800
#define BL_INS_HIGH         0xF800
#define BL_INS_LOW          0xF000
#define BLX_INX_MASK        0xFF00
#define BLX_INX             0x4700

    if ((ins2 & BL_INS_MASK) == BL_INS_HIGH && (ins1 & BL_INS_MASK) == BL_INS_LOW) {
        return true;
    } else if ((ins2 & BLX_INX_MASK) == BLX_INX) {
        return true;
    } else {
        return false;
    }
}

/**
 * backtrace function call stack
 *
 * @param buffer call stack buffer
 * @param size buffer size
 * @param sp stack pointer
 *
 * @return depth
 */
size_t cm_backtrace_call_stack(uint32_t *buffer, size_t size, uint32_t sp) {
    uint32_t stack_start_addr = main_stack_start_addr, pc;
    size_t depth = 0, stack_size = main_stack_size;
    bool regs_saved_lr_is_valid = false;

    if (on_fault) {
        if (!stack_is_overflow) {
            /* first depth is PC */
            buffer[depth++] = regs.saved.pc;
            /* fix the LR address in thumb mode */
            pc = regs.saved.lr - 1;
            if ((pc >= code_start_addr) && 
                (pc <= code_start_addr + code_size) && 
                (depth < CMB_CALL_STACK_MAX_DEPTH) && 
                (depth < size)) {
                buffer[depth++] = pc;
                regs_saved_lr_is_valid = true;
            }
        }
        /* program is running on thread before fault */
        if (on_thread_before_fault)
            get_cur_thread_stack_info(sp, &stack_start_addr, &stack_size);
    } else {
        /* OS environment */
        if (cmb_get_sp() == cmb_get_psp())
            get_cur_thread_stack_info(sp, &stack_start_addr, &stack_size);
    }

    if (stack_is_overflow) {
        if (sp < stack_start_addr) {
            sp = stack_start_addr;
        } else if (sp > stack_start_addr + stack_size) {
            sp = stack_start_addr + stack_size;
        }
    }

    /* copy called function address */
    for (; sp < stack_start_addr + stack_size; sp += sizeof(size_t)) {
        /* the *sp value may be LR, so need decrease a word to PC */
        pc = *((uint32_t *) sp) - sizeof(size_t);
        /* the Cortex-M using thumb instruction, so the pc must be an odd number */
        if (pc % 2 == 0) {
            continue;
        }
        /* fix the PC address in thumb mode */
        pc = *((uint32_t *) sp) - 1;
        if ((pc >= code_start_addr + sizeof(size_t)) && 
            (pc <= code_start_addr + code_size) && 
                (depth < CMB_CALL_STACK_MAX_DEPTH)
                /* check the the instruction before PC address is 'BL' or 'BLX' */
                && disassembly_ins_is_bl_blx(pc - sizeof(size_t)) && 
                (depth < size)) {
            /* the second depth function may be already saved, so need ignore repeat */
            if ((depth == 2) && regs_saved_lr_is_valid && (pc == buffer[1])) {
                continue;
            }
            buffer[depth++] = pc;
        }
    }

    return depth;
}

/**
 * dump function call stack
 *
 * @param sp stack pointer
 */
static void print_call_stack(uint32_t sp) {
    size_t i, cur_depth = 0;
    uint32_t call_stack_buf[CMB_CALL_STACK_MAX_DEPTH] = {0};

    cur_depth = cm_backtrace_call_stack(call_stack_buf, CMB_CALL_STACK_MAX_DEPTH, sp);

    for (i = 0; i < cur_depth; i++) {
        sprintf(call_stack_info + i * (8 + 1), "%08lx", (unsigned long)call_stack_buf[i]);
        call_stack_info[i * (8 + 1) + 8] = ' ';
    }

    if (cur_depth) {
        call_stack_info[cur_depth * (8 + 1) - 1] = '\0';
        cmb_println(print_info[PRINT_CALL_STACK_INFO], fw_name, ".elf", call_stack_info);
    } else {
        cmb_println(print_info[PRINT_CALL_STACK_ERR]);
    }
}

/**
 * backtrace for assert
 *
 * @param sp the stack pointer when on assert occurred
 */
void cm_backtrace_assert(uint32_t sp) {
    CMB_ASSERT(init_ok);
    uint32_t cur_stack_pointer = cmb_get_sp();

    // cm_backtrace_firmware_info();

    /* OS environment */
    if (cur_stack_pointer == cmb_get_msp()) {
        cmb_println(print_info[PRINT_ASSERT_ON_HANDLER]);

#ifdef CMB_USING_DUMP_STACK_INFO
        dump_stack(main_stack_start_addr, main_stack_size, (uint32_t *) sp);
#endif /* CMB_USING_DUMP_STACK_INFO */

    } else if (cur_stack_pointer == cmb_get_psp()) {
        cmb_println(print_info[PRINT_ASSERT_ON_THREAD], get_cur_thread_name());

#ifdef CMB_USING_DUMP_STACK_INFO
        uint32_t stack_start_addr;
        size_t stack_size;
        get_cur_thread_stack_info(sp, &stack_start_addr, &stack_size);
        dump_stack(stack_start_addr, stack_size, (uint32_t *) sp);
#endif /* CMB_USING_DUMP_STACK_INFO */

    }
    print_call_stack(sp);
}

#if (CMB_CPU_PLATFORM_TYPE != CMB_CPU_ARM_CORTEX_M0)
/**
 * fault diagnosis then print cause of fault
 */
static void fault_diagnosis(void) {
    if (regs.hfsr.bits.VECTBL) {
        cmb_println(print_info[PRINT_HFSR_VECTBL]);
    }
    if (regs.hfsr.bits.FORCED) {
        /* Memory Management Fault */
        if (regs.mfsr.value) {
            if (regs.mfsr.bits.IACCVIOL) {
                cmb_println(print_info[PRINT_MFSR_IACCVIOL]);
            }
            if (regs.mfsr.bits.DACCVIOL) {
                cmb_println(print_info[PRINT_MFSR_DACCVIOL]);
            }
            if (regs.mfsr.bits.MUNSTKERR) {
                cmb_println(print_info[PRINT_MFSR_MUNSTKERR]);
            }
            if (regs.mfsr.bits.MSTKERR) {
                cmb_println(print_info[PRINT_MFSR_MSTKERR]);
            }

#if (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M4) || \
    (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M7) || \
    (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M33)
            if (regs.mfsr.bits.MLSPERR) {
                cmb_println(print_info[PRINT_MFSR_MLSPERR]);
            }
#endif

            if (regs.mfsr.bits.MMARVALID) {
                if (regs.mfsr.bits.IACCVIOL || regs.mfsr.bits.DACCVIOL) {
                    cmb_println(print_info[PRINT_MMAR], regs.mmar);
                }
            }
        }
        /* Bus Fault */
        if (regs.bfsr.value) {
            if (regs.bfsr.bits.IBUSERR) {
                cmb_println(print_info[PRINT_BFSR_IBUSERR]);
            }
            if (regs.bfsr.bits.PRECISERR) {
                cmb_println(print_info[PRINT_BFSR_PRECISERR]);
            }
            if (regs.bfsr.bits.IMPREISERR) {
                cmb_println(print_info[PRINT_BFSR_IMPREISERR]);
            }
            if (regs.bfsr.bits.UNSTKERR) {
                cmb_println(print_info[PRINT_BFSR_UNSTKERR]);
            }
            if (regs.bfsr.bits.STKERR) {
                cmb_println(print_info[PRINT_BFSR_STKERR]);
            }

#if (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M4) || \
    (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M7) || \
    (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M33)
            if (regs.bfsr.bits.LSPERR) {
                cmb_println(print_info[PRINT_BFSR_LSPERR]);
            }
#endif

            if (regs.bfsr.bits.BFARVALID) {
                if (regs.bfsr.bits.PRECISERR) {
                    cmb_println(print_info[PRINT_BFAR], regs.bfar);
                }
            }

        }
        /* Usage Fault */
        if (regs.ufsr.value) {
            if (regs.ufsr.bits.UNDEFINSTR) {
                cmb_println(print_info[PRINT_UFSR_UNDEFINSTR]);
            }
            if (regs.ufsr.bits.INVSTATE) {
                cmb_println(print_info[PRINT_UFSR_INVSTATE]);
            }
            if (regs.ufsr.bits.INVPC) {
                cmb_println(print_info[PRINT_UFSR_INVPC]);
            }
            if (regs.ufsr.bits.NOCP) {
                cmb_println(print_info[PRINT_UFSR_NOCP]);
            }
#if (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M33)
            if (regs.ufsr.bits.STKOF) {
                cmb_println(print_info[PRINT_UFSR_STKOF]);
            }
#endif
            if (regs.ufsr.bits.UNALIGNED) {
                cmb_println(print_info[PRINT_UFSR_UNALIGNED]);
            }
            if (regs.ufsr.bits.DIVBYZERO0) {
                cmb_println(print_info[PRINT_UFSR_DIVBYZERO0]);
            }
        }
    }
    /* Debug Fault */
    if (regs.hfsr.bits.DEBUGEVT) {
        if (regs.dfsr.value) {
            if (regs.dfsr.bits.HALTED) {
                cmb_println(print_info[PRINT_DFSR_HALTED]);
            }
            if (regs.dfsr.bits.BKPT) {
                cmb_println(print_info[PRINT_DFSR_BKPT]);
            }
            if (regs.dfsr.bits.DWTTRAP) {
                cmb_println(print_info[PRINT_DFSR_DWTTRAP]);
            }
            if (regs.dfsr.bits.VCATCH) {
                cmb_println(print_info[PRINT_DFSR_VCATCH]);
            }
            if (regs.dfsr.bits.EXTERNAL) {
                cmb_println(print_info[PRINT_DFSR_EXTERNAL]);
            }
        }
    }
}
#endif /* (CMB_CPU_PLATFORM_TYPE != CMB_CPU_ARM_CORTEX_M0) */

#if (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M4) || \
    (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M7) || \
    (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M33)
static uint32_t statck_del_fpu_regs(uint32_t fault_handler_lr, uint32_t sp) {
    statck_has_fpu_regs = (fault_handler_lr & (1UL << 4)) == 0 ? true : false;

    /* the stack has S0~S15 and FPSCR registers when statck_has_fpu_regs is true, double word align */
    return statck_has_fpu_regs == true ? sp + sizeof(size_t) * 18 : sp;
}
#endif

/**
 * backtrace for fault
 * @note only call once
 *
 * @param fault_handler_lr the LR register value on fault handler
 * @param fault_handler_sp the stack pointer on fault handler
 */
void cm_backtrace_fault(uint32_t fault_handler_lr, uint32_t fault_handler_sp) {
    uint32_t stack_pointer = fault_handler_sp, saved_regs_addr = stack_pointer;
    const char *regs_name[] = { 
        "R0 ", "R1 ", "R2 ", "R3 ", 
        "R12", "LR ", "PC ", "PSR" 
    };

#ifdef CMB_USING_DUMP_STACK_INFO
    uint32_t stack_start_addr = main_stack_start_addr;
    size_t stack_size = main_stack_size;
#endif

    CMB_ASSERT(init_ok);
    /* only call once */
    CMB_ASSERT(!on_fault);

    on_fault = true;

    cmb_println("\n");
    cm_backtrace_firmware_info();

    on_thread_before_fault = fault_handler_lr & (1UL << 2);
    /* check which stack was used before (MSP or PSP) */
    if (on_thread_before_fault) {
        cmb_println(print_info[PRINT_FAULT_ON_THREAD], get_cur_thread_name());
        saved_regs_addr = stack_pointer = cmb_get_psp();

#ifdef CMB_USING_DUMP_STACK_INFO
        get_cur_thread_stack_info(stack_pointer, &stack_start_addr, &stack_size);
#endif /* CMB_USING_DUMP_STACK_INFO */

    } else {
        cmb_println(print_info[PRINT_FAULT_ON_HANDLER]);
    }

    /* delete saved R0~R3, R12, LR,PC,xPSR registers space */
    stack_pointer += sizeof(size_t) * 8;

#if (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M4) || \
    (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M7) || \
    (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M33)
    stack_pointer = statck_del_fpu_regs(fault_handler_lr, stack_pointer);
#endif /* (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M4) || (CMB_CPU_PLATFORM_TYPE == CMB_CPU_ARM_CORTEX_M7) */

#ifdef CMB_USING_DUMP_STACK_INFO
    /* check stack overflow */
    if (stack_pointer < stack_start_addr || stack_pointer > stack_start_addr + stack_size) {
        stack_is_overflow = true;
    }
    /* dump stack information */
    dump_stack(stack_start_addr, stack_size, (uint32_t *) stack_pointer);
#endif /* CMB_USING_DUMP_STACK_INFO */

    /* the stack frame may be get failed when it is overflow  */
    if (!stack_is_overflow) {
        /* dump register */
        cmb_println(print_info[PRINT_REGS_TITLE]);

        regs.saved.r0        = ((uint32_t *)saved_regs_addr)[0];  // Register R0
        regs.saved.r1        = ((uint32_t *)saved_regs_addr)[1];  // Register R1
        regs.saved.r2        = ((uint32_t *)saved_regs_addr)[2];  // Register R2
        regs.saved.r3        = ((uint32_t *)saved_regs_addr)[3];  // Register R3
        regs.saved.r12       = ((uint32_t *)saved_regs_addr)[4];  // Register R12
        regs.saved.lr        = ((uint32_t *)saved_regs_addr)[5];  // Link register LR
        regs.saved.pc        = ((uint32_t *)saved_regs_addr)[6];  // Program counter PC
        regs.saved.psr.value = ((uint32_t *)saved_regs_addr)[7];  // Program status word PSR

        cmb_println("  %s: %08x  %s: %08x  %s: %08x  %s: %08x", 
            regs_name[0], regs.saved.r0,
            regs_name[1], regs.saved.r1,
            regs_name[2], regs.saved.r2,
            regs_name[3], regs.saved.r3);
        cmb_println("  %s: %08x  %s: %08x  %s: %08x  %s: %08x", 
            regs_name[4], regs.saved.r12,
            regs_name[5], regs.saved.lr,
            regs_name[6], regs.saved.pc,
            regs_name[7], regs.saved.psr.value);
        cmb_println("==============================================================");
    }

    /* the Cortex-M0 is not support fault diagnosis */
#if (CMB_CPU_PLATFORM_TYPE != CMB_CPU_ARM_CORTEX_M0)
    regs.syshndctrl.value = CMB_SYSHND_CTRL;  // System Handler Control and State Register
    regs.mfsr.value       = CMB_NVIC_MFSR;    // Memory Fault Status Register
    regs.mmar             = CMB_NVIC_MMAR;    // Memory Management Fault Address Register
    regs.bfsr.value       = CMB_NVIC_BFSR;    // Bus Fault Status Register
    regs.bfar             = CMB_NVIC_BFAR;    // Bus Fault Manage Address Register
    regs.ufsr.value       = CMB_NVIC_UFSR;    // Usage Fault Status Register
    regs.hfsr.value       = CMB_NVIC_HFSR;    // Hard Fault Status Register
    regs.dfsr.value       = CMB_NVIC_DFSR;    // Debug Fault Status Register
    regs.afsr             = CMB_NVIC_AFSR;    // Auxiliary Fault Status Register

    fault_diagnosis();
#endif

    print_call_stack(stack_pointer);
}

static void backtrace_init(void) {
    cm_backtrace_init("rtems", NULL, NULL);
}
RTEMS_SYSINIT_ITEM(backtrace_init,
    RTEMS_SYSINIT_BSP_EARLY,
    RTEMS_SYSINIT_ORDER_MIDDLE
);
