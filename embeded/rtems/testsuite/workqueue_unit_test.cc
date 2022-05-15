#include <stdio.h>
#include <rtems/sysinit.h>

#include "component/workqueue.h"
#include "gtest/gtest.h"

#include "bsp/asm/stacktrace.h"

#define K_MSEC(n)      WQ_MSEC(n)
#define SLEEP_MS(msec) rtems_task_wake_after(WQ_MSEC(msec))

extern "C"{

static volatile int test_phase;

static void __attribute__((noinline)) func_a(void) {
    rtems_printer printer;
    rtems_print_printer_printf(&printer);
    stack_backtrace(&printer);
}

static void __attribute__((noinline)) func_b(void) {
    printf(" ");
    func_a();
    printf(" ");
}

static void __attribute__((noinline)) func_c(void) {
    printf(" ");
    func_b();
    printf(" ");
}

static void test_task_1(struct work_struct *work) {
    test_phase = 3;
    printf("<%s> start\n", __func__);
    SLEEP_MS(2000);
    printf("<%s> end\n", __func__);
    test_phase = 5;
}

static void test_task_2(struct work_struct *work) {
    printf("\n**************<%s> start\n", __func__);
    func_c();
    printf("****************<%s> end\n\n", __func__);
}

static struct work_struct test_work_1 = WORK_INITIALIZER(test_task_1);
static struct work_struct test_work_2 = WORK_INITIALIZER(test_task_2);
}

TEST(workqueue, schedule) {
    ASSERT_TRUE(test_phase == 0);
    schedule_work(&test_work_1);
    SLEEP_MS(10);
    cancel_work_sync(&test_work_1);
    SLEEP_MS(3000);
    ASSERT_TRUE(test_phase == 5);
    
}

TEST(unwind, backtrace) {
    schedule_work(&test_work_2);
}
