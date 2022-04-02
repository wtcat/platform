#include <stdio.h>
#include <rtems/sysinit.h>

#include "component/workqueue.h"


#define K_MSEC(n)      WQ_MSEC(n)
#define SLEEP_MS(msec) rtems_task_wake_after(WQ_MSEC(msec))

static volatile int test_phase;

static void test_task_1(struct work_struct *work) {
    test_phase = 3;
    printf("<%s> start\n", __func__);
    SLEEP_MS(2000);
    printf("<%s> end\n", __func__);
    test_phase = 5;
}

static void test_task_2(struct work_struct *work) {
    printf("<%s> start\n", __func__);
    

    printf("<%s> end\n", __func__);
}

static struct work_struct test_work_1 = WORK_INITIALIZER(test_task_1);
static struct work_struct test_work_2 = WORK_INITIALIZER(test_task_2);

int main(void) {
    rtems_task_priority prio;
    rtems_status_code sc;
    sc = rtems_task_set_priority(rtems_task_self(), 100, &prio);
    _Assert(sc == RTEMS_SUCCESSFUL);
    printf("******** Workqueue test start **********\n");
    schedule_work(&test_work_1);
    SLEEP_MS(10);
    cancel_work_sync(&test_work_1);
    SLEEP_MS(3000);
    _Assert(test_phase == 5);
    printf("******** Workqueue test end **********\n");
    return 0;
}
