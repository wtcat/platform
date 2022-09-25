/*
 * Copyright wtcat 2022
 */
#include <stdio.h>
#include <rtems.h>
#include <rtems/counter.h>

#include "base/compiler.h"
#include "base/timer_ii.h"
#include "lvgl.h"

enum ui_view_message {
    kUIViewDisplay,
    kUIViewLayout,
    kUIViewFocus,
    kUIViewDefocus,
    kUIViewPreload,
    kUIViewCreate,
    kUIViewDelete,
    kUIViewPaint,
    kUIViewRefresh,
};

struct thread_arg {
    rtems_id handle;
    void (*thread_entry)(void *arg);
};

struct ui_thread_arg {
    struct thread_arg base;
    rtems_id message;
};

struct dp_thread_arg {
#define UI_DISPLAY_EVENT RTEMS_EVENT_20
    struct thread_arg base;
    struct timer_ii timer;
    rtems_counter_ticks last_timestamp;
    bool busy;
};

struct ui_message {
    uint16_t type;
};


#define UI_THREAD_STACK_SIZE 4096
#define DP_THREAD_STACK_SIZE 2048
#define UI_THREAD_PRIORITY 10
#define DP_THREAD_PRIORITY 9

static struct ui_thread_arg ui_thread;
static struct dp_thread_arg dp_thread;
static char ui_thread_stack[UI_THREAD_STACK_SIZE] __aligned(8);
static char dp_thread_stack[DP_THREAD_STACK_SIZE] __aligned(8);

static int ui_thread_create(struct thread_arg *thread, void *stack, 
    size_t size, int piro);

static void ui_sevice_thread(void *arg) {
    struct ui_thread_arg *ui = (struct ui_thread_arg *)arg;
    struct ui_message msg;
    rtems_status_code sc;
    size_t size;

    sc = rtems_message_queue_create(rtems_build_name('U','I','M','G'),
        32, sizeof(struct ui_message), RTEMS_LOCAL, &ui->message);
    if (sc != RTEMS_SUCCESSFUL) {
        printf("Error***(%s): Create message queue failed(%s)\n", __func__, 
            rtems_status_text(sc));
        return;
    }
    int ret = ui_thread_create(&dp_thread.base, dp_thread_stack, 
        DP_THREAD_STACK_SIZE, DP_THREAD_PRIORITY);
    if (ret) {
        rtems_message_queue_delete(ui->message);
        return;
    }
    for ( ; ; ) {
        rtems_message_queue_receive(ui->message, &msg, &size, 
            RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        switch (msg.type) {
        case kUIViewDisplay:
            lv_timer_handler();
            break;
        default:
            break;
        }
    }
}

static void ui_display_thread(void *arg) {
    (void) arg;
    rtems_event_set ev;

    for ( ; ; ) {
        rtems_event_receive(UI_DISPLAY_EVENT, RTEMS_WAIT|RTEMS_EVENT_ALL, 
            RTEMS_NO_TIMEOUT, &ev);
        (void) ev;
        
        /* Input process */

        /* Animation process */

        /* Display process */
    }
}

static void ui_thread_entry(rtems_task_argument arg) {
    struct thread_arg *th = (struct thread_arg *)arg;
    th->thread_entry(th);
    rtems_task_exit();
}

static int ui_thread_create(struct thread_arg *thread, void *stack, 
    size_t size, int piro) {
    rtems_task_config config;
    rtems_status_code sc;

    config.name = rtems_build_name('L','V','G','L');
    config.storage_area = stack;
    config.storage_size = size;
    config.storage_free = NULL;
    config.attributes = RTEMS_LOCAL | RTEMS_NO_FLOATING_POINT;
    config.initial_modes = RTEMS_PREEMPT | RTEMS_NO_TIMESLICE | RTEMS_NO_ASR;
    config.initial_priority = piro;
    config.maximum_thread_local_storage_size = 0;
    sc = rtems_task_construct(&config, &thread->handle);
    if (sc != RTEMS_SUCCESSFUL) {
        printf("Error***(%s): create thread failed(%s)\n", __func__,
            rtems_status_text(sc));
    }
    sc = rtems_task_start(thread->handle, ui_thread_entry, 
        (rtems_task_argument)thread);
    return -rtems_status_code_to_errno(sc);
}

static int ui_thread_restart(struct thread_arg *thread) {
    rtems_status_code sc;
    sc = rtems_task_restart(thread->handle, (rtems_task_argument)thread);
    return -rtems_status_code_to_errno(sc);    
}

static void vsync_delayed_timer(struct timer_ii *timer) {
    struct dp_thread_arg *dp = RTEMS_CONTAINER_OF(timer, struct dp_thread_arg, timer);
    rtems_event_send(dp->base.handle, UI_DISPLAY_EVENT);
}

void ui_display_vsync(rtems_counter_ticks timestamp) {
    struct dp_thread_arg *dp = &dp_thread;
    rtems_counter_ticks delta;
    delta = rtems_counter_difference(timestamp, dp->last_timestamp);
    dp->last_timestamp = timestamp;
    delta = delta * 1000 / rtems_counter_frequency();
    if (delta > )
    timer_ii_mod(&dp->timer, delta);
}

int ui_init(void) {
	lv_init();
    return 0;
}


// static char lvgl_heap_buffer
// void *_lvgl_malloc(size_t size) {

// }

// void *_lvgl_realloc(void *ptr, size_t size) {

// }

// void _lvgl_free(void *ptr) {

// }


/* HAL settings */
#define LV_TICK_CUSTOM			     1
#define LV_TICK_CUSTOM_INCLUDE		 <rtems/rtems/clock.h>
#define LV_TICK_CUSTOM_SYS_TIME_EXPR ({ \
    uint32_t _now = rtems_clock_get_ticks_since_boot(); \
    uint32_t _ret = _now * rtems_configuration_get_milliseconds_per_tick(); \
    _ret;})