/*
 * CopyRight(c) 2022 wtcat
 */
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <rtems.h>
#include <rtems/thread.h>
#include <rtems/imfs.h>
#include <rtems/shell.h>
#include <rtems/bspIo.h>
#include <rtems/media.h>
#include <rtems/untar.h>
#ifdef CONFIGURE_GDBSERVER
#include <rtems/rtems-debugger-remote-tcp.h>
#endif

#include <bsp/stackalloc.h>
#include "bsp/board/sysconf.h"
#include "bsp/init.h"

#if defined (__rtems_libbsd__)
#include <rtems/bsd/bsd.h>
#include <machine/rtems-bsd-config.h>
#endif

#define TIMEOUT_MS(ms) RTEMS_MILLISECONDS_TO_TICKS(ms)
#define SYS_ERROR(fmt, ...) \
  rtems_panic("System Error***: "fmt"(%s:%d)", ##__VA_ARGS__, \
    __func__, __LINE__)

#ifdef CONFIGURE_MEDIA_SERVICE

#if defined (__rtems_libbsd__)
static rtems_binary_semaphore bsd_completed_sem = 
    RTEMS_BINARY_SEMAPHORE_INITIALIZER("mount");
#endif

static rtems_status_code 
media_listener(rtems_media_event event, rtems_media_state state, 
    const char *src, const char *dest, void *arg) {
    (void) src;
    (void) arg;
    if (event == RTEMS_MEDIA_EVENT_MOUNT) {
      if (state == RTEMS_MEDIA_STATE_SUCCESS) 
        symlink(dest, "/home");
    #if defined (__rtems_libbsd__)
      rtems_binary_semaphore_post(&bsd_completed_sem);
    #endif
    }
    return RTEMS_SUCCESSFUL;
}

static int media_service_setup(void) {
  rtems_status_code sc;
  sc = rtems_media_initialize();
  if (sc == RTEMS_SUCCESSFUL) {
    sc = rtems_media_server_initialize(110, 4096,
      RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES);
    if (sc == RTEMS_SUCCESSFUL) 
      rtems_media_listener_add(media_listener, NULL);
  }
  return rtems_status_code_to_errno(sc);
}
#endif

static void libbsd_setup(void) {
#if defined(__rtems_libbsd__)
  rtems_task_priority prio;
  rtems_task_set_priority(RTEMS_SELF, 110, &prio);
  (void) prio;
  if (rtems_bsd_initialize())
    rtems_panic("LIBBSD initialize failed\n");
#ifdef CONFIGURE_MEDIA_SERVICE
  rtems_binary_semaphore_wait(&bsd_completed_sem);
#endif
  rtems_bsd_run_etc_rc_conf(TIMEOUT_MS(5000), true);
#endif/* __rtems_libbsd__ */
}

static int make_rootfs(void) {
extern const unsigned char __rootfs_image[];
extern const size_t __rootfs_image_size;
  return Untar_FromMemory((char*)__rootfs_image, __rootfs_image_size);
}

int RTEMS_WEAK rtems_main(void) {
  return 0;
}

static rtems_task Init(rtems_task_argument arg) {
  (void) arg;
  int err = make_rootfs();
  if (err) {
    SYS_ERROR("Make rootfs error(%d)\n", err);
    exit(-1);
  }

  err = ramblk_init();
  if (err) 
    SYS_ERROR("Create ramdisk failed: %d\n", err);
  err = shell_init(NULL);
  if (err) 
    SYS_ERROR("Shell initialize failed: %d\n", err);
#ifdef CONFIGURE_MEDIA_SERVICE
  err = media_service_setup();
  if (err)
    SYS_ERROR("Media service start failed: %d\n", err);
#endif
  // environment_load();
  libbsd_setup();
#ifdef CONFIGURE_GDBSERVER
  rtems_debugger_register_tcp_remote();
#endif
  rtems_main();
  rtems_task_exit();
}

#if defined (__rtems_libbsd__) && \
    defined(CONFIGURE_FDT)
RTEMS_BSD_DEFINE_NEXUS_DEVICE(ofwbus, 0, 0, NULL);
SYSINIT_DRIVER_REFERENCE(simplebus, ofwbus);
#endif

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
