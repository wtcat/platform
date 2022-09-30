/*
 * CopyRight(c) 2022 wtcat
 */
#include <rtems.h>
#include <rtems/thread.h>
#include <rtems/shell.h>
#include <rtems/bspIo.h>
#include <rtems/media.h>
#include <rtems/untar.h>

#if defined (__rtems_libbsd__)
#include <rtems/bsd/bsd.h>
#include <machine/rtems-bsd-config.h>
#ifdef CONFIGURE_GDBSERVER
#include <rtems/rtems-debugger-remote-tcp.h>
#endif
#endif

#include <bsp/stackalloc.h>
#include "bsp/board/sysconf.h"
#include "bsp/init.h"

#include "shell/shell_utils.h"


#define TIMEOUT_MS(ms) RTEMS_MILLISECONDS_TO_TICKS(ms)
#define kerror(fmt, ...) rtems_panic("<*Kernel Panic*>: "fmt, ##__VA_ARGS__) 

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

static void make_rootfs(void) {
extern const unsigned char __rootfs_image[];
extern const size_t __rootfs_image_size;
  int err = Untar_FromMemory((char*)__rootfs_image, __rootfs_image_size);
  if (err)
    kerror("make rootfs failed(%d)\n", err);
}

int RTEMS_WEAK rtems_main(void) {
  return 0;
}

static rtems_task Init(rtems_task_argument arg) {
  (void) arg;
  make_rootfs();
  if (shell_run_script("/etc/start.rs"))
    kerror("Run /etc/start.rs failed");

#ifdef CONFIGURE_MEDIA_SERVICE
  err = media_service_setup();
  if (err)
    SYS_ERROR("Media service start failed: %d\n", err);
#endif

#ifdef __rtems_libbsd__
  if (rtems_bsd_initialize())
    rtems_panic("LIBBSD initialize failed\n");
  rtems_bsd_run_etc_rc_conf(TIMEOUT_MS(5000), true);
#ifdef CONFIGURE_GDBSERVER
  rtems_debugger_register_tcp_remote();
#endif
#endif
  shell_run_script("/etc/post.rs");
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
