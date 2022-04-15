/*
 * CopyRight(c) 2022 wtcat
 */
#include <string.h>
#include <unistd.h>

#include <rtems.h>
#include <rtems/imfs.h>
#include <rtems/shell.h>
#include <rtems/bspIo.h>
#include <rtems/media.h>
#ifdef CONFIGURE_DEBUGGER
#include <rtems/rtems-debugger-remote-tcp.h>
#endif

#include <bsp/stackalloc.h>
#include "bsp/sysconf.h"
#include "bsp/init.h"

#if defined (__rtems_libbsd__)
#include <rtems/bsd/bsd.h>
#include <machine/rtems-bsd-config.h>
#endif

#define SYS_ERROR(fmt, ...) \
  rtems_panic("System Error***: "fmt"(%s:%d)", ##__VA_ARGS__, \
    __func__, __LINE__)

#ifdef CONFIGURE_MEDIA_SERVICE
static rtems_status_code 
media_listener(rtems_media_event event, rtems_media_state state, 
    const char *src, const char *dest, void *arg)
{
    if (event == RTEMS_MEDIA_EVENT_MOUNT &&
        state == RTEMS_MEDIA_STATE_SUCCESS) {
        symlink(dest, "/home");
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

static int sysfile_create(const char *pathname, int mode,
    const char *content) {
    if (access(pathname, F_OK) < 0) {
        return IMFS_make_linearfile(pathname, mode, 
            content, strlen(content));
    }
    return 0;
}

static void environment_load(void) {
#ifdef CONFIG_JOEL_SCRIPT_CONTENT
    if (!sysfile_create("/etc/start.joel", 0777, 
      CONFIG_JOEL_SCRIPT_CONTENT)) {
        char *script[] = {"/etc/start.joel"};
      rtems_shell_script_file(0, script);
    }
#endif
#ifdef CONFIGURE_ETC_RC_CONF_CONTENT
    sysfile_create("/etc/rc.conf", 0666, 
      CONFIGURE_ETC_RC_CONF_CONTENT);
#endif
#ifdef CONFIG_DYNMAIC_LIB_CONTENT
    sysfile_create("/etc/libdl.conf", 0666, 
      CONFIG_DYNMAIC_LIB_CONTENT);
#endif
}

static void libbsd_setup(void) {
#if defined(__rtems_libbsd__)
  rtems_task_priority prio;
  rtems_task_set_priority(RTEMS_SELF, 110, &prio);
  (void) prio;
  if (rtems_bsd_initialize())
    rtems_panic("LIBBSD initialize failed\n");
  rtems_task_wake_after(RTEMS_MILLISECONDS_TO_TICKS(1000));
  rtems_bsd_run_etc_rc_conf(RTEMS_MILLISECONDS_TO_TICKS(5000), true);
#endif/* __rtems_libbsd__ */
}

int RTEMS_WEAK app_main(void) {
  return 0;
}

static rtems_task Init(rtems_task_argument arg) {
  int err = ramblk_init();
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
  environment_load();
  libbsd_setup();
#ifdef CONFIGURE_DEBUGGER
  rtems_debugger_register_tcp_remote();
#endif
  app_main();
  rtems_task_exit();
}

#if defined (__rtems_libbsd__) && \
    defined(CONFIGURE_FDT)
RTEMS_BSD_DEFINE_NEXUS_DEVICE(ofwbus, 0, 0, NULL);
SYSINIT_DRIVER_REFERENCE(simplebus, ofwbus);
#endif

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
