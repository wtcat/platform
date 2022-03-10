#include <unistd.h>

#include <rtems.h>
#include <rtems/shell.h>
#include <rtems/bspIo.h>
#include "bsp/sysconf.h"

#if defined (__rtems_libbsd__)
#include <rtems/bsd/bsd.h>
#include <machine/rtems-bsd-config.h>
#endif

extern int shell_init(rtems_shell_login_check_t login_check);

static int sysfile_add(const char *pathname, int mode,
    const char *content) {
    if (access(pathname, F_OK) < 0) {
        return IMFS_make_linearfile(pathname, mode, 
            content, strlen(content));
    }
    return 0;
}

static void etc_init(void) {
#ifdef CONFIG_JOEL_SCRIPT_CONTENT
    if (!sysfile_add("/etc/start.joel", 0777, 
      CONFIG_JOEL_SCRIPT_CONTENT))
      rtems_shell_script_file(0, "/etc/start.joel");
#endif
#ifdef CONFIGURE_ETC_RC_CONF_CONTENT
    sysfile_add("/etc/rc.conf", 0666, 
      CONFIGURE_ETC_RC_CONF_CONTENT);
#endif
#ifdef CONFIG_DYNMAIC_LIB_CONTENT
    sysfile_add("/etc/libdl.conf", 0666, 
      CONFIG_DYNMAIC_LIB_CONTENT);
#endif
}

static rtems_task Init(rtems_task_argument ignored) {
  printk( "Hello World\n" );
  shell_init(NULL);
  etc_init();
#if defined(__rtems_libbsd__)
  if (rtems_bsd_initialize())
    rtems_panic("LIBBSD initialize failed\n");
  /* Execute /etc/rc.conf script */
  rtems_bsd_run_etc_rc_conf(RTEMS_MILLISECONDS_TO_TICKS(10000), true);
#else /* !__rtems_libbsd__ */

#endif /* __rtems_libbsd__ */
}

#if defined (__rtems_libbsd__) && \
    defined(CONFIGURE_FDT)
RTEMS_BSD_DEFINE_NEXUS_DEVICE(ofwbus, 0, 0, NULL);
SYSINIT_DRIVER_REFERENCE(simplebus, ofwbus);
#endif

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
