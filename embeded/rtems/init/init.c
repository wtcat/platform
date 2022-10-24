/*
 * CopyRight(c) 2022 wtcat
 */
#include "configs/rtems_confdefs.h"

#include <rtems.h>
#include <rtems/untar.h>
#ifdef __rtems_libbsd__
#include <rtems/bsd/bsd.h>
#include <machine/rtems-bsd-rc-conf.h>
#ifdef CONFIGURE_GDBSERVER
#include <rtems/rtems-debugger-remote-tcp.h>
#endif
#endif /* __rtems_libbsd__ */
#include "shell/shell_utils.h"

#define MS(ms) RTEMS_MILLISECONDS_TO_TICKS(ms)

void RTEMS_WEAK _shell_init(void) {
}

int RTEMS_WEAK rtems_main(void) {
  return 0;
}

static void rootfs_init(void) {
  extern char __rootfs_image[];
  extern char __rootfs_size[];
  int err = Untar_FromMemory(__rootfs_image, (size_t)__rootfs_size);
  if (err)
    rtems_panic("make rootfs failed(%d)\n", err);
}

static rtems_task Init(rtems_task_argument arg) {
  (void) arg;
  rootfs_init();

  /* Run startup script */
  if (shell_run_script("/etc/start.rs"))
    rtems_panic("run /etc/start.rs failed\n");

#if defined(__rtems_libbsd__)
  /* Setup run environment for freebsd */
  if (rtems_bsd_initialize())
    rtems_panic("libbsd startup failed\n");

  /* Run configure script */
  rtems_bsd_run_etc_rc_conf(MS(5000), true);

#ifdef CONFIGURE_GDBSERVER
  rtems_debugger_register_tcp_remote();
#endif
#endif /* __rtems_libbsd__ */
  _shell_init();
  rtems_main();
  rtems_task_exit();
}

#if defined (__rtems_libbsd__) && \
    defined(CONFIGURE_FDT)
#include <machine/rtems-bsd-config.h>
RTEMS_BSD_DEFINE_NEXUS_DEVICE(ofwbus, 0, 0, NULL);
SYSINIT_DRIVER_REFERENCE(simplebus, ofwbus);
#endif

#define CONFIGURE_INIT
#include <bsp/stackalloc.h>
#include <rtems/confdefs.h>
