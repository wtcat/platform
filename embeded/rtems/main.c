/*
 * CopyRight(c) 2022 wtcat
 */
#include <rtems.h>
#include <rtems/untar.h>
#if defined (__rtems_libbsd__)
#include <rtems/bsd/bsd.h>
#ifdef CONFIGURE_GDBSERVER
#include <rtems/rtems-debugger-remote-tcp.h>
#endif
#endif

#include "bsp/board/sysconf.h"
#include "shell/shell_utils.h"


#define MS(ms) RTEMS_MILLISECONDS_TO_TICKS(ms)
#define kerr(fmt, ...) rtems_panic("<*Kernel Panic*>: "fmt, ##__VA_ARGS__) 

int RTEMS_WEAK rtems_main(void) {
  return 0;
}

static void make_rootfs(void) {
  extern const unsigned char __rootfs_image[];
  extern const size_t __rootfs_image_size;
  int err = Untar_FromMemory((char*)__rootfs_image, __rootfs_image_size);
  if (err)
    kerr("make rootfs failed(%d)\n", err);
}

static rtems_task Init(rtems_task_argument arg) {
  (void) arg;
  /* */
  make_rootfs();

  /* system startup script */
  if (shell_run_script("/etc/start.rs"))
    kerr("run /etc/start.rs failed\n");

#if defined(__rtems_libbsd__)
  /* Setup run environment for freebsd */
  if (rtems_bsd_initialize())
    kerror("libbsd startup failed\n");

  /* Run configure script */
  rtems_bsd_run_etc_rc_conf(MS(5000), true);

#ifdef CONFIGURE_GDBSERVER
  rtems_debugger_register_tcp_remote();
#endif
#endif /* __rtems_libbsd__ */
  shell_run_script("/etc/post.rs");
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
