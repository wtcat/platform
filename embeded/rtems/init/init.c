/*
 * CopyRight(c) 2022 wtcat
 */
#include "configs/rtems_confdefs.h"

#include <rtems.h>
#include <rtems/untar.h>
#include <rtems/sysinit.h>
#include <rtems/media.h>
#ifdef __rtems_libbsd__
#include <rtems/bsd/bsd.h>
#include <machine/rtems-bsd-rc-conf.h>
#ifdef CONFIGURE_GDBSERVER
#include <rtems/rtems-debugger-remote-tcp.h>
#endif
#endif /* __rtems_libbsd__ */
#include "shell/shell_utils.h"
#include "base/init.h"

#define CONFIGURE_INIT
#include <bsp/stackalloc.h>
#include <rtems/confdefs.h>


#ifndef CONFIGURE_IRQ_SERVER_PRIO
#define CONFIGURE_IRQ_SERVER_PRIO 1
#endif
#ifndef CONFIGURE_TMS_SERVER_STKSZ
#define CONFIGURE_TMS_SERVER_STKSZ 4096
#endif
#ifndef CONFIGURE_TMS_SERVER_PRIO
#define CONFIGURE_TMS_SERVER_PRIO 3
#endif
#ifndef CONFIGURE_IRQ_SERVER_STKSZ
#define CONFIGURE_IRQ_SERVER_STKSZ 4096
#endif
#ifndef CONFIGURE_MEDIA_SERVER_STKSZ
#define CONFIGURE_MEDIA_SERVER_STKSZ 2048
#endif

#define MS(ms) RTEMS_MILLISECONDS_TO_TICKS(ms)

RTEMS_LINKER_ROSET(_Init, rtems_sysinit_item);

void RTEMS_WEAK _shell_init(void) {
}

int RTEMS_WEAK rtems_main(void) {
  return 0;
}

#ifndef __rtems_libbsd__
static void svrs_sysinit(void) {
	rtems_status_code sc;
#ifndef CONFIGURE_IRQ_SERVER_DISABLE
	sc = rtems_interrupt_server_initialize(CONFIGURE_IRQ_SERVER_PRIO, 
CONFIGURE_IRQ_SERVER_STKSZ,
		RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES, NULL);
  if (sc != RTEMS_SUCCESSFUL)
    rtems_panic("irq-server init failed\n");
#endif /* CONFIGURE_IRQ_SERVER_DISABLE */

#ifndef CONFIGURE_TMS_SERVER_DISABLE
	sc =  rtems_timer_initiate_server(CONFIGURE_TMS_SERVER_PRIO, 
  CONFIGURE_TMS_SERVER_STKSZ, RTEMS_DEFAULT_ATTRIBUTES );
  if (sc != RTEMS_SUCCESSFUL)
    rtems_panic("timer-server init failed\n");
#endif /* CONFIGURE_TMS_SERVER_DISABLE */

#ifndef CONFIGURE_MEDIA_DISABLE
  rtems_media_initialize();
  sc = rtems_media_server_initialize(CONFIGURE_INIT_TASK_PRIORITY, 
  CONFIGURE_MEDIA_SERVER_STKSZ,
  RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES);
  if (sc != RTEMS_SUCCESSFUL) {
    printf("%s: create media server failed(%s)\n", __func__, 
    rtems_status_text(sc));
  }
#endif /* CONFIGURE_MEDIA_DISABLE */
  (void) sc;
}

RTEMS_SYSINIT_ITEM(svrs_sysinit, 
  RTEMS_SYSINIT_ROOT_FILESYSTEM, 
  RTEMS_SYSINIT_LAST
);
#endif /* !__rtems_libbsd__ */

static void rootfs_init(void) {
  extern char __rootfs_image[];
  extern char __rootfs_size[];
  int err = Untar_FromMemory(__rootfs_image, (size_t)__rootfs_size);
  if (err)
    rtems_panic("make rootfs failed(%d)\n", err);
}

rtems_task Init(rtems_task_argument arg) {
  (void) arg;
  sysinit_post();
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
