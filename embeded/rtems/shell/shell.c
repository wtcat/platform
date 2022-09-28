/*
 * CopyRight 2022 wtcat
 */
#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>

#include <rtems.h>
#include <rtems/shell.h>
#include <rtems/imfs.h>
#include <rtems/console.h>

#include <bsp/irq-info.h>

#include "shell/shell_vt100.h"
#include "bsp/board/sysconf.h"


#if defined(__rtems_libbsd__)
#define SHELL_STACKSZ  (32 * 1024)
#else // !__rtems_libbsd__

#ifndef CONFIGURE_SHELL_STACKSZ
#define SHELL_STACKSZ  (4 * 1024)
#else 
#define SHELL_STACKSZ  CONFIGURE_SHELL_STACKSZ
#endif 
#endif //__rtems_libbsd__

#ifndef CONFIGURE_SHELL_TASKPRIO
#define SHELL_PRIO  (CONFIGURE_MAXIMUM_PRIORITY - 3)
#else
#define SHELL_PRIO CONFIGURE_SHELL_TASKPRIO
#endif

/* Macro to send VT100 commands. */
#define SHELL_VT100_CMD(_cmd_) \
    do {				   \
        static const char cmd[] = _cmd_; \
        fprintf(stdout, "%s", cmd);	\
    } while (0)

#define BSD_SHELL_COMMAND(key) \
    extern rtems_shell_cmd_t rtems_shell_##key##_Command; \
    rtems_shell_add_cmd_struct(&rtems_shell_##key##_Command)

#define SHELL_COMMAND_ADD(key) \
    extern rtems_shell_cmd_t shell_##key##_command; \
    rtems_shell_add_cmd_struct(&shell_##key##_command)


#if defined(CONFIGURE_SHELL_COMMAND_CLEAR)
static int shell_main_clear(int argc, char *argv[]) {
    (void) argv;
    if (argc > 1)
        return -EINVAL;
    SHELL_VT100_CMD(SHELL_VT100_CURSORHOME);
    SHELL_VT100_CMD(SHELL_VT100_CLEARSCREEN);
    return 0;
}

static rtems_shell_cmd_t shell_clear_command = {
    "clear",                         /* name */
    "clear     # Clear screen",      /* usage */
    "rtems",                         /* topic */
    shell_main_clear,                /* command */
    NULL,                            /* aliass */
    NULL                             /* next */
};
#endif /* CONFIGURE_SHELL_COMMAND_CLEAR */

#if defined(CONFIGURE_SHELL_COMMAND_RAP)
#include <rtems/rtl/rap-shell.h>

static struct rtems_shell_cmd_tt rtems_rap_command = {
  .name     = "rap",
  .usage   = "Runtime load",
  .topic   = "rtems",
  .command = shell_rap,
  .alias   = NULL,
  .next    = NULL
};
#endif /* CONFIGURE_SHELL_COMMAND_RAP */

#if defined(CONFIGURE_SHELL_COMMAND_RTL)
#include <rtems/rtl/rtl-shell.h>

static struct rtems_shell_cmd_tt rtems_rtl_command = {
  .name     = "rtl",
  .usage   = "Runtime Linker",
  .topic   = "rtems",
  .command = rtems_rtl_shell_command,
  .alias   = NULL,
  .next    = NULL
};
#endif /* CONFIGURE_SHELL_COMMAND_RTL */

#if defined(CONFIGURE_SHELL_COMMAND_REBOOT)
static int shell_main_reboot(int argc, char *argv[]) {
    extern void bsp_reset(void);
    (void) argv;
    if (argc > 1)
        return -EINVAL;
    bsp_reset();
    return 0;
}

static rtems_shell_cmd_t shell_reboot_command = {
    "reboot",                                   /* name */
    "Reboot system immidateliy",                /* usage */
    "rtems",                                    /* topic */
    shell_main_reboot,                          /* command */
    NULL,                                       /* alias */
    NULL                                        /* next */
};
#endif /* CONFIG_SHELL_REBOOT */

static void shell_commands_register(void) {
#ifdef CONFIGURE_GDBSERVER
    BSD_SHELL_COMMAND(DEBUGGER);
#endif
#ifdef CONFIGURE_SHELL_COMMAND_XMODEM
    SHELL_COMMAND_ADD(xmodem);
#endif
#if defined(CONFIGURE_SHELL_COMMAND_DISK)
    extern rtems_shell_cmd_t shell_fdisk_command;
    rtems_shell_add_cmd_struct(&shell_fdisk_command);
#endif
#if defined(CONFIGURE_SHELL_COMMAND_CLEAR)
    rtems_shell_add_cmd_struct(&shell_clear_command);
#endif
#if defined(CONFIGURE_SHELL_COMMAND_IRQ)
    rtems_shell_add_cmd_struct(&bsp_interrupt_shell_command);        
#endif 
#if defined(CONFIGURE_SHELL_COMMAND_RAP)
    rtems_shell_add_cmd_struct(&rtems_rap_command);
#endif
#if defined(CONFIGURE_SHELL_COMMAND_RTL)
    rtems_shell_add_cmd_struct(&rtems_rtl_command);
#endif
#if defined(CONFIGURE_SHELL_COMMAND_REBOOT)
    rtems_shell_add_cmd_struct(&shell_reboot_command);
#endif
#ifdef __rtems_libbsd__
#if defined(CONFIGURE_BSD_SHELL_COMMAND_HOSTNAME)
    BSD_SHELL_COMMAND(HOSTNAME);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_ARP)
    BSD_SHELL_COMMAND(ARP);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_DHCPCD)
    BSD_SHELL_COMMAND(DHCPCD);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_I2C)
    BSD_SHELL_COMMAND(I2C);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_IFCONFIG)
    BSD_SHELL_COMMAND(IFCONFIG);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_NETSTAT)
    BSD_SHELL_COMMAND(NETSTAT);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_NVMECONTROL)
    BSD_SHELL_COMMAND(NVMECONTROL);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_OPENSSL)
    BSD_SHELL_COMMAND(OPENSSL);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_PFCTL)
    BSD_SHELL_COMMAND(PFCTL);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_PING)
    BSD_SHELL_COMMAND(PING);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_RACOON)
    BSD_SHELL_COMMAND(RACOON);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_ROUTE)
    BSD_SHELL_COMMAND(ROUTE);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_SETKEY)
    BSD_SHELL_COMMAND(SETKEY);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_STTY)
    BSD_SHELL_COMMAND(STTY);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_SYSCTL)
    BSD_SHELL_COMMAND(SYSCTL);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_TCPDUMP)
    BSD_SHELL_COMMAND(TCPDUMP);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_VMSTAT)
    BSD_SHELL_COMMAND(VMSTAT);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_WLANSTATS)
    BSD_SHELL_COMMAND(WLANSTATS);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_WPA_SUPPLICANT)
    BSD_SHELL_COMMAND(WPA_SUPPLICANT);
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_WPA_SUPPLICANT_FORK)
    BSD_SHELL_COMMAND(WPA_SUPPLICANT_FORK);
#endif
#endif /* __rtems_libbsd__ */
}

int shell_init(rtems_shell_login_check_t login_check) {
#if defined(CONFIGURE_SHELL_COMMANDS_INIT)
    rtems_status_code sc;
    shell_commands_register();
    sc = rtems_shell_init("root", SHELL_STACKSZ, SHELL_PRIO,
        CONSOLE_DEVICE_NAME, false, false, login_check);
    return rtems_status_code_to_errno(sc);
#else
    (void) login_check;
#endif //CONFIGURE_SHELL_COMMANDS_INIT
    return 0;
}

#include <rtems/shellconfig.h>
