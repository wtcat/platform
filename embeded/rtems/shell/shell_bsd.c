/*
 * Copyright 2022 wtcat
 */
#include "configs/rtems_confdefs.h"
#include "shell/shell_utils.h"

#define BSD_SHELL_COMMAND_DECLARE(key) \
    extern rtems_shell_cmd_t rtems_shell_##key##_Command
#define BSD_SHELL_COMMAND(key) \
    &rtems_shell_##key##_Command

BSD_SHELL_COMMAND_DECLARE(HOSTNAME);
BSD_SHELL_COMMAND_DECLARE(ARP);
BSD_SHELL_COMMAND_DECLARE(DHCPCD);
BSD_SHELL_COMMAND_DECLARE(I2C);
BSD_SHELL_COMMAND_DECLARE(IFCONFIG);
BSD_SHELL_COMMAND_DECLARE(NETSTAT);
BSD_SHELL_COMMAND_DECLARE(NVMECONTROL);
BSD_SHELL_COMMAND_DECLARE(OPENSSL);
BSD_SHELL_COMMAND_DECLARE(PFCTL);
BSD_SHELL_COMMAND_DECLARE(PING);
BSD_SHELL_COMMAND_DECLARE(RACOON);
BSD_SHELL_COMMAND_DECLARE(ROUTE);
BSD_SHELL_COMMAND_DECLARE(SETKEY);
BSD_SHELL_COMMAND_DECLARE(STTY);
BSD_SHELL_COMMAND_DECLARE(SYSCTL);
BSD_SHELL_COMMAND_DECLARE(TCPDUMP);
BSD_SHELL_COMMAND_DECLARE(VMSTAT);
BSD_SHELL_COMMAND_DECLARE(WLANSTATS);
BSD_SHELL_COMMAND_DECLARE(WPA_SUPPLICANT);
BSD_SHELL_COMMAND_DECLARE(WPA_SUPPLICANT_FORK);


SHELL_CMDS_REGISTER(bsd_cmds, 
    BSD_SHELL_COMMAND(IFCONFIG),
    BSD_SHELL_COMMAND(PING),
#if defined(CONFIGURE_BSD_SHELL_COMMAND_HOSTNAME)
    BSD_SHELL_COMMAND(HOSTNAME),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_ARP)
    BSD_SHELL_COMMAND(ARP),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_DHCPCD)
    BSD_SHELL_COMMAND(DHCPCD),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_I2C)
    BSD_SHELL_COMMAND(I2C),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_NETSTAT)
    BSD_SHELL_COMMAND(NETSTAT),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_NVMECONTROL)
    BSD_SHELL_COMMAND(NVMECONTROL),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_OPENSSL)
    BSD_SHELL_COMMAND(OPENSSL),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_PFCTL)
    BSD_SHELL_COMMAND(PFCTL),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_RACOON)
    BSD_SHELL_COMMAND(RACOON),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_ROUTE)
    BSD_SHELL_COMMAND(ROUTE),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_SETKEY)
    BSD_SHELL_COMMAND(SETKEY),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_STTY)
    BSD_SHELL_COMMAND(STTY),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_SYSCTL)
    BSD_SHELL_COMMAND(SYSCTL),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_TCPDUMP)
    BSD_SHELL_COMMAND(TCPDUMP),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_VMSTAT)
    BSD_SHELL_COMMAND(VMSTAT),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_WLANSTATS)
    BSD_SHELL_COMMAND(WLANSTATS),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_WPA_SUPPLICANT)
    BSD_SHELL_COMMAND(WPA_SUPPLICANT),
#endif
#if defined(CONFIGURE_BSD_SHELL_COMMAND_WPA_SUPPLICANT_FORK)
    BSD_SHELL_COMMAND(WPA_SUPPLICANT_FORK),
#endif
);
