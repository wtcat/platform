#ifndef AM43XX_IDK_EVM_NETCONF_H_
#define AM43XX_IDK_EVM_NETCONF_H_

#if defined(__rtems_libbsd__)
#define CONFIGURE_FDT
#define CONFIGURE_ETC_RC_CONF_CONTENT \
    "syslog_priority=\"debug\"\n"    \
    "hostname=\"www.rtems.org\"\n"   \
    "ifconfig_cpsw0=\"inet 192.168.1.100 netmask 255.255.255.0 up\"\n" \
    "defaultrouter=\"192.168.1.1\"\n" \
    "defaultroute_delay=\"5\"\n"        \
    "telnetd_enable=\"YES\"\n"          \
    "telnetd_options=\"-v -C 1\"\n"   \
    "ftpd_enable=\"YES\"\n"                    \
    "ftpd_options=\"-v -p 21 -C 1\"\n"

/*
 * libbsd configure options
 */
#define RTEMS_BSD_CONFIG_DOMAIN_PAGE_MBUFS_SIZE (8 * 1024 * 1024ul)
#define RTEMS_BSD_CONFIG_NET_PF_UNIX
// #define RTEMS_BSD_CONFIG_NET_IP_MROUTE
// #define RTEMS_BSD_CONFIG_NET_IP6_MROUTE
//#define RTEMS_BSD_CONFIG_NET_IF_BRIDGE
// #define RTEMS_BSD_CONFIG_NET_IF_LAGG
// #define RTEMS_BSD_CONFIG_NET_IF_VLAN
//#define RTEMS_BSD_CONFIG_IPSEC
// #define RTEMS_BSD_CONFIG_FIREWALL_PF
// #define RTEMS_BSD_CONFIG_FIREWALL_PFLOG
// #define RTEMS_BSD_CONFIG_FIREWALL_PFSYNC
#define RTEMS_BSD_CONFIG_TERMIOS_KQUEUE_AND_POLL
    
#else /* !__rtems_libbsd__ */


#endif /* __rtems_libbsd__ */
#endif /* AM43XX_IDK_EVM_NETCONF_H_*/
