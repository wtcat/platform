#include <rtems/rtems_bsdnet.h>

#ifdef CONFIG_NET_FTP
#include <rtems/ftpd.h>
#endif
#ifdef CONFIG_NET_NFS
#include <librtemsNfs.h>
#endif
#ifdef CONFIG_NET_TELNET
#include <rtems/telnetd.h>
#include <rtems/shell.h>
#endif

#include "bsp/sysconf.h"

#ifndef CONFIG_NET_TASK_PRIORITY
#define CONFIG_NET_TASK_PRIORITY 100
#endif
#ifndef CONFIG_NET_MBUF_SIZE
#define CONFIG_NET_MBUF_SIZE (128 * 512)
#endif
#ifndef CONFIG_NET_MBUF_CLUSTER_SIZE
#define CONFIG_NET_MBUF_CLUSTER_SIZE (2048 * 256)
#endif
#ifndef CONFIG_NET_HOST_NAME
#define CONFIG_NET_HOST_NAME "rtems"
#endif
#ifndef CONFIG_NET_DOMAIN_NAME
#define CONFIG_NET_DOMAIN_NAME " "
#endif
#ifndef CONFIG_NET_GATEWAY
#define CONFIG_NET_GATEWAY "192.168.199.1"
#endif

#ifndef CONFIG_NET_TELNET_TASK_PRIORITY
#define CONFIG_NET_TELNET_TASK_PRIORITY CONFIG_NET_TASK_PRIORITY
#endif
#ifndef CONFIG_NET_TELNET_MAX_CLIENTS
#define CONFIG_NET_TELNET_MAX_CLIENTS 1
#endif
#ifndef CONFIG_NET_TELNET_PORT
#define CONFIG_NET_TELNET_PORT  23
#endif
#ifndef CONFIG_NET_TELNET_TASK_STACK
#define CONFIG_NET_TELNET_TASK_STACK  8192
#endif

#ifndef CONFIG_NET_FTP_TASK_PRIORITY
#define CONFIG_NET_FTP_TASK_PRIORITY CONFIG_NET_TASK_PRIORITY
#endif
#ifndef CONFIG_NET_FTP_PORT
#define CONFIG_NET_FTP_PORT 21
#endif
#ifndef CONFIG_NET_FTP_MAX_CONNECTS
#define CONFIG_NET_FTP_MAX_CONNECTS 1
#endif
#ifndef CONFIG_NET_FTP_IDLE_TIMEOUT
#define CONFIG_NET_FTP_IDLE_TIMEOUT 300
#endif

#ifdef CONFIG_NET_TELNET
static void telnet_command(char *device, void *arg) {
    rtems_shell_login_check_t login = NULL;
    rtems_shell_env_t shell_env;
 //   if (telnet_login)
 //       login = rtems_shell_login_check;
    rtems_shell_dup_current_env(&shell_env);
    shell_env.devname       = device;
    shell_env.taskname      = "TELn";
    shell_env.exit_shell    = false;
    shell_env.forever       = 0;
    shell_env.echo          = 0;
    shell_env.input         = NULL;
    shell_env.output        = NULL;
    shell_env.output_append = 0;
    shell_env.wake_on_end   = 0;
    shell_env.login_check   = login;
    rtems_shell_main_loop (&shell_env); 
}

rtems_telnetd_config_table rtems_telnetd_config = {
    .command = telnet_command,
    .arg = NULL,
    .priority = CONFIG_NET_TELNET_TASK_PRIORITY, /* We feel important today */
    .stack_size = CONFIG_NET_TELNET_TASK_STACK, /* Shell needs a large stack */
    .login_check = NULL, /* Shell asks for user and password */
    .keep_stdio = false,
    .client_maximum = CONFIG_NET_TELNET_MAX_CLIENTS,
    .port = CONFIG_NET_TELNET_PORT
    
};
#endif /* CONFIG_NET_TELNET */

#ifdef CONFIG_NET_FTP
static rtems_status_code net_ftpd_init(void) {
    struct rtems_ftpd_configuration config = {
        .priority = CONFIG_NET_FTP_TASK_PRIORITY,        /* FTPD task priority */
        .max_hook_filesize = 0, /* Maximum buffersize for hooks */
        .port = CONFIG_NET_FTP_PORT,             /* Well-known port */
        .hooks = NULL,          /* List of hooks */
        .root = NULL,           /* Root for FTPD or NULL for "/" */
        .tasks_count = CONFIG_NET_FTP_MAX_CONNECTS,  /* Max. connections */
        .idle = CONFIG_NET_FTP_IDLE_TIMEOUT,  /* Idle timeout in seconds  or 0 for no (infinite) timeout */
        .access = 0,            /* Access: 0 - r/w, 1 - read-only, 2 - write-only,
                               * 3 - browse-only */
        .login = NULL,           /* Login */
        .verbose = IS_ENABLED(CONFIG_NET_FTP_VERBOSE)
    };
    return rtems_ftpd_start(&config);
}
#endif /* CONFIG_NET_FTP */

#ifdef CONFIG_NET_NFS
static int net_nfs_init(void) {   
    int ret = rpcUdpInit();
    if (ret) {
        printf("%s RPC initialize failed with error %d\n", __func__, ret);
        goto exit;
    }
    ret = nfsInit(0, 0);
    if (ret)
        printf("%s NFS initialize failed with error %d\n", __func__, ret);
exit:
    return ret;
}
#endif /* CONFIG_NET_NFS */


struct rtems_bsdnet_config rtems_bsdnet_config = {
    .ifconfig               = NULL,
    .bootp                  = NULL,
    .network_task_priority  = CONFIG_NET_TASK_PRIORITY,
    .mbuf_bytecount         = CONFIG_NET_MBUF_SIZE,
    .mbuf_cluster_bytecount = CONFIG_NET_MBUF_CLUSTER_SIZE,
    .hostname               = CONFIG_NET_HOST_NAME,
    .domainname             = CONFIG_NET_DOMAIN_NAME,
    .gateway                = CONFIG_NET_GATEWAY,
    .log_host               = "127.0.0.1",
    .name_server            = {"127.0.0.1" },
    .ntp_server             = {"127.0.0.1" },
    .sb_efficiency          = 1,
    .udp_tx_buf_size        = 0,
    .udp_rx_buf_size        = 0,
    .tcp_tx_buf_size        = 0,
    .tcp_rx_buf_size        = 0
};

int net0_init(void) {
    rtems_status_code sc;
    int ret = rtems_bsdnet_initialize_network();
    if (ret) {
        printf("%s initialize network failed\n", __func__);
        goto out;
    }
    rtems_bsdnet_show_inet_routes();
#ifdef CONFIG_NET_FTP
    sc = net_ftpd_init();
    if (sc != RTEMS_SUCCESSFUL) {
        printf("%s ftp start failed: %s\n", __func__, rtems_status_text(sc));
        goto out;
    }
#endif
#ifdef CONFIG_NET_TELNET
    sc = rtems_telnetd_initialize();
    if (sc != RTEMS_SUCCESSFUL) {
        printf("%s telnet start failed: %s\n", __func__, rtems_status_text(sc));
        goto out;
    }
#endif
#ifdef CONFIG_NET_NFS
    ret = net_nfs_init();
#endif
out:
    return ret;
}
