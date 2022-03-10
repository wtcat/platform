#ifndef AM43XX_IDK_EVM_SHELLCONF_H_
#define AM43XX_IDK_EVM_SHELLCONF_H_

#define CONFIG_SHELL_STACKSZ 8192

#define CONFIG_DYNMAIC_LIB_CONTENT \
    "/etc/libdl*.a\n"
    
#define CONFIG_JOEL_SCRIPT_CONTENT \
    "#! joel -t JOEL -p 10 -s 8192\n" \
    "sleep 2\n" \
    "mkdir /home\n" \
    "mount -t dosfs /dev/fdisk /home\n"

#define CONFIGURE_SHELL_COMMAND_IRQ
#define CONFIGURE_SHELL_COMMANDS_INIT
#define CONFIGURE_SHELL_COMMANDS_ALL
#endif /* AM43XX_IDK_EVM_SHELLCONF_H_ */
