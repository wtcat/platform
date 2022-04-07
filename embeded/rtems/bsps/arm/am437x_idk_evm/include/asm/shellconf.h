#ifndef AM43XX_IDK_EVM_SHELLCONF_H_
#define AM43XX_IDK_EVM_SHELLCONF_H_

#define CONFIG_SHELL_STACKSZ 4096

#define CONFIG_DYNMAIC_LIB_CONTENT \
    "/etc/libdl*.a\n"
    
#define CONFIG_JOEL_SCRIPT_CONTENT \
    "#! joel -v -t JOEL -p 110 -s 8192\n" \
    "sleep 3\n" \
    "mkdir /temp\n" \
    "mkdos /dev/ram0\n" \
    "mount -t dosfs /dev/ram0 /temp\n"

#define CONFIGURE_SHELL_COMMAND_XMODEM
#define CONFIGURE_SHELL_COMMAND_IRQ
#define CONFIGURE_SHELL_COMMAND_CLEAR
#define CONFIGURE_SHELL_COMMANDS_INIT
#define CONFIGURE_SHELL_COMMANDS_ALL
#endif /* AM43XX_IDK_EVM_SHELLCONF_H_ */
