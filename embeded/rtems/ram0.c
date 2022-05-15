#include <rtems/ramdisk.h>
#include "bsp/board/sysconf.h"

struct ram_block {
    uint32_t blksz;
    uint32_t capacity;
};

#define RAM_DISK(blksz, capactity) {blksz, capactity}
    
#ifdef CONFIGURE_RAMDISK
static const struct ram_block ramblk_cfg[] = {
    CONFIGURE_RAMDISK
};

int ramblk_init(void) {
    rtems_status_code sc = RTEMS_NOT_DEFINED;
    char devname[] = {"/dev/ramX"};
    for (int i = 0; i < RTEMS_ARRAY_SIZE(ramblk_cfg); i++) {
        uint32_t blksz = ramblk_cfg[i].blksz;
        uint32_t blkcnt = ramblk_cfg[i].capacity / blksz;
        devname[8] = i + '0';
        sc = ramdisk_register(blksz, blkcnt, false, devname);
        if (sc != RTEMS_SUCCESSFUL)
            break;
    }
    return rtems_status_code_to_errno(sc);
}

#else  /* !CONFIGURE_RAMDISK */
int ramblk_init(void) {
    return 0;
}
#endif /* CONFIGURE_RAMDISK */

