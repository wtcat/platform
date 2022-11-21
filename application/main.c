/*
 * Copyright 2022 wtcat
 */
#ifdef __rtems_libbsd__
#include <machine/rtems-bsd-kernel-space.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <sys/kthread.h>
#include <dev/evdev/input.h>
#endif /* __rtems_libbsd__ */
#include <stdio.h>

/* Application Entry */
int rtems_main(void) {
#ifdef __rtems_libbsd__
    int fd;

    fd = open("/dev/input/event0", O_RDONLY);
    if (fd < 0) {
        printf("%s: Open device failed!", __func__);
        kthread_exit();
    }

    while (1) {
        struct input_event event;
        if (read(fd, &event, sizeof(event)) > 0)
            printf("key event: code(%d) value(%d)\n", event.code, event.value);
        else
            printf("key event read error\n");     
    }
#endif /* __rtems_libbsd__ */
    return 0;
}
