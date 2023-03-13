/*
 * Copyright 2022 wtcat
 */
#ifdef __rtems_libbsd__
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <rtems.h>

#include <sys/event.h>
#include <dev/evdev/input.h>
#endif /* __rtems_libbsd__ */


/* Application Entry */
int rtems_main(void) {
#ifdef __rtems_libbsd__
    struct kevent event[2];
    int ret;
    int fd;

    fd = open("/dev/input/event0", O_RDONLY);
    if (fd < 0) {
        printf("%s: Open device failed!", __func__);
        rtems_task_exit();
    }

    int kq = kqueue();
    if (kq == -1) {
        printf("create kqueue failed\n");
        return errno;
    }

    EV_SET(&event[0], fd, EVFILT_READ, EV_ADD|EV_ENABLE, 0, 0, NULL);
    ret = kevent(kq, &event[0], 1, NULL, 0, NULL);
    if (ret == -1 || (event[0].flags & EV_ERROR)) {
        printf("kqueue add event failed\n");
        return errno;
    }
    
    while (1) {
        ret = kevent(kq, NULL, 0, &event[1], 1, NULL);
        if (ret < 0) {
            printf("kqueue wait event failed\n");
            break;
        }

        printf("## kevent: num(%d) filter(0x%x) flags(0x%x) fflags(0x%x) data(%lld) udata(%p)\n", 
            ret, event[1].filter, event[1].flags, event[1].fflags, event[1].data,event[1].udata);

        if (event[1].filter == EVFILT_READ) {
            size_t nevt = event[1].data / sizeof(struct input_event);
            struct input_event ie[nevt];
            if (read(fd, ie, (size_t)event[1].data) > 0) {
                for (size_t i = 0; i < nevt; i++)
                    printf("key event: code(%d) value(%d)\n", ie[i].code, ie[i].value);
            }
        }
        
        // struct input_event event;
        // if (read(fd, &event, sizeof(event)) > 0)
        //     printf("key event: code(%d) value(%d)\n", event.code, event.value);
        // else
        //     printf("key event read error\n");     
    }
#endif /* __rtems_libbsd__ */
    return 0;
}
