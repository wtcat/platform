#include <stdio.h>

int rtems(int argc, char *argv[]) {
    (void) argc;
    (void) argv;
    printf("Hello\n");
    return 0;
}