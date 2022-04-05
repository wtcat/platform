/*
 * CopyRight 2022 wtcat
 */
#include <stdio.h>
#include <stdlib.h>
#include <rtems.h>

#include "gtest/gtest.h"


// int fileno(FILE* file) {
//     return 1;
// }

// char* strdup(const char* string) {
//     size_t len = strlen(string);
//     char *dup = malloc(len + 1);
//     if (dup == NULL)
//         return dup;
//     dup[len] = '\0';
//     return memcpy(dup, string, len);
// }

extern "C" int main(void) {
    rtems_task_priority prio;
    rtems_status_code sc;
    int argc = 1;
    char *argv[] = {(char *)" "};
    sc = rtems_task_set_priority(rtems_task_self(), 100, &prio);
    _Assert(sc == RTEMS_SUCCESSFUL);
    (void) prio;

    printf("Running main() from %s\n", __FILE__);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}