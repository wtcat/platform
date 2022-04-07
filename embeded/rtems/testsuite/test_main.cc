/*
 * CopyRight 2022 wtcat
 */
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <limits.h>
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

extern "C" {

int flsl(long i) {
	if (i == 0)
		return 0;
	return (sizeof(i) * CHAR_BIT - __builtin_clzl(i));
}

static uint32_t change_prio(uint32_t pri) {
    rtems_task_priority prio;
    rtems_status_code sc;
    sc = rtems_task_set_priority(rtems_task_self(), pri, &prio);
    _Assert(sc == RTEMS_SUCCESSFUL);
    return prio;
}

int app_main(void) {
    int argc = 1;
    char *argv[] = {(char *)" "};

    change_prio(100);
    printf("Running main() from %s\n", __FILE__);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

} //extern "C"