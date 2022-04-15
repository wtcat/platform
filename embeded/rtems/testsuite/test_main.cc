/*
 * CopyRight 2022 wtcat
 */
#include <stdio.h>
#include <limits.h>

#include "gtest/gtest.h"

extern "C" int flsl(long i) {
	if (i == 0)
		return 0;
	return (sizeof(i) * CHAR_BIT - __builtin_clzl(i));
}

extern "C" int app_main(void) {
    int argc = 1;
    char *argv[] = {(char *)" "};
    printf("Running main() from %s\n", __FILE__);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
