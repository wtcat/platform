
#include <stdbool.h>
#include <stdio.h>

#include "component/compiler.h"

struct op_class {
    int (*start)(void);
    bool (*exec)(void);
    void (*end)(void);
};

static inline int __test_generic_func(const struct op_class *ops) {
    int ret = ops->start();
    bool okay = false;
    switch (ret) {
    case 0:
        ops->end();
        break;
    case 1:
        okay = ops->exec();
        if (okay)
            ops->end();
    default:
        break;
    }
    return ret;
}
static int __test_var;

static int __noinline __test_start(void) {
    if (__test_var++ == 0) {
        printf("starting...\n");
        return 1;
    }
    return 0;
}

static bool __noinline __test_exec(void) {
    if (__test_var) {
        printf("executing...\n");
        return true;
    }
    return false;
}

static void __noinline __test_stop(void) {
    if (__test_var > 0) {
        __test_var--;
        if (__test_var == 0)
            printf("stoping...\n");
    }
}

void test_func(void) {
    static const struct op_class test_interface = {
        .start = __test_start,
        .exec = __test_exec,
        .end = __test_stop
    };
    __test_generic_func(&test_interface);
}