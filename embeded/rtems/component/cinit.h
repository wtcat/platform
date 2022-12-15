/*
 * Copyright 2022 wtcat
 */
#ifndef BASEWORK_CINIT_H_
#define BASEWORK_CINIT_H_

#ifdef __cplusplus
extern "C"{
#endif

enum cinit_order {
    kDeviceOrder      = 0x0000,
    kApplicationOrder = 0x0080,
};

int c_initializer_run(void);
int c_initializer_run_order(int order);

#ifdef __cplusplus
}
#endif
#endif /* BASEWORK_CINIT_H_ */
