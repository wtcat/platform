/*
 * Copyright (c) 2022 wtcat
 */
#ifndef COMPONENT_FINDBIT_H_
#define COMPONENT_FINDBIT_H_

#ifdef __cplusplus
extern "C"{
#endif

#define	ffs(_x)		__builtin_ffs((unsigned int)(_x))
#define	ffsl(_x)	__builtin_ffsl((unsigned long)(_x))
#define	ffsll(_x)	__builtin_ffsll((unsigned long long)(_x))
#define	fls(_x)		__fls(_x)
#define	flsl(_x)	__flsl(_x)
#define	flsll(_x)	__flsll(_x)

static inline int __fls(int x) {
    return (x != 0 ? sizeof(x) * 8 - __builtin_clz((unsigned int)x) : 0);
}

static inline int __flsl(long x) {
    return (x != 0 ? sizeof(x) * 8 - __builtin_clzl((unsigned long)x) : 0);
}

static inline int __flsll(long long x) {
    return (x != 0 ? sizeof(x) * 8 - __builtin_clzll((unsigned long long)x) : 0);
}

#ifdef __cplusplus
}
#endif
#endif /* COMPONENT_FINDBIT_H_ */
