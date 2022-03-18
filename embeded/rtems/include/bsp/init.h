#ifndef BSP_INIT_H_
#define BSP_INIT_H_

#ifdef __cplusplus
extern "C"{
#endif

int net0_init(void);
int shell_init(rtems_shell_login_check_t login_check);
int ramblk_init(void);

#ifdef __cplusplus
}
#endif
#endif /* BSP_INIT_H_ */
