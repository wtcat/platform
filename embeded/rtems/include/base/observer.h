/*
 * Copyright 2023 wtcat
 */
#ifndef BASE_CONTAINER_NOTIFIER_H_
#define BASE_CONTAINER_NOTIFIER_H_

#ifdef __cplusplus
extern "C"{
#endif


struct observer_base;
typedef int (*observer_fn_t)(struct observer_base *nb,
			unsigned long action, void *data);

struct observer_base {
	observer_fn_t update;
	struct observer_base  *next;
	int priority;
};


#define NOTIFY_DONE		0x0000		/* Don't care */
#define NOTIFY_OK		0x0001		/* Suits me */
#define NOTIFY_STOP_MASK	0x8000		/* Don't call further */
#define NOTIFY_BAD		(NOTIFY_STOP_MASK|0x0002)
						/* Bad/Veto action */
/*
 * Clean way to return from the notifier and stop further calls.
 */
#define NOTIFY_STOP		(NOTIFY_OK|NOTIFY_STOP_MASK)

#ifdef OBSERVER_CLASS_DEFINE
#define _OBSERVER_CLASS
#else
#define _OBSERVER_CLASS extern
#endif

/*
 * method:
 *  int _class_name##_notify(unsigned long action, void *ptr)
 *  int _class_name##_add_observer(struct observer_base *obs)
 *  int _class_name##_remove_observer(struct observer_base *obs)
 */
#define _OBSERVER_NAME(_class_name) _##_class_name##__observer_list_head
#define OBSERVER_CLASS_DECLARE(_class_name) \
    _OBSERVER_CLASS struct observer_base *_OBSERVER_NAME(_class_name); \
    static inline int _class_name##_notify(unsigned long value, void *ptr) { \
        return observer_notify(&_OBSERVER_NAME(_class_name), value, ptr); \
    } \
    static inline int _class_name##_add_observer(struct observer_base *obs) { \
        return observer_cond_register(&_OBSERVER_NAME(_class_name), obs); \
    } \
    static inline int _class_name##_remove_observer(struct observer_base *obs) { \
        return observer_unregister(&_OBSERVER_NAME(_class_name), obs); \
    }

#define OBSERVER_STATIC_INIT(call, pri) \
	{ \
		.update = call, \
		.next = NULL, \
		.priority = pri \
	}


int observer_register(struct observer_base **nl,
		struct observer_base *n);
int observer_cond_register(struct observer_base **nl,
		struct observer_base *n);
int observer_unregister(struct observer_base **nl,
		struct observer_base *n);
int observer_notify(struct observer_base **nl,
			       unsigned long val,
                   void *v);

#ifdef __cplusplus
}
#endif
#endif // BASE_CONTAINER_NOTIFIER_H_