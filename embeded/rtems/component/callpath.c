
/*
 * Copyright (c) 2022 wtcat
 */
#include <errno.h>
#include <string.h>

#if defined(__rtems__)
#include <stdlib.h>

#include <rtems/score/objectimpl.h>
#include <rtems/score/threadimpl.h>
#include <rtems/score/thread.h>
#include <rtems/score/percpu.h>
#include <rtems/score/statesimpl.h>
#include <rtems/score/sysstate.h>
#include <rtems.h>
#include <rtems/malloc.h>
#include <rtems/bspIo.h>
#include <rtems/sysinit.h>

#include "component/callpath.h"
#include "component/compiler.h"

#define CALLPATH_LOCKCTX_DECLARE \
	rtems_interrupt_lock_context lock_context;
#define CALLPATH_LOCK_DECLARE(_name) \
	rtems_interrupt_lock _name
#define CALLPATH_LOCK(_pobj) \
	rtems_interrupt_lock_acquire(&(_pobj)->lock, &lock_context)
#define CALLPATH_UNLOCK(_pobj) \
	rtems_interrupt_lock_release(&(_pobj)->lock, &lock_context)
#define CALLPATH_PRINT(printer, fmt, ...) \
	rtems_printf(printer, fmt, ##__VA_ARGS__)

typedef struct _Thread_Control thread_t;

#elif defined(__ZEPHYR__)
#include <zephyr.h>
#include <spinlock.h>

#define CALLPATH_LOCKCTX_DECLARE \
	k_spinlock_key_t __key;
#define CALLPATH_LOCK_DECLARE(_name) \
	struct k_spinlock _name
#define CALLPATH_LOCK(_pobj) \
	__key = k_spin_lock(&(_pobj)->lock)
#define CALLPATH_UNLOCK(_pobj) \
	k_spin_unlock(&(_pobj)->lock, __key);
#define CALLPATH_PRINT(printer, fmt, ...)

typedef struct k_thread thread_t;

#else
#error "Unknown operation system"
#endif

struct call_node {
	uintptr_t addr;
};

struct call_path {
#define CALLPATH_MAX_DEEP 100
#define CALLPATH_MAGIC 0xdeadbeef
	unsigned int magic;
	CALLPATH_LOCK_DECLARE(lock);
	/* protected by call_path.lock */
	unsigned int ptr;
	struct call_node nodes[CALLPATH_MAX_DEEP];
};

static int __notrace callpath_push(struct call_path *path, uintptr_t addr) {
    CALLPATH_LOCKCTX_DECLARE
	if (path == NULL)
		return -EINVAL;
    CALLPATH_LOCK(path);
	unsigned int ptr = path->ptr;
	if (ptr > 0) {
		path->nodes[--ptr].addr = addr;
		path->ptr = ptr;
		CALLPATH_UNLOCK(path);
		return 0;
	}
	CALLPATH_UNLOCK(path);
	return -EINVAL;
}

static int __notrace callpath_pop(struct call_path *path, uintptr_t addr) {
    CALLPATH_LOCKCTX_DECLARE
	if (path == NULL)
		return -EINVAL;
	CALLPATH_LOCK(path);
	if (path->ptr < CALLPATH_MAX_DEEP && 
		path->nodes[path->ptr].addr == addr) {
		path->ptr++;
		CALLPATH_UNLOCK(path);
		return 0;
	}
	CALLPATH_UNLOCK(path);
	printk("Error ***: Callpath not match(%d 0x%08x 0x%08x)\n",
		path->ptr, path->nodes[path->ptr].addr, addr);
	return -EINVAL;
}

static int __notrace callpath_init(struct call_path *path) {
	if (path == NULL)
		return -EINVAL;
	memset(path, 0, sizeof(path));
	path->magic = CALLPATH_MAGIC;
	path->ptr = CALLPATH_MAX_DEEP;
	return 0;
}

static const char *__notrace kernel_symbols(unsigned long addr) {
#define SYMBOL_MAGIC 0xFF000000ul
	unsigned long *ptr= (unsigned long *)(addr - 4);
	if ((*ptr & SYMBOL_MAGIC) == SYMBOL_MAGIC) {
		unsigned long ofs = *ptr & ~SYMBOL_MAGIC;
		return (char *)ptr - ofs;
	}
	return "Unknown";
}

#if defined(__rtems__)
static size_t callpath_extension_index;

static inline struct call_path *__notrace thread_get_callpath(const thread_t *thread) {
	return thread->extensions[callpath_extension_index];
}

static inline void __notrace thread_set_callpath(thread_t *thread, 
	void *extension) {
	thread->extensions[callpath_extension_index] = extension;
}

static inline thread_t *__notrace thread_get_current(void) {
	return _Thread_Get_executing();
}

static inline struct call_path *__notrace current_callpath(void) {
	return thread_get_callpath(thread_get_current());
}

static bool __notrace callpath_extension_thread_create(Thread_Control *executing,
	Thread_Control *created) {
	(void)executing;
	struct call_path *path;
	if (thread_get_callpath(created))
		return false;
	path = rtems_malloc(sizeof(*path));
	if (path) {
		callpath_init(path);
		thread_set_callpath(created, path);
		return true;
	}
	return false;
}

static void __notrace callpath_extension_thread_delete(Thread_Control *executing,
	Thread_Control *deleted) {
	struct call_path *path = thread_get_callpath(deleted);
	if (path) {
		thread_set_callpath(deleted, NULL);
		free(path);
	}
	(void)executing;
}

static const rtems_extensions_table callpath_extensions = {
	.thread_create = callpath_extension_thread_create,
	.thread_delete = callpath_extension_thread_delete
};

static void __notrace callpath_early_init(void) {
	rtems_status_code sc;
	rtems_id ext_id;
	sc = rtems_extension_create(rtems_build_name('c','a','l','l'),
		&callpath_extensions, &ext_id);
	if (sc != RTEMS_SUCCESSFUL)
		rtems_panic("cannot create extension");
	callpath_extension_index = rtems_object_id_get_index(ext_id);

}

RTEMS_SYSINIT_ITEM(callpath_early_init, 
	RTEMS_SYSINIT_ROOT_FILESYSTEM, RTEMS_SYSINIT_ORDER_MIDDLE);

#elif defined(__ZEPHYR__)
static inline struct call_path *__notrace thread_get_callpath(const thread_t *thread) {
	return (struct call_path *)thread->custom_data;
}

static inline void __notrace thread_set_callpath(thread_t *thread, 
	void *extension) {
	thread->custom_data = extension;
}

static inline struct call_path *__notrace current_callpath(void) {
	return thread_get_callpath(k_current_get());
}

void __notrace sys_trace_k_thread_create(thread_t *new_thread) {
	if (!thread_get_callpath(new_thread)) {
		struct call_path *path = k_malloc(sizeof(*path));
		if (path) {
			callpath_init(path);
			thread_set_callpath(new_thread, path);
		}
	}
}

void __notrace sys_trace_k_thread_sched_abort(thread_t *thread) {
	struct call_path *path = thread_get_callpath(thread);
	if (path) {
		thread_set_callpath(deleted, NULL);
		free(path);
	}
}

#else
static inline struct call_path *__notrace thread_get_callpath(const thread_t *thread) {
	return NULL;
}

static inline struct call_path *__notrace current_callpath(void) {
	return NULL;
}
#endif /* __rtems__ */

int __notrace callpath_print(void *thread, const callpath_printer_t *printer) {
    CALLPATH_LOCKCTX_DECLARE
	if (!thread || !printer)
		return -EINVAL;
	struct call_path temp_path, *path = thread_get_callpath(thread);
	CALLPATH_LOCK(path);
	memcpy(&temp_path, path, sizeof(temp_path));
	CALLPATH_UNLOCK(path);
	if (temp_path.magic != CALLPATH_MAGIC)
		return -EINVAL;
#if defined(__rtems__)
	CALLPATH_PRINT(printer, "[*CallPath()%d*]:\n", temp_path.ptr);
#else
	CALLPATH_PRINT(printer, "\nCallPath: addr2line -e app.axf -a -f");
#endif
	for (unsigned int lvl = 0, i = temp_path.ptr; 
		i < CALLPATH_MAX_DEEP; i++, lvl++) {
		uintptr_t addr = temp_path.nodes[i].addr & ~0x1ul;
#if defined(__rtems__)
		const char *sym = kernel_symbols(addr);
		CALLPATH_PRINT(printer, " [%2d] - <0x%lx>@ %s\n", lvl, addr, sym);
#else
		CALLPATH_PRINT(printer, " 0x%08x", addr);
#endif
	}
	CALLPATH_PRINT(printer, "\n");
	return 0;
}

int __notrace callpath_print_current(const callpath_printer_t *printer) {
	return callpath_print(thread_get_current(), printer);
}

void __notrace __cyg_profile_func_enter(void *this_fn, void *call_site) {
    (void) call_site;
	if (_System_state_Is_up(_System_state_Get())) {
		struct call_path *path = current_callpath();
		callpath_push(path, (uintptr_t)this_fn);
	}
}

void __notrace __cyg_profile_func_exit(void *this_fn, void *call_site) {
    (void) call_site;
	if (_System_state_Is_up(_System_state_Get())) {
		struct call_path *path = current_callpath();
		callpath_pop(path, (uintptr_t)this_fn);
	}
} 

