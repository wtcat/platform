
/*
 * Copyright (c) 2022 wtcat
 */
#include <errno.h>
#include <string.h>

#include <rtems.h>
#include <rtems/thread.h>

#define CALLPATH_LOCKCTX_DECLARE
#define CALLPATH_LOCK_DECLARE(_name)
#define CALLPATH_LOCK(_pobj)
#define CALLPATH_UNLOCK(_pobj)

typedef struct k_thread thread_t;

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





static int callpath_push(struct call_path *path, uintptr_t addr) {
    CALLPATH_LOCKCTX_DECLARE
	if (path == NULL)
		return -EINVAL;
    CALLPATH_LOCK(path);
	unsigned int ptr = path->ptr;
	if (ptr > 0) {
		path->nodes[ptr].addr = addr;
		path->ptr = ptr - 1;
		CALLPATH_UNLOCK(path);
		return 0;
	}
	CALLPATH_UNLOCK(path);
	return -EINVAL;
}

static int callpath_pop(struct call_path *path, uintptr_t addr) {
    CALLPATH_LOCKCTX_DECLARE
	if (path == NULL)
		return -EINVAL;
	CALLPATH_LOCK(path);
	if (path->nodes[path->ptr].addr == addr) {
		path->ptr++;
		CALLPATH_UNLOCK(path);
		return 0;
	}
	CALLPATH_UNLOCK(path);
	ll_info("Error ***: Callpath not match(%d 0x%08x 0x%08x)\n",
		path->ptr, path->nodes[path->ptr].addr, addr);
	return -EINVAL;
}

int callpath_print(thread_t *thread, struct ll_printer *printer) {
    CALLPATH_LOCKCTX_DECLARE
	if (!thread || !thread->custom_data || !printer)
		return -EINVAL;
	struct call_path temp_path, *path = thread->custom_data;
	CALLPATH_LOCK(path);
	memcpy(&temp_path, path, sizeof(temp_path));
	CALLPATH_UNLOCK(path);

	if (temp_path.magic != CALLPATH_MAGIC)
		return -EINVAL;
	ll_print(printer, "\nCallPath: addr2line -e app.axf -a -f");
	for (unsigned int i = temp_path.ptr; i < CALLPATH_MAX_DEEP; i++) {
		uintptr_t addr = temp_path.nodes[i].addr & ~0x1ul;
		ll_print(printer, " 0x%08x", addr);
	};
	ll_print(printer, "\n");
	return 0;
}

int callpath_init(thread_t *thread, struct call_path *path) {
	if (thread == NULL || path == NULL)
		return -EINVAL;
	if (thread->custom_data)
		return -EBUSY;
	path->magic = CALLPATH_MAGIC;
	path->ptr = CALLPATH_MAX_DEEP;
	thread->custom_data = path;
	return 0;
}

void __cyg_profile_func_enter(void *this_fn, void *call_site) {
    (void) call_site;
	callpath_push(NULL, (uintptr_t)this_fn);
}

void __cyg_profile_func_exit(void *this_fn, void *call_site) {
    (void) call_site;
	callpath_pop(NULL, (uintptr_t)this_fn);
} 

// -finstrument-functions
