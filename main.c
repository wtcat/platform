#include <stdint.h>
#include <stddef.h>
#include <stdio.h>


typedef struct sys_snode_s {
    struct sys_snode_s *next;
    struct sys_snode_s *prev;
} sys_dnode_t;

struct _timeout {
	sys_dnode_t node;
	void *fn;
	int32_t dticks;
};

struct bt_gatt_indicate_params {
	const void *uuid;
	const void *attr;
	void *func;
	void *destroy;
	const void *data;
	uint16_t len;
	uint8_t _ref;
};

struct k_work {
	sys_dnode_t node;
	void *handler;
	void *queue;
	uint32_t flags;
};

struct k_work_delayable {
	struct k_work work;
	struct _timeout timeout;
	void *queue;
};

static struct gatt_sc {
	struct bt_gatt_indicate_params params;
	uint16_t start;
	uint16_t end;
	struct k_work_delayable work;
    uint32_t flags;
};


/*! Union of different values */
union drvmgr_key_value {
	unsigned long		i;	/*!< Key data type UNSIGNED INTEGER */
	char			*str;	/*!< Key data type STRING */
	void			*ptr;	/*!< Key data type ADDRESS/POINTER */
};

/* One key. One Value. Holding information relevant to the driver. */
struct drvmgr_key {
	char			*key_name;	/* Name of key */
	int		key_type;	/* How to interpret key_value */
	union drvmgr_key_value	key_value;	/* The value or pointer to value */
};

#define DRVMGR_KEY_EMPTY	{NULL, 0, {0}}
#define RESOURCE_TERMINAL {NULL, NULL, }
#define RESOURCE_BASE_DECLARE \
		const char *compatible; \
		const char *name; \
		int parent_bus; \
		const struct drvmgr_key keys[];

struct bus_resource {
	RESOURCE_BASE_DECLARE
};

#define TRN(rname, type, value) {rname, type, {value}}
#define TEMPLATE_RESOURCE(_name, _compatible, _devname, _parent, ...) \
	static const struct resoruce_##_name {\
		RESOURCE_BASE_DECLARE } \
		_name = { \
			.compatible = _compatible, \
			.name = _devname, \
			.parent_bus = _parent, \
			.keys = { __VA_ARGS__, DRVMGR_KEY_EMPTY } \
		}

#define TEMPLATE_RESOURCES(_name, ...) \
	static const void *_name[] = { \
		__VA_ARGS__, {0} \
	}

#define RN(node) (void *)(&node)

TEMPLATE_RESOURCE(uart0, "st,uart", "uart0", 0,
	TRN("REG0", 0, 0x100000000),
	TRN("IRQ0", 0, 0X16)
);

TEMPLATE_RESOURCE(spi0, "st,uart", "uart0", 0,
	TRN("REG0", 0, 0x102000000),
	TRN("IRQ0", 0, 0X11)
);

TEMPLATE_RESOURCES(platform_resources,
	RN(uart0),
	RN(spi0)
);

#define print_struct_size(type) \
    printf("Sizeof(" #type "): %d\n", (int)sizeof(type));

#define print_member_offset(type, filed) \
    printf("Offset(" #type ":" #filed "): %d\n", (int)offsetof(type, filed))

#define printf_resource(resources) \
	print_resource_info(const struct bus_resource *)resources))


static void print_resource_info(const struct bus_resource *r) {
	const struct drvmgr_key *keys = r->keys;
	printf("Compatible: %s\nName: %s\n", r->compatible, r->name);
	while (keys->key_name) {
		printf("%s: %d\n", keys->key_name, (int)keys->key_value.i);
		keys++;
	}
}

static void print_all_resources(const struct bus_resource *r) {
	while (r->compatible) {
		print_resource_info(r);
		r++;
	}
}

int main(int argc, char *argv[]) {
    print_struct_size(struct sys_snode_s);
    print_struct_size(struct _timeout);
    print_struct_size(struct bt_gatt_indicate_params);
    print_struct_size(struct k_work);
    print_struct_size(struct k_work_delayable);
    print_struct_size(struct gatt_sc);

    print_member_offset(struct k_work_delayable, timeout);
    print_member_offset(struct k_work_delayable, queue);

    print_member_offset(struct gatt_sc, start);
    print_member_offset(struct gatt_sc, end);
    print_member_offset(struct gatt_sc, work);
    print_member_offset(struct gatt_sc, flags);

	print_resource_info((const struct bus_resource *)&uart0);
	print_resource_info((const struct bus_resource *)&spi0);
	print_all_resources((const struct bus_resource *)&platform_resources);
    return 0;
}