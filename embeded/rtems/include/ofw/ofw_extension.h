/*
 * Copyright 2022 wtcat
 */
#ifndef OFW_OFW_EXTENSION_H_
#define OFW_OFW_EXTENSION_H_

#include <ofw/ofw.h>

#ifdef __cplusplus
extern "C"{
#endif

struct ofw_phandle_args {
#define OFW_MAX_PHANDLE_ARGS 16
	phandle_t np; //struct device_node *np;
	int args_count;
	uint32_t args[OFW_MAX_PHANDLE_ARGS];
};

int ofw_count_phandle_with_args(phandle_t np, const char *list_name, 
	const char *cells_name, int cell_count);
int ofw_parse_phandle_with_args(phandle_t np, const char *list_name, 
	const char *cells_name, int cell_count, int index,
	struct ofw_phandle_args *out_args);
phandle_t ofw_parse_phandle(phandle_t np, const char *phandle_name, int index);

int ofw_property_match_string(phandle_t np, const char *propname, 
	const char *string, char *buffer, size_t bufsz);
int ofw_property_read_string_helper(phandle_t node, const char *propname, 
    const char **out_strs, size_t sz, int skip, char *buffer, size_t bufsz);

static inline int ofw_property_read_string_array(phandle_t node, const char *propname, 
    const char **out_strs, size_t sz, char *buffer, size_t size) {
	return ofw_property_read_string_helper(node, propname, out_strs, sz, 0, 
        buffer, size);
}

static inline int ofw_property_count_strings(phandle_t node, const char *propname, 
    char *buffer, size_t size) {
	return ofw_property_read_string_helper(node, propname, NULL, 0, 0, 
        buffer, size);
}

static inline int ofw_property_read_string_index(phandle_t node, const char *propname,
    int index, const char **output, char *buffer, size_t size) {
	int rc = ofw_property_read_string_helper(node, propname, output, 1, index, 
        buffer, size);
	return rc < 0 ? rc : 0;
}

#ifdef __cplusplus
}
#endif
#endif /* OFW_OFW_EXTENSION_H_ */
