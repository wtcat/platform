/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <string.h>

#include <ofw/ofw.h>

int ofw_property_read_string_helper(phandle_t node, const char *propname, 
    const char **out_strs, size_t sz, int skip, char *buffer, size_t bufsz) {
    int len;
	int l = 0, i = 0;
	const char *p, *end;

    len = rtems_ofw_get_prop(node, propname, buffer, bufsz);
	if (len < 0)
		return -EINVAL;
	p = buffer;
	end = p + len;
	for (i = 0; p < end && (!out_strs || i < skip + (int)sz); i++, p += l) {
		l = strnlen(p, end - p) + 1;
		if (p + l > end)
			return -EILSEQ;
		if (out_strs && i >= skip)
			*out_strs++ = p;
	}
	i -= skip;
	return i <= 0 ? -ENODATA : i;
}
