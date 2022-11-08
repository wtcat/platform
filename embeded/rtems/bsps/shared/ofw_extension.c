/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <rtems/bspIo.h>

#include <ofw/ofw.h>
#include "base/byteorder.h"

#include "drivers/devbase.h"
#include "drivers/ofw_platform_bus.h"


static phandle_t ofw_find_node_by_phandle(phandle_t np) {
	struct drvmgr_dev *dev = ofw_device_get_by_phandle(np);
	if (dev) {
		struct dev_private *devp = device_get_private(dev);
		return devp->np;
	}
	return -1;
}

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

int ofw_property_match_string(phandle_t np, const char *propname, 
	const char *string, char *buffer, size_t bufsz) {
	const char *p, *end;
	int i, len;
	size_t l;

	len = rtems_ofw_get_prop(np, propname, buffer, bufsz);
	if (len < 0)
		return -EINVAL;
	p = buffer;
	end = p + len;

	for (i = 0; p < end; i++, p += l) {
		l = strnlen(p, end - p) + 1;
		if (p + l > end)
			return -EILSEQ;
		if (strcmp(string, p) == 0)
			return i; /* Found it; return index */
	}
	return -ENODATA;
}

static int __ofw_parse_phandle_with_args(phandle_t np, const char *list_name,
	const char *cells_name, int cell_count, int index, 
	struct ofw_phandle_args *out_args) {
	void *buffer;
	const uint32_t *list, *list_end;
	int rc = 0, cur_index = 0;
	uint32_t count;
	// struct device_node *node = NULL;
	phandle_t node, phandle;
	int size;

	/* Retrieve the phandle list property */
	size = rtems_ofw_get_prop_alloc(np, list_name, &buffer);
	if (size < 0)
		return -ENOENT;
	// list = of_get_property(np, list_name, &size);
	// if (!list)
	// 	return -ENOENT;
	list = (uint32_t *)buffer;
	list_end = list + size / sizeof(*list);

	/* Loop over the phandles until all the requested entry is found */
	while (list < list_end) {
		rc = -EINVAL;
		count = 0;

		/*
		 * If phandle is 0, then it is an empty entry with no
		 * arguments.  Skip forward to the next entry.
		 */
		phandle = be32_to_cpu(list++);
		if (phandle) {
			/*
			 * Find the provider node and parse the #*-cells
			 * property to determine the argument length.
			 *
			 * This is not needed if the cell count is hard-coded
			 * (i.e. cells_name not set, but cell_count is set),
			 * except when we're going to return the found node
			 * below.
			 */
			if (cells_name || cur_index == index) {
				node = ofw_find_node_by_phandle(phandle);
				if ((int)node < 0) {
					printk("%s: could not find phandle(%u)\n", __func__, phandle); 
					goto err;
				}
			}

			if (cells_name) {
				if (rtems_ofw_get_enc_prop(node, cells_name, &count, sizeof(count)) < 0) {
					printk("%s: could not get %s for node(0x%x)\n", __func__,
					      cells_name, node);
					goto err;
				}
			} else {
				count = cell_count;
			}

			/*
			 * Make sure that the arguments actually fit in the
			 * remaining property data length
			 */
			if (list + count > list_end) {
				printk("%s: arguments longer than property <count: %d>\n", __func__, count);
				goto err;
			}
		}

		/*
		 * All of the error cases above bail out of the loop, so at
		 * this point, the parsing is successful. If the requested
		 * index matches, then fill the out_args structure and return,
		 * or return -ENOENT for an empty entry.
		 */
		rc = -ENOENT;
		if (cur_index == index) {
			if (!phandle)
				goto err;

			if (out_args) {
				if (count > OFW_MAX_PHANDLE_ARGS) {
					printk("Warning***: count > OFW_MAX_PHANDLE_ARGS\n");
					count = OFW_MAX_PHANDLE_ARGS;
				}
				out_args->np = node;
				out_args->args_count = count;
				for (int i = 0; i < (int)count; i++)
					out_args->args[i] = be32_to_cpu(list++);
			} else {
				// of_node_put(node);
			}

			/* Found it! return success */
			free(buffer);
			return 0;
		}

		// of_node_put(node);
		// node = NULL;
		list += count;
		cur_index++;
	}

	/*
	 * Unlock node before returning result; will be one of:
	 * -ENOENT : index is for empty phandle
	 * -EINVAL : parsing error on data
	 * [1..n]  : Number of phandle (count mode; when index = -1)
	 */
	rc = index < 0 ? cur_index : -ENOENT;
 err:
	// if (node)
	// 	of_node_put(node);
	free(buffer);
	return rc;
}

phandle_t ofw_parse_phandle(phandle_t np, const char *phandle_name, int index) {
	struct ofw_phandle_args args;
	if (index < 0)
		return -EINVAL;
	if (__ofw_parse_phandle_with_args(np, phandle_name, NULL, 0, 
		index, &args)) 
		return -ENOENT;
	return args.np;
}

int ofw_parse_phandle_with_args(phandle_t np, const char *list_name, 
	const char *cells_name, int cell_count, int index,
	struct ofw_phandle_args *out_args) {
	if (index < 0)
		return -EINVAL;
	return __ofw_parse_phandle_with_args(np, list_name, cells_name,
			cell_count, index, out_args);
}

int ofw_count_phandle_with_args(phandle_t np, const char *list_name, 
	const char *cells_name, int cell_count) {
	return __ofw_parse_phandle_with_args(np, list_name, cells_name,
			cell_count, -1, NULL);
}
