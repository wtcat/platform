/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <string.h>
#include <rtems/bspIo.h>
#include <sys/errno.h>

#include "drivers/clock.h"
#include "drivers/devbase.h"
#include "drivers/ofw_platform_bus.h"

#include "ofw/ofw_extension.h"


static int clk_ofw_xlate_default(struct clk *clk, struct ofw_phandle_args *args) {
	if (args->args_count > 1) {
		printk("Invaild args_count: %d\n", args->args_count);
		return -EINVAL;
	}
	if (args->args_count)
		clk->id = args->args[0];
	else
		clk->id = 0;
	return 0;
}

int ofw_clk_request(phandle_t np, int index, struct clk *clk) {
	struct ofw_phandle_args args;
    struct drvmgr_dev *dev;
	int ret;

    ret = ofw_parse_phandle_with_args(np, "clocks", "#clock-cells", 0,
                index, &args);
    if (ret)
        return ret;                 
    dev = ofw_device_get_by_devnode(args.np);
    if (!dev)
        return -ENODEV;
    if (clk_xlate(dev, clk, &args))
        clk_ofw_xlate_default(clk, &args);
    clk->dev = dev;
    return 0;
}

struct drvmgr_dev *ofw_clock_request(phandle_t np, const char *name, 
    pcell_t *pcell, size_t maxsize) {
    struct drvmgr_dev *dev = NULL;
    phandle_t clknp;
    pcell_t args[32];
    pcell_t clk_cells;
    char buffer[128];
    int count, ofs;
    size_t bytes;
    
    if (rtems_ofw_get_enc_prop(np, "clocks", args, sizeof(args)) < 0) {
        printk("%s: not found \"clocks\" property!\n", __func__);
        errno = -ENOSTR;
        goto _out;
    }
    if (name) {
        count = ofw_property_count_strings(np, "clock-names", 
            buffer, sizeof(buffer));
    } else {
        count = 1;
    }

    ofs = 0;
    for (int i = 0; i < count; i++) {
        const char *output = NULL;
        dev = ofw_device_get_by_phandle(args[ofs]);
        if (dev == NULL) {
            errno = -ENODEV;
            return NULL;
        }
        clknp = ofw_phandle_get(dev);
        if (rtems_ofw_get_enc_prop(clknp, "#clock-cells", 
            &clk_cells, sizeof(clk_cells)) < 0) {
            printk("%s: not found \"#clock-cells\" property!\n", __func__);
            errno = -ENOSTR;
            return NULL;
        }
        if (name) {
            ofw_property_read_string_index(np, "clock-names", i, 
                &output, buffer, sizeof(buffer));
            if (!strcmp(output, name))
                goto _next;
            if (i == count - 1) {
                errno = -ENODEV;
                return NULL;
            }
        } else {
            if (i == count - 1)
                break;
        }
        ofs += 1 + clk_cells;
    }

_next:
    dev = ofw_device_get_by_phandle(args[ofs]);
    if (dev == NULL) {
        printk("%s: not founc clock device (ofs: %d, args[ofs]: %d)\n", 
            __func__, ofs, args[ofs]);
        errno = -ENODEV;
        goto _out;
    }
    if (pcell && maxsize > 0) {
        bytes = clk_cells * sizeof(pcell_t);
        memcpy(pcell, &args[ofs+1], min_t(size_t, bytes, maxsize));
    }

_out:
    return dev;
}
