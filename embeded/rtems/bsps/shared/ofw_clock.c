/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <string.h>

#include "drivers/clock.h"
#include "drivers/ofw_platform_bus.h"
#include "ofw/ofw.h"


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
            errno = -ENOSTR;
            return NULL;
        }
        if (name) {
            ofw_property_read_string_index(np, "clock-names", i, 
                &output, buffer, sizeof(buffer));
            if (!strcmp(output, name))
                goto _next;
        }
        ofs += 1 + clk_cells;
    }

_next:
    dev = ofw_device_get_by_phandle(args[ofs]);
    if (dev == NULL) {
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
