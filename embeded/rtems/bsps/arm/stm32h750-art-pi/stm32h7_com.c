/*
 * Copyright 2022 wtcat
 */
#include <errno.h>
#include <sys/errno.h>

#include "drivers/ofw_platform_bus.h"
#include "drivers/pinctrl.h"
#include "drivers/clock.h"



int stm32_ofw_get_clkdev(phandle_t np, struct drvmgr_dev **clkdev, int *clkid) {
    pcell_t clks[2];
    if (clkdev == NULL || clkid == NULL)
        return -EINVAL;
    if (rtems_ofw_get_enc_prop(np, "clocks", clks, sizeof(clks)) < 0)
        return -ENODATA;
    *clkdev = ofw_device_get_by_phandle(clks[0]);
    if (*clkdev == NULL)
        return -ENODATA;
    *clkid = clks[1];
    return 0;
}

int stm32_pinctrl_set(struct drvmgr_dev *dev) {
    static struct drvmgr_dev *pinctrl;
    if (dev == NULL)
        return -EINVAL;
    if (pinctrl == NULL) {
        pinctrl = drvmgr_dev_by_name("/dev/pinctrl");
        if (pinctrl == NULL)
            return -ENODEV;
    }
     return pinctrl_generic_set_state(pinctrl, dev);
}

int stm32_pinctrl_set_np(phandle_t np) {
    return pinctrl_simple_set_np("/dev/pinctrl", np);
}

int stm32_clk_enable(phandle_t np, int index) {
    struct drvmgr_dev *clk;
    pcell_t clkid;

    clk = ofw_clock_request(np, NULL, (pcell_t *)&clkid, 
    sizeof(clkid));
    if (clk)
        return clk_enable(clk, &clkid);
    (void) index;
    return -ENODEV;
}
