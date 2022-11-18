/*
 * Copyright 2022 wtcat
 */
#include <machine/rtems-bsd-kernel-space.h>

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <rtems/bsd/local/opt_evdev.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/rman.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/sysctl.h>
#include <sys/selinfo.h>
#include <sys/poll.h>
#include <sys/uio.h>
#include <sys/conf.h>
#include <sys/taskqueue.h>

#include <dev/evdev/input.h>
#include <dev/evdev/evdev.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>
#include <bsp.h>

#include "stm32/stm32_com.h"
#include "drivers/gpio.h"


#define GPIO_DEBOUNCE_TIME RTEMS_MILLISECONDS_TO_TICKS(10)

struct gpio_keys_softc;

struct gpio_button {
    rtems_id timer;
    struct gpio_keys_softc *sc;
    struct gpio_pin *btn;
    uint16_t code;
};

struct gpio_keys_softc {
    device_t dev;
    struct mtx mtx;
    struct evdev_dev *evdev;
    size_t nkey;
    struct gpio_button *keys;
};

static void __isr gpio_keys_tmr(rtems_id timer, void *arg) {
    struct gpio_button *btn = (struct gpio_button *)arg;
    struct evdev_dev *evdev = btn->sc->evdev;

    if (gpiod_is_active(btn->btn))
        evdev_push_event(evdev, EV_KEY, btn->code, 1);
    else
        evdev_push_event(evdev, EV_KEY, btn->code, 0);
    evdev_sync(evdev);
    (void) timer;
}

static void __isr gpio_keys_isr(void *arg) {
    struct gpio_button *btn = arg;
    rtems_timer_server_fire_after(btn->timer, GPIO_DEBOUNCE_TIME, 
        gpio_keys_tmr, btn);
}

static int gpio_keys_ev_open(struct evdev_dev *evdev) {
	struct gpio_keys_softc *sc = evdev_get_softc(evdev);
    struct gpio_button *btn;
    size_t i, j;
    int err;
    (void) evdev;

    for (i = 0; i < sc->nkey; i++) {
        btn = sc->keys + i;
        err = gpiod_pin_irq_request(btn->btn, "gpio-pin", gpio_keys_isr, btn);
        if (err) {
            device_printf(sc->dev, "%s: gpio-pin request irq failed!(%d)\n", __func__, err);
            goto _free;
        }
        rtems_timer_create(rtems_build_name('k','e','y','s'+i), &btn->timer);
    }
	return 0;

_free:
    for (j = 0; j < i; j++) {
        btn = sc->keys + j;
        rtems_timer_delete(btn->timer);
        gpiod_pin_irq_remove(btn->btn, gpio_keys_isr, btn);
    }
    return -ENXIO;
}

static int gpio_keys_ev_close(struct evdev_dev *evdev) {
	struct gpio_keys_softc *sc = evdev_get_softc(evdev);
    (void) evdev;

    for (size_t i = 0; i < sc->nkey; i++) {
        struct gpio_button *btn = sc->keys + i;
        rtems_timer_delete(btn->timer);
        gpiod_pin_irq_remove(btn->btn, gpio_keys_isr, btn);
    }
	return 0;
}

static const struct evdev_methods evdev_methods = {
	.ev_open = gpio_keys_ev_open,
	.ev_close = gpio_keys_ev_close,
};

static const struct ofw_compat_data compat_data[] = {
	{"gpio-keys",	1},
	{NULL,		 	0},
};

static int gpio_keys_probe(device_t dev) {
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "GPIO-KEYS driver\n");
		return (0);
	}
    return ENXIO;
}

static int gpio_parse_dts(device_t dev) {
    struct gpio_keys_softc *sc = device_get_softc(dev);
    struct gpio_button *buttons;
    phandle_t parent, child;
    phandle_t prop;
    int nr = 0;
    int err;
    
    parent = ofw_bus_get_node(dev);
    for (child = OF_child(parent); child > 0; child = OF_peer(child)) {
        if (OF_hasprop(child, "gpios"))
		    nr++;
    }
    buttons = malloc(sizeof(*buttons) * nr, M_DEVBUF, M_ZERO|M_WAITOK);
    if (!buttons)
        return ENOMEM;

    for (child = OF_child(parent), nr = 0; child > 0; child = OF_peer(child)) {
        if (!OF_hasprop(child, "gpios"))
            continue;

        if (rtems_ofw_get_enc_prop(child, "rtems,code", &prop, sizeof(prop)) < 0) {
            device_printf(sc->dev, "%s: not found rtems,code\n", __func__);
            goto _freem;
        }

        buttons[nr].btn = ofw_gpios_request(child, GPIO_INTR(GPIO_EDGE_BOTH), NULL);
        if (!buttons[nr].btn) {
            device_printf(sc->dev, "%s: request gpios failed!\n", __func__);
            goto _freem;
        }

        buttons[nr].code = (uint16_t)prop;
        buttons[nr].sc = sc;
        nr++;
    }

    sc->nkey = nr;
    sc->keys = buttons;
    return 0;

_freem:
    free(buttons, M_DEVBUF);
    return ENXIO;
}

static int gpio_keys_attach(device_t dev) {
    struct gpio_keys_softc *sc = device_get_softc(dev);
    int err;

    if (gpio_parse_dts(dev)) {
        device_printf(dev, "gpio keys parse failed\n");
        return ENXIO;
    }

    sc->dev = dev;
    sc->evdev = evdev_alloc();
    mtx_init(&sc->mtx, device_get_nameunit(sc->dev), 
        "gpio-keys", MTX_DEF);
	evdev_set_name(sc->evdev, device_get_desc(sc->dev));
	evdev_set_phys(sc->evdev, device_get_nameunit(sc->dev));
	evdev_set_id(sc->evdev, BUS_HOST, 0, 0, 0);
	evdev_set_methods(sc->evdev, sc, &evdev_methods);
	evdev_support_prop(sc->evdev, INPUT_PROP_DIRECT);
	evdev_support_event(sc->evdev, EV_SYN);
	evdev_support_event(sc->evdev, EV_KEY);
    evdev_support_key(sc->evdev, KEY_9);
	err = evdev_register_mtx(sc->evdev, &sc->mtx);
	if (err) {
		evdev_free(sc->evdev);
		sc->evdev = NULL;
	}

	return 0;
}

static int gpio_keys_detach(device_t dev) {
	struct gpio_keys_softc *sc = device_get_softc(dev);
    free(sc->keys, M_DEVBUF);
	evdev_free(sc->evdev);
    mtx_destroy(&sc->mtx);
    return 0;
}

static device_method_t gpio_keys_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe,   gpio_keys_probe),
    DEVMETHOD(device_attach,  gpio_keys_attach),
    DEVMETHOD(device_detach,  gpio_keys_detach),
    DEVMETHOD_END
};

static driver_t gpio_keys_driver = {
    .name = "gpio-keys",
    .methods = gpio_keys_methods,
    .size = sizeof(struct gpio_keys_softc)
};

static devclass_t gpio_keys_devclass;

DRIVER_MODULE(gpio_keys, simplebus, gpio_keys_driver, gpio_keys_devclass, 0, 0);
MODULE_DEPEND(gpio_keys, evdev, 1, 1, 1);