#include <machine/rtems-bsd-kernel-space.h>

/* SPDX-License-Identifier: BSD-2-Clause */

/*
 * Copyright (C) 2021 embedded brains GmbH (http://www.embedded-brains.de)
 * Copyright (C) 2022 wtcat
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * STM32H7xx SDMMC controller. Documentation: ST RM0433 (Rev 6), Chapter 54
 *
 * According to Linux DTS, the SDMMC is compatible with an ARM Primecell PL18X
 * with peripheral ID 0x10153180.
 */
/*
 * Note: This driver is inspired by the NetBSD pl181 driver written by Jared D.
 * McNeill.
 */
/*
 * Note regarding DMA on STM32H7:
 *
 * The STM32H7 SDMMC doesn't have an interrupt for few received data (less than
 * half the FIFO size). So especially for short responses, it is not possible to
 * use the SDMMC without DMA.
 *
 * On the STM32H7 SDMMC there are two DMAs: One IDMA integrated into the SDMMC
 * and one MDMA that is a general purpose DMA. MDMA can only be used on first
 * instance of the SDMMC. For the second instance, the trigger signals are not
 * connected.
 *
 * The IDMA of SDMMC1 can only access AXI SRAM, QSPI and FMC (where SDRAM is
 * located). The IDMA of SDMMC2 could access memory in other domains too.
 *
 * MDMA is designed to be a companion for IDMA. It seems that ST thought of a
 * very specific software structure for that. It can be either used to change
 * the IDMA buffer addresses (which would allow some kind of scatter gather
 * functionality with fixed buffer sizes) or to refill IDMA buffers from some
 * RAM that can't be accessed directly by the IDMA. Take a look at ST AN5200 Rev
 * 1 "Getting started with STM32H7 Series SDMMC host controller" for more
 * details.
 */

#include <rtems/malloc.h>
#include <rtems/irq-extension.h>
#include <rtems/bsd/bsd.h>

#include <bsp.h>

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/condvar.h>

#include <pthread.h>

#include <machine/resource.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcbrvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#undef free

#include "base/sections.h"
#undef log
#include "stm32/stm32_com.h"
#include "stm32h7xx.h"
#include "stm32h7xx_ll_rcc.h"


struct st_sdmmc_softc {
	device_t dev;
	struct mmc_host host;
	struct mtx mtx;
	rtems_binary_semaphore completed;
	int bus_busy;
	struct resource *mem_res;
	struct resource *irq_res;
	SDMMC_TypeDef *sdmmc;
	DLYB_TypeDef *dlyb;
	rtems_vector_number irq;

	uint32_t sdmmc_ker_ck;
	struct st_sdmmc_config cfg;

	uint32_t intr_status;
	uint8_t *dmabuf;
};


#define	ST_SDMMC_LOCK(_sc)		mtx_lock(&(_sc)->mtx)
#define	ST_SDMMC_UNLOCK(_sc)	mtx_unlock(&(_sc)->mtx)
#define	ST_SDMMC_LOCK_INIT(_sc) \
	mtx_init(&_sc->mtx, device_get_nameunit(_sc->dev), \
	    "st_sdmmc", MTX_DEF)

#define ST_SDMMC_ERRORS \
    (SDMMC_STA_IDMATE | SDMMC_STA_ACKTIMEOUT | \
     SDMMC_STA_RXOVERR | SDMMC_STA_TXUNDERR | \
     SDMMC_STA_DTIMEOUT | SDMMC_STA_CTIMEOUT | \
     SDMMC_STA_DCRCFAIL | SDMMC_STA_CCRCFAIL)

#define ST_SDMMC_MASKR_ALL \
    (SDMMC_MASK_CCRCFAILIE | SDMMC_MASK_DCRCFAILIE | SDMMC_MASK_CTIMEOUTIE | \
     SDMMC_MASK_TXUNDERRIE | SDMMC_MASK_RXOVERRIE | SDMMC_MASK_CMDRENDIE | \
     SDMMC_MASK_CMDSENTIE | SDMMC_MASK_DATAENDIE | SDMMC_MASK_ACKTIMEOUTIE)


static const struct ofw_compat_data compat_data[] = {
	{"arm,primecell",	1},
	{NULL,		 	0},
};


static void __fastcode st_sdmmc_intr(void *arg) {
	struct st_sdmmc_softc *sc = arg;
    SDMMC_TypeDef *sdmmc = sc->sdmmc;
	uint32_t status = sdmmc->STA;;

	status = sdmmc->STA;
	sdmmc->ICR = status;
	sc->intr_status = status;
	/*
	 * There seems to be some odd combination where the status is zero but
	 * an interrupt occurred. In that case, the task shouldn't wake up.
	 * Therefore check for status != 0.
	 */
	rtems_binary_semaphore_post(&sc->wait_done);
}

static int st_sdmmc_probe(device_t dev) {
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_has_prop(dev, "non-removable"))
		return ENXIO;
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "STM32H7xx SDMMC Host");
		return (0);
	}
	return ENXIO;
}

static int st_sdmmc_set_clock_and_bus(struct st_sdmmc_softc *sc, 
    uint32_t freq, enum mmc_bus_width width) {
	uint32_t clk_div;
	uint32_t clkcr;

	if (freq == 0)
		return 0;

	clkcr = SDMMC_CLKCR_NEGEDGE | SDMMC_CLKCR_PWRSAV | SDMMC_CLKCR_HWFC_EN;
	clk_div = howmany(sc->sdmmc_ker_ck, freq) / 2;
	if (clk_div > SDMMC_CLKCR_CLKDIV >> SDMMC_CLKCR_CLKDIV_Pos) 
		clk_div = SDMMC_CLKCR_CLKDIV >> SDMMC_CLKCR_CLKDIV_Pos;
	
	clkcr |= clk_div << SDMMC_CLKCR_CLKDIV_Pos;
	switch (width) {
	default:
		BSD_ASSERT(width == bus_width_1);
		clkcr |= 0 << SDMMC_CLKCR_WIDBUS_Pos;
		break;
	case bus_width_4:
		clkcr |= 1 << SDMMC_CLKCR_WIDBUS_Pos;
		break;
	case bus_width_8:
		clkcr |= 2 << SDMMC_CLKCR_WIDBUS_Pos;
		break;
	}

	sc->sdmmc->CLKCR = clkcr;
	return 0;
}

static int st_sdmmc_update_ios(device_t brdev, device_t reqdev) {
	struct st_sdmmc_softc *sc;
	struct mmc_ios *ios;
	int err;

	(void) reqdev;
	sc = device_get_softc(brdev);
	ST_SDMMC_LOCK(sc);
	ios = &sc->host.ios;
	err = st_sdmmc_set_clock_and_bus(sc, ios->clock, ios->bus_width);
	if (err) 
		return err;
	if (ios->power_mode == power_off) {
		/*
		 * FIXME: Maybe a reset of the module is necessary instead. But
		 * the ST samples use a power off too so it should work. But
		 * power saving hasn't been tested during development.
		 */
		sc->sdmmc->POWER &= ~(SDMMC_POWER_PWRCTRL);
	} else {
		sc->sdmmc->POWER |= SDMMC_POWER_PWRCTRL;
	}

	ST_SDMMC_UNLOCK(sc);
	return EIO;
}

static void st_sdmmc_send_cmd(struct st_sdmmc_softc *sc, struct mmc_command *cmd) {
    SDMMC_TypeDef *sdmmc = sc->sdmmc;
    uint32_t reg_cmd;

    rtems_binary_semaphore_init(&sc->completed, "sdmmc");

    sdmmc->MASK |= SDIO_MASKR_ALL;
    reg_cmd = (cmd->opcode & SDMMC_CMD_CMDINDEX_Msk) | SDMMC_CMD_CPSMEN;
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			cmdval |= SDMMC_CMD_WAITRESP_0 | SDMMC_CMD_WAITRESP_1;
		} else if (cmd->flags & MMC_RSP_CRC) 
			cmdval |= SDMMC_CMD_WAITRESP_0;
		else
			cmdval |= SDMMC_CMD_WAITRESP_1;
	} else
		int_mask |= SDMMC_INT_CMD_DONE_MASK;

	if (cmd->opcode == MMC_STOP_TRANSMISSION)
		cmdval |= SDMMC_CMD_CMDSTOP;
	if ((cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC)
		cmdval |= SDMMC_CMD_CMDTRANS;
	

	if ((cmd->flags & MMC_RSP_BUSY) != 0) 
		int_mask |= SDMMC_MASK_BUSYD0ENDIE;
	}

}

static int st_sdmmc_request(device_t brdev, device_t reqdev, struct mmc_request *req) {
	struct st_sdmmc_softc *sc = device_get_softc(brdev);
	(void) reqdev;

	ST_SDMMC_LOCK(sc);
	st_sdmmc_cmd_do(sc, req->cmd);
	if (req->stop) 
		st_sdmmc_cmd_do(sc, req->stop);
	ST_SDMMC_UNLOCK(sc);
	(*req->done)(req);
	return 0;
}

static int st_sdmmc_get_ro(device_t brdev, device_t reqdev) {
	(void) brdev;
	(void) reqdev;
	/*
	 * FIXME: Currently just ignore write protection. Micro-SD doesn't have
	 * it anyway and most boards are now using Micro-SD slots.
	 */
	return 0;
}

static int st_sdmmc_acquire_host(device_t brdev, device_t reqdev) {
	struct st_sdmmc_softc *sc;
	(void) reqdev;
	sc = device_get_softc(brdev);

	ST_SDMMC_LOCK(sc);
	while (sc->bus_busy)
		msleep(sc, &sc->mtx, PZERO, "stsdmmcah", hz / 5);
	sc->bus_busy++;
	ST_SDMMC_UNLOCK(sc);
	return 0;
}

static int st_sdmmc_release_host(device_t brdev, device_t reqdev) {
	struct st_sdmmc_softc *sc;
	(void) reqdev;
	sc = device_get_softc(brdev);

	ST_SDMMC_LOCK(sc);
	sc->bus_busy--;
	wakeup(sc);
	ST_SDMMC_UNLOCK(sc);
	return (0);
}

static int st_sdmmc_read_ivar(device_t bus, device_t child, 
    int which, uintptr_t *result) {
	struct st_sdmmc_softc *sc;
	(void) child;
	sc = device_get_softc(bus);

	switch (which) {
	default:
		return (EINVAL);
	case MMCBR_IVAR_BUS_MODE:
		*(int *)result = sc->host.ios.bus_mode;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		*(int *)result = sc->host.ios.bus_width;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		*(int *)result = sc->host.ios.chip_select;
		break;
	case MMCBR_IVAR_CLOCK:
		*(int *)result = sc->host.ios.clock;
		break;
	case MMCBR_IVAR_F_MIN:
		*(int *)result = sc->host.f_min;
		break;
	case MMCBR_IVAR_F_MAX:
		*(int *)result = sc->host.f_max;
		break;
	case MMCBR_IVAR_HOST_OCR:
		*(int *)result = sc->host.host_ocr;
		break;
	case MMCBR_IVAR_MODE:
		*(int *)result = sc->host.mode;
		break;
	case MMCBR_IVAR_OCR:
		*(int *)result = sc->host.ocr;
		break;
	case MMCBR_IVAR_POWER_MODE:
		*(int *)result = sc->host.ios.power_mode;
		break;
	case MMCBR_IVAR_VDD:
		*(int *)result = sc->host.ios.vdd;
		break;
	case MMCBR_IVAR_VCCQ:
		*(int *)result = sc->host.ios.vccq;
		break;
	case MMCBR_IVAR_CAPS:
		*(int *)result = sc->host.caps;
		break;
	case MMCBR_IVAR_MAX_DATA:
		/*
		 * Note: At the moment of writing this, RTEMS ignores this
		 * value. So it's quite irrelevant what is returned here.
		 */
		*(int *)result = (SDMMC_DLEN_DATALENGTH_Msk) / MMC_SECTOR_SIZE;
		break;
	case MMCBR_IVAR_TIMING:
		*(int *)result = sc->host.ios.timing;
		break;
	}
	return (0);
}

static int st_sdmmc_write_ivar(device_t bus, device_t child, 
    int which, uintptr_t value) {
	struct st_sdmmc_softc *sc;
	(void) child;
	sc = device_get_softc(bus);

	switch (which) {
	default:
		return (EINVAL);
	case MMCBR_IVAR_BUS_MODE:
		sc->host.ios.bus_mode = value;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		sc->host.ios.bus_width = value;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		sc->host.ios.chip_select = value;
		break;
	case MMCBR_IVAR_CLOCK:
		sc->host.ios.clock = value;
		break;
	case MMCBR_IVAR_MODE:
		sc->host.mode = value;
		break;
	case MMCBR_IVAR_OCR:
		sc->host.ocr = value;
		break;
	case MMCBR_IVAR_POWER_MODE:
		sc->host.ios.power_mode = value;
		break;
	case MMCBR_IVAR_VDD:
		sc->host.ios.vdd = value;
		break;
	case MMCBR_IVAR_TIMING:
		sc->host.ios.timing = value;
		break;
	case MMCBR_IVAR_VCCQ:
		sc->host.ios.vccq = value;
		break;
	/* These are read-only */
	case MMCBR_IVAR_CAPS:
	case MMCBR_IVAR_HOST_OCR:
	case MMCBR_IVAR_F_MIN:
	case MMCBR_IVAR_F_MAX:
	case MMCBR_IVAR_MAX_DATA:
		return EINVAL;
	}
	return 0;
}

static device_method_t st_sdmmc_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe, st_sdmmc_probe),
	DEVMETHOD(device_attach, st_sdmmc_attach),
	DEVMETHOD(device_detach, st_sdmmc_detach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar, st_sdmmc_read_ivar),
	DEVMETHOD(bus_write_ivar, st_sdmmc_write_ivar),

	/* mmcbr_if */
	DEVMETHOD(mmcbr_update_ios, st_sdmmc_update_ios),
	DEVMETHOD(mmcbr_request, st_sdmmc_request),
	DEVMETHOD(mmcbr_get_ro, st_sdmmc_get_ro),
	DEVMETHOD(mmcbr_acquire_host, st_sdmmc_acquire_host),
	DEVMETHOD(mmcbr_release_host, st_sdmmc_release_host),

	DEVMETHOD_END
};

static driver_t st_sdmmc_driver = {
	"st_sdmmc",
	st_sdmmc_methods,
	sizeof(struct st_sdmmc_softc)
};

static devclass_t st_sdmmc_devclass;

DRIVER_MODULE(st_sdmmc, simplebus, st_sdmmc_driver, st_sdmmc_devclass, NULL, NULL);
DRIVER_MODULE(mmc, st_sdmmc, mmc_driver, mmc_devclass, NULL, NULL);
MODULE_DEPEND(st_sdmmc, mmc, 1, 1, 1);
SYSINIT_DRIVER_REFERENCE(mmcsd, mmc);