/*-
 * Copyright (c) 2006 Bernd Walter.  All rights reserved.
 * Copyright (c) 2006 M. Warner Losh.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Portions of this software may have been developed with reference to
 * the SD Simplified Specification.  The following disclaimer may apply:
 *
 * The following conditions apply to the release of the simplified
 * specification ("Simplified Specification") by the SD Card Association and
 * the SD Group. The Simplified Specification is a subset of the complete SD
 * Specification which is owned by the SD Card Association and the SD
 * Group. This Simplified Specification is provided on a non-confidential
 * basis subject to the disclaimers below. Any implementation of the
 * Simplified Specification may require a license from the SD Card
 * Association, SD Group, SD-3C LLC or other third parties.
 *
 * Disclaimers:
 *
 * The information contained in the Simplified Specification is presented only
 * as a standard specification for SD Cards and SD Host/Ancillary products and
 * is provided "AS-IS" without any representations or warranties of any
 * kind. No responsibility is assumed by the SD Group, SD-3C LLC or the SD
 * Card Association for any damages, any infringements of patents or other
 * right of the SD Group, SD-3C LLC, the SD Card Association or any third
 * parties, which may result from its use. No license is granted by
 * implication, estoppel or otherwise under any patent or other rights of the
 * SD Group, SD-3C LLC, the SD Card Association or any third party. Nothing
 * herein shall be construed as an obligation by the SD Group, the SD-3C LLC
 * or the SD Card Association to disclose or distribute any technical
 * information, know-how or other confidential information to any third party.
 *
 * $FreeBSD$
 */

#ifndef DEV_MMC_PRIVATE_H
#define	DEV_MMC_PRIVATE_H

#include <rtems/thread.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

struct mmc_host;
struct mmc_softc;
struct mmc_request;

struct mmc_bus_ops {
	void (*retune_pause)(struct mmc_softc *sc, bool retune);
	void (*retune_unpause)(struct mmc_softc *sc);
	int (*wait_for_request)(struct mmc_softc *sc, struct mmc_request *req);
	int (*acquire_bus)(struct mmc_softc *sc);
	int (*release_bus)(struct mmc_softc *sc);
};

struct mmc_softc {
	struct mmc_host *host;
	const struct mmc_bus_ops *ops;
	rtems_recursive_mutex sc_mtx;
	struct intr_config_hook config_intrhook;
	device_t owner;
	device_t *child_list;
	int child_count;
	uint16_t last_rca;
	uint16_t retune_paused;
	uint8_t retune_needed;
	uint8_t retune_ongoing;
	uint16_t squelched;	/* suppress reporting of (expected) errors */
	int log_count;
	struct timeval log_time;
};


static inline void MMCBUS_RETUNE_PAUSE(struct mmc_softc *sc, bool retune) {
	sc->ops->retune_pause(sc, retune);
}

static inline void MMCBUS_RETUNE_UNPAUSE(struct mmc_softc *sc) {
	sc->ops->retune_unpause(sc);
}

static inline int MMCBUS_WAIT_FOR_REQUEST(struct mmc_softc *sc, struct mmc_request *req) {
	sc->ops->wait_for_request(sc, req);
}

static inline int MMCBUS_ACQUIRE_BUS(struct mmc_softc *sc) {
	sc->ops->acquire_bus(sc);
}

static inline int MMCBUS_RELEASE_BUS(struct mmc_softc *sc) {
	sc->ops->release_bus(sc);
}

#endif /* DEV_MMC_PRIVATE_H */
