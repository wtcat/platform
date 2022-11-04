/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
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

#ifndef DEV_MMC_MMCBRVAR_H
#define	DEV_MMC_MMCBRVAR_H

#include <dev/mmc/mmcreg.h>

#include "mmc_bridge.h"

enum mmcbr_device_ivars {
    MMCBR_IVAR_BUS_MODE,
    MMCBR_IVAR_BUS_WIDTH,
    MMCBR_IVAR_CHIP_SELECT,
    MMCBR_IVAR_CLOCK,
    MMCBR_IVAR_F_MIN,
    MMCBR_IVAR_F_MAX,
    MMCBR_IVAR_HOST_OCR,
    MMCBR_IVAR_MODE,
    MMCBR_IVAR_OCR,
    MMCBR_IVAR_POWER_MODE,
    MMCBR_IVAR_RETUNE_REQ,
    MMCBR_IVAR_VDD,
    MMCBR_IVAR_VCCQ,
    MMCBR_IVAR_CAPS,
    MMCBR_IVAR_TIMING,
    MMCBR_IVAR_MAX_DATA,
    MMCBR_IVAR_MAX_BUSY_TIMEOUT
};

/*
 * Simplified accessors for bridge devices
 */
static inline void mmcbr_set_max_busy_timeout(struct mmc_host *host, u_int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_MAX_BUSY_TIMEOUT, &v);
}

static inline int mmcbr_get_max_busy_timeout(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_MAX_BUSY_TIMEOUT, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static inline void mmcbr_set_max_data(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_MAX_DATA, &v);
}

static inline int mmcbr_get_max_data(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_MAX_DATA, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static inline void mmcbr_set_timing(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_TIMING, &v);
}

static inline int mmcbr_get_timing(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_TIMING, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static inline void mmcbr_set_caps(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_CAPS, &v);
}

static inline int mmcbr_get_caps(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_CAPS, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static inline void mmcbr_set_vccq(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_VCCQ, &v);
}

static inline int mmcbr_get_vccq(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_VCCQ, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static inline void mmcbr_set_vdd(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_VDD, &v);
}

static inline int mmcbr_get_vdd(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_VDD, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static inline void mmcbr_set_power_mode(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_POWER_MODE, &v);
}

static inline int mmcbr_get_power_mode(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_POWER_MODE, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static inline void mmcbr_set_ocr(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_OCR, &v);
}

static inline int mmcbr_get_ocr(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_OCR, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static inline void mmcbr_set_mode(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_MODE, &v);
}

static inline int mmcbr_get_mode(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_MODE, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static inline void mmcbr_set_host_ocr(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_HOST_OCR, &v);
}

static inline int mmcbr_get_host_ocr(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_HOST_OCR, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static inline void mmcbr_set_f_min(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_F_MIN, &v);
}

static inline int mmcbr_get_f_min(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_F_MIN, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static inline void mmcbr_set_f_max(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_F_MAX, &v);
}

static inline int mmcbr_get_f_max(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_F_MAX, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static inline void mmcbr_set_clock(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_CLOCK, &v);
}

static inline int mmcbr_get_clock(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_CLOCK, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static inline void mmcbr_set_chip_select(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_CHIP_SELECT, &v);
}

static inline int mmcbr_get_chip_select(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_CHIP_SELECT, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}
								
static inline void mmcbr_set_bus_mode(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_BUS_MODE, &v);
}

static inline int mmcbr_get_bus_mode(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_BUS_MODE, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static inline void mmcbr_set_bus_width(struct mmc_host *host, int t) {
	uintptr_t v = (uintptr_t)t;
    host->ops->write_ivar(host, MMCBR_IVAR_BUS_WIDTH, &v);
}

static inline int mmcbr_get_bus_width(struct mmc_host *host) {
	uintptr_t v;
    int err = host->ops->read_ivar(host, MMCBR_IVAR_BUS_WIDTH, &v);
    BSD_ASSERT(err == 0);
    return (int)v;
}

static int inline mmcbr_get_retune_req(struct mmc_host *host) {
	uintptr_t v;
	if (__predict_false(host->ops->read_ivar(host,
	    MMCBR_IVAR_RETUNE_REQ, &v) != 0))
		return retune_req_none;
	return (int)v;
}

#endif /* DEV_MMC_MMCBRVAR_H */
