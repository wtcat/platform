/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2006 Bernd Walter.  All rights reserved.
 * Copyright (c) 2006 M. Warner Losh.
 * Copyright (c) 2017 Marius Strobl <marius@FreeBSD.org>
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/endian.h>
#include <sys/fcntl.h>
#include <sys/ioccom.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/priv.h>
#include <sys/slicer.h>
#include <sys/sysctl.h>
#include <sys/time.h>

#include <geom/geom.h>
#include <geom/geom_disk.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmc_ioctl.h>
#include <dev/mmc/mmc_subr.h>
#include <dev/mmc/mmcbrvar.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcvar.h>

#include <rtems/bsd/local/mmcbus_if.h>
#ifdef __rtems__
#include <machine/rtems-bsd-support.h>
#include <rtems/bdbuf.h>
#include <rtems/diskdevs.h>
#include <rtems/libio.h>
#include <rtems/media.h>
#endif /* __rtems__ */

#if __FreeBSD_version < 800002
#define	kproc_create	kthread_create
#define	kproc_exit	kthread_exit
#endif

#define	MMCSD_CMD_RETRIES	5

#define	MMCSD_FMT_BOOT		"mmcsd%dboot"
#define	MMCSD_FMT_GP		"mmcsd%dgp"
#define	MMCSD_FMT_RPMB		"mmcsd%drpmb"
#define	MMCSD_LABEL_ENH		"enh"

#define	MMCSD_PART_NAMELEN	(16 + 1)

struct mmcsd_softc;

struct mmcsd_part {
	struct mtx disk_mtx;
	struct mtx ioctl_mtx;
	struct mmcsd_softc *sc;

	u_int cnt;
	u_int type;
	int running;
	int suspend;
	int ioctl;
	bool ro;
	char name[MMCSD_PART_NAMELEN];
};

struct mmcsd_softc {
	struct mmc_softc *mmc;
	device_t dev;
	device_t mmcbus;
	struct mmcsd_part *part[MMC_PART_MAX];
	enum mmc_card_mode mode;
	u_int max_data;		/* Maximum data size [blocks] */
	u_int erase_sector;	/* Device native erase sector size [blocks] */
	uint8_t	high_cap;	/* High Capacity device (block addressed) */
	uint8_t part_curr;	/* Partition currently switched to */
	uint8_t ext_csd[MMC_EXTCSD_SIZE];
	uint16_t rca;
	uint32_t flags;
#define	MMCSD_INAND_CMD38	0x0001
#define	MMCSD_USE_TRIM		0x0002
#define	MMCSD_FLUSH_CACHE	0x0004
#define	MMCSD_DIRTY		0x0008
	uint32_t cmd6_time;	/* Generic switch timeout [us] */
	uint32_t part_time;	/* Partition switch timeout [us] */
	off_t enh_base;		/* Enhanced user data area slice base ... */
	off_t enh_size;		/* ... and size [bytes] */
	int log_count;
	struct timeval log_time;
	struct cdev *rpmb_dev;
};


static const char * const errmsg[] =
{
	"None",
	"Timeout",
	"Bad CRC",
	"Fifo",
	"Failed",
	"Invalid",
	"NO MEMORY"
};

static SYSCTL_NODE(_hw, OID_AUTO, mmcsd, CTLFLAG_RD, NULL, "mmcsd driver");

static int mmcsd_cache = 1;
SYSCTL_INT(_hw_mmcsd, OID_AUTO, cache, CTLFLAG_RDTUN, &mmcsd_cache, 0,
    "Device R/W cache enabled if present");

#define	LOG_PPS		5 /* Log no more than 5 errors per second. */

/* bus entry points */
static int mmcsd_attach(device_t dev);
static int mmcsd_detach(device_t dev);
static int mmcsd_probe(device_t dev);
static int mmcsd_shutdown(device_t dev);


/* RMPB cdev interface */
static int mmcsd_ioctl_rpmb(struct cdev *dev, u_long cmd, caddr_t data,
    int fflag, struct thread *td);

static void mmcsd_add_part(struct mmcsd_softc *sc, u_int type,
    const char *name, u_int cnt, off_t media_size, bool ro);
static int mmcsd_bus_bit_width(device_t dev);

static const char *mmcsd_errmsg(int e);

static const char *mmcsd_errmsg(int e);
static int mmcsd_ioctl(struct mmcsd_part *part, u_long cmd, void *data,
    int fflag, struct thread *td);
static int mmcsd_ioctl_cmd(struct mmcsd_part *part, struct mmc_ioc_cmd *mic,
    int fflag);
static uintmax_t mmcsd_pretty_size(off_t size, char *unit);

static int mmcsd_set_blockcount(struct mmcsd_softc *sc, u_int count, bool rel);

static int mmcsd_switch_part(device_t bus, device_t dev, uint16_t rca,
    u_int part);

#define	MMCSD_DISK_LOCK(_part)		rtems_recursive_mutex_lock(&(_part)->disk_mtx)
#define	MMCSD_DISK_UNLOCK(_part)	rtems_recursive_mutex_unlock(&(_part)->disk_mtx)
#define	MMCSD_DISK_LOCK_INIT(_part)	rtems_recursive_mutex_init(&(_part)->disk_mtx, "mmcsd disk")
#define	MMCSD_DISK_LOCK_DESTROY(_part) rtems_recursive_mutex_destroy(&(_part)->disk_mtx);

#define	MMCSD_IOCTL_LOCK(_part)		rtems_recursive_mutex_lock(&(_part)->ioctl_mtx)
#define	MMCSD_IOCTL_UNLOCK(_part)	rtems_recursive_mutex_unlock(&(_part)->ioctl_mtx)
#define	MMCSD_IOCTL_LOCK_INIT(_part) rtems_recursive_mutex_init(&(_part)->ioctl_mtx, "mmcsd IOCTL")
#define	MMCSD_IOCTL_LOCK_DESTROY(_part)	rtems_recursive_mutex_destroy(&(_part)->ioctl_mtx);


static int
mmcsd_probe(device_t dev)
{
	device_quiet(dev);
	device_set_desc(dev, "MMC/SD Memory Card");
	return (0);
}

#ifdef __rtems__
static rtems_status_code
mmcsd_set_block_size(struct mmc_softc *mmc, uint32_t block_size)
{
	rtems_status_code status_code = RTEMS_SUCCESSFUL;
	struct mmc_command cmd;
	struct mmc_request req;

	memset(&req, 0, sizeof(req));
	memset(&cmd, 0, sizeof(cmd));

	req.cmd = &cmd;
	cmd.opcode = MMC_SET_BLOCKLEN;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
	cmd.arg = block_size;
	mmcbus_wait_for_request(mmc, &req);
	if (req.cmd->error != MMC_ERR_NONE) {
		status_code = RTEMS_IO_ERROR;
	}

	return status_code;
}

static int
mmcsd_disk_read_write(struct mmcsd_part *part, rtems_blkdev_request *blkreq)
{
	rtems_status_code status_code = RTEMS_SUCCESSFUL;
	struct mmcsd_softc *sc = part->sc;
	device_t dev = sc->dev;
	int shift = mmc_get_high_cap(dev) ? 0 : 9;
	int rca = mmc_get_rca(dev);
	uint32_t buffer_count = blkreq->bufnum;
	uint32_t transfer_bytes = blkreq->bufs[0].length;
	uint32_t block_count = transfer_bytes / MMC_SECTOR_SIZE;
	uint32_t opcode;
	uint32_t data_flags;
	uint32_t i;

	if (blkreq->req == RTEMS_BLKDEV_REQ_WRITE) {
		if (block_count > 1) {
			opcode = MMC_WRITE_MULTIPLE_BLOCK;
		} else {
			opcode = MMC_WRITE_BLOCK;
		}

		data_flags = MMC_DATA_WRITE;
	} else {
		BSD_ASSERT(blkreq->req == RTEMS_BLKDEV_REQ_READ);

		if (block_count > 1) {
			opcode = MMC_READ_MULTIPLE_BLOCK;
		} else {
			opcode = MMC_READ_SINGLE_BLOCK;
		}

		data_flags = MMC_DATA_READ;
	}

	MMCSD_DISK_LOCK(part);

	for (i = 0; i < buffer_count; ++i) {
		rtems_blkdev_sg_buffer *sg = &blkreq->bufs [i];
		struct mmc_request req;
		struct mmc_command cmd;
		struct mmc_command stop;
		struct mmc_data data;
		rtems_interval timeout;

		memset(&req, 0, sizeof(req));
		memset(&cmd, 0, sizeof(cmd));
		memset(&stop, 0, sizeof(stop));

		req.cmd = &cmd;

		cmd.opcode = opcode;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
		cmd.data = &data;
		cmd.arg = sg->block << shift;

		if (block_count > 1) {
			data_flags |= MMC_DATA_MULTI;
			stop.opcode = MMC_STOP_TRANSMISSION;
			stop.flags = MMC_RSP_R1B | MMC_CMD_AC;
			req.stop = &stop;
		}

		data.flags = data_flags;;
		data.data = sg->buffer;
		data.mrq = &req;
		data.len = transfer_bytes;

		mmcbus_wait_for_request(sc->mmc, &req);
		if (req.cmd->error != MMC_ERR_NONE) {
			status_code = RTEMS_IO_ERROR;
			goto error;
		}

		timeout = rtems_clock_tick_later_usec(250000);
		while (1) {
			struct mmc_request req2;
			struct mmc_command cmd2;
			uint32_t status;

			memset(&req2, 0, sizeof(req2));
			memset(&cmd2, 0, sizeof(cmd2));
			req2.cmd = &cmd2;
			cmd2.opcode = MMC_SEND_STATUS;
			cmd2.arg = rca << 16;
			cmd2.flags = MMC_RSP_R1 | MMC_CMD_AC;

			mmcbus_wait_for_request(sc->mmc, &req2);
			if (req2.cmd->error != MMC_ERR_NONE) {
				status_code = RTEMS_IO_ERROR;
				goto error;
			}

			status = cmd2.resp[0];
			if ((status & R1_READY_FOR_DATA) != 0
			    && R1_CURRENT_STATE(status) != R1_STATE_PRG) {
				break;
			}
			if (!rtems_clock_tick_before(timeout)) {
				status_code = RTEMS_IO_ERROR;
				goto error;
			}
		}
	}

error:
	MMCSD_DISK_UNLOCK(part);
	rtems_blkdev_request_done(blkreq, status_code);
	return 0;
}

static int
mmcsd_disk_ioctl(rtems_disk_device *dd, uint32_t req, void *arg)
{

	if (req == RTEMS_BLKIO_REQUEST) {
		struct mmcsd_part *part = rtems_disk_get_driver_data(dd);
		rtems_blkdev_request *blkreq = arg;

		return mmcsd_disk_read_write(part, blkreq);
	} else if (req == RTEMS_BLKIO_CAPABILITIES) {
		*(uint32_t *) arg = RTEMS_BLKDEV_CAP_MULTISECTOR_CONT;
		return 0;
	} else {
		return rtems_blkdev_ioctl(dd, req, arg);
	}
}

static rtems_status_code
mmcsd_attach_worker(rtems_media_state state, const char *src, char **dest, void *arg)
{
	rtems_status_code status_code = RTEMS_SUCCESSFUL;
	struct mmcsd_part *part = arg;
	char *disk = NULL;

	if (state == RTEMS_MEDIA_STATE_READY) {
		struct mmcsd_softc *sc = part->sc;
		device_t dev = sc->dev;
		uint32_t block_count = mmc_get_media_size(dev);
		uint32_t block_size = MMC_SECTOR_SIZE;

		disk = rtems_media_create_path("/dev", src, device_get_unit(dev));
		if (disk == NULL) {
			printf("OOPS: create path failed\n");
			goto error;
		}

		/*
		 * FIXME: There is no release for this acquire. Implementing
		 * this would be necessary for:
		 * - multiple hardware partitions of eMMC chips
		 * - multiple devices on one bus
		 *
		 * On the other hand it would mean that the bus has to be
		 * acquired on every read which would decrease the performance.
		 */
		MMCBUS_ACQUIRE_BUS(device_get_parent(dev), dev);

		status_code = mmcsd_set_block_size(sc->mmc, block_size);
		if (status_code != RTEMS_SUCCESSFUL) {
			printf("OOPS: set block size failed\n");
			goto error;
		}

		status_code = rtems_blkdev_create(disk, block_size,
		    block_count, mmcsd_disk_ioctl, part);
		if (status_code != RTEMS_SUCCESSFUL) {
			goto error;
		}

		*dest = strdup(disk, M_RTEMS_HEAP);
	}

	return RTEMS_SUCCESSFUL;

error:
	free(disk, M_RTEMS_HEAP);

	return RTEMS_IO_ERROR;
}
#endif /* __rtems__ */
static int
mmcsd_attach(device_t dev)
{
	device_t mmcbus;
	struct mmcsd_softc *sc;
	const uint8_t *ext_csd;
	off_t erase_size, sector_size, size, wp_size;
	uintmax_t bytes;
	int err, i;
	uint32_t quirks;
	uint8_t rev;
	bool comp, ro;
	char unit[2];

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->mmcbus = mmcbus = device_get_parent(dev);
	sc->mode = mmc_get_card_type(dev);
	/*
	 * Note that in principle with an SDHCI-like re-tuning implementation,
	 * the maximum data size can change at runtime due to a device removal/
	 * insertion that results in switches to/from a transfer mode involving
	 * re-tuning, iff there are multiple devices on a given bus.  Until now
	 * mmc(4) lacks support for rescanning already attached buses, however,
	 * and sdhci(4) to date has no support for shared buses in the first
	 * place either.
	 */
	sc->max_data = mmc_get_max_data(dev);
	sc->high_cap = mmc_get_high_cap(dev);
	sc->rca = mmc_get_rca(dev);
	sc->cmd6_time = mmc_get_cmd6_timeout(dev);
	quirks = mmc_get_quirks(dev);

	/* Only MMC >= 4.x devices support EXT_CSD. */
	if (mmc_get_spec_vers(dev) >= 4) {
		MMCBUS_ACQUIRE_BUS(mmcbus, dev);
		err = mmc_send_ext_csd(mmcbus, dev, sc->ext_csd);
		MMCBUS_RELEASE_BUS(mmcbus, dev);
		if (err != MMC_ERR_NONE) {
			device_printf(dev, "Error reading EXT_CSD %s\n",
			    mmcsd_errmsg(err));
			return (ENXIO);
		}
	}
	ext_csd = sc->ext_csd;

	if ((quirks & MMC_QUIRK_INAND_CMD38) != 0) {
		if (mmc_get_spec_vers(dev) < 4) {
			device_printf(dev,
			    "MMC_QUIRK_INAND_CMD38 set but no EXT_CSD\n");
			return (EINVAL);
		}
		sc->flags |= MMCSD_INAND_CMD38;
	}

	/*
	 * EXT_CSD_SEC_FEATURE_SUPPORT_GB_CL_EN denotes support for both
	 * insecure and secure TRIM.
	 */
	if ((ext_csd[EXT_CSD_SEC_FEATURE_SUPPORT] &
	    EXT_CSD_SEC_FEATURE_SUPPORT_GB_CL_EN) != 0 &&
	    (quirks & MMC_QUIRK_BROKEN_TRIM) == 0) {
		if (bootverbose)
			device_printf(dev, "taking advantage of TRIM\n");
		sc->flags |= MMCSD_USE_TRIM;
		sc->erase_sector = 1;
	} else
		sc->erase_sector = mmc_get_erase_sector(dev);

	/*
	 * Enhanced user data area and general purpose partitions are only
	 * supported in revision 1.4 (EXT_CSD_REV == 4) and later, the RPMB
	 * partition in revision 1.5 (MMC v4.41, EXT_CSD_REV == 5) and later.
	 */
	rev = ext_csd[EXT_CSD_REV];

	/*
	 * With revision 1.5 (MMC v4.5, EXT_CSD_REV == 6) and later, take
	 * advantage of the device R/W cache if present and useage is not
	 * disabled.
	 */
	if (rev >= 6 && mmcsd_cache != 0) {
		size = le32dec(&ext_csd[EXT_CSD_CACHE_SIZE]);
		if (bootverbose)
			device_printf(dev, "cache size %juKB\n", size);
		if (size > 0) {
			MMCBUS_ACQUIRE_BUS(mmcbus, dev);
			err = mmc_switch(mmcbus, dev, sc->rca,
			    EXT_CSD_CMD_SET_NORMAL, EXT_CSD_CACHE_CTRL,
			    EXT_CSD_CACHE_CTRL_CACHE_EN, sc->cmd6_time, true);
			MMCBUS_RELEASE_BUS(mmcbus, dev);
			if (err != MMC_ERR_NONE)
				device_printf(dev, "failed to enable cache\n");
			else
				sc->flags |= MMCSD_FLUSH_CACHE;
		}
	}

	/*
	 * Ignore user-creatable enhanced user data area and general purpose
	 * partitions partitions as long as partitioning hasn't been finished.
	 */
	comp = (ext_csd[EXT_CSD_PART_SET] & EXT_CSD_PART_SET_COMPLETED) != 0;

	/*
	 * Add enhanced user data area slice, unless it spans the entirety of
	 * the user data area.  The enhanced area is of a multiple of high
	 * capacity write protect groups ((ERASE_GRP_SIZE + HC_WP_GRP_SIZE) *
	 * 512 KB) and its offset given in either sectors or bytes, depending
	 * on whether it's a high capacity device or not.
	 * NB: The slicer and its slices need to be registered before adding
	 *     the disk for the corresponding user data area as re-tasting is
	 *     racy.
	 */
	sector_size = mmc_get_sector_size(dev);
	size = ext_csd[EXT_CSD_ENH_SIZE_MULT] +
	    (ext_csd[EXT_CSD_ENH_SIZE_MULT + 1] << 8) +
	    (ext_csd[EXT_CSD_ENH_SIZE_MULT + 2] << 16);
	if (rev >= 4 && comp == TRUE && size > 0 &&
	    (ext_csd[EXT_CSD_PART_SUPPORT] &
	    EXT_CSD_PART_SUPPORT_ENH_ATTR_EN) != 0 &&
	    (ext_csd[EXT_CSD_PART_ATTR] & (EXT_CSD_PART_ATTR_ENH_USR)) != 0) {
		erase_size = ext_csd[EXT_CSD_ERASE_GRP_SIZE] * 1024 *
		    MMC_SECTOR_SIZE;
		wp_size = ext_csd[EXT_CSD_HC_WP_GRP_SIZE];
		size *= erase_size * wp_size;
		if (size != mmc_get_media_size(dev) * sector_size) {
			sc->enh_size = size;
			sc->enh_base =
			    le32dec(&ext_csd[EXT_CSD_ENH_START_ADDR]) *
			    (sc->high_cap == 0 ? MMC_SECTOR_SIZE : 1);
		} else if (bootverbose)
			device_printf(dev,
			    "enhanced user data area spans entire device\n");
	}

	/*
	 * Add default partition.  This may be the only one or the user
	 * data area in case partitions are supported.
	 */
	ro = mmc_get_read_only(dev);
	mmcsd_add_part(sc, EXT_CSD_PART_CONFIG_ACC_DEFAULT, "mmcsd",
	    device_get_unit(dev), mmc_get_media_size(dev) * sector_size, ro);

	if (mmc_get_spec_vers(dev) < 3)
		return (0);

	/* Belatedly announce enhanced user data slice. */
	if (sc->enh_size != 0) {
		bytes = mmcsd_pretty_size(size, unit);
		printf(FLASH_SLICES_FMT ": %ju%sB enhanced user data area "
		    "slice offset 0x%jx at %s\n", device_get_nameunit(dev),
		    MMCSD_LABEL_ENH, bytes, unit, (uintmax_t)sc->enh_base,
		    device_get_nameunit(dev));
	}

	/*
	 * Determine partition switch timeout (provided in units of 10 ms)
	 * and ensure it's at least 300 ms as some eMMC chips lie.
	 */
	sc->part_time = max(ext_csd[EXT_CSD_PART_SWITCH_TO] * 10 * 1000,
	    300 * 1000);

	/* Add boot partitions, which are of a fixed multiple of 128 KB. */
	size = ext_csd[EXT_CSD_BOOT_SIZE_MULT] * MMC_BOOT_RPMB_BLOCK_SIZE;
	if (size > 0 && (mmcbr_get_caps(mmcbus) & MMC_CAP_BOOT_NOACC) == 0) {
		mmcsd_add_part(sc, EXT_CSD_PART_CONFIG_ACC_BOOT0,
		    MMCSD_FMT_BOOT, 0, size,
		    ro | ((ext_csd[EXT_CSD_BOOT_WP_STATUS] &
		    EXT_CSD_BOOT_WP_STATUS_BOOT0_MASK) != 0));
		mmcsd_add_part(sc, EXT_CSD_PART_CONFIG_ACC_BOOT1,
		    MMCSD_FMT_BOOT, 1, size,
		    ro | ((ext_csd[EXT_CSD_BOOT_WP_STATUS] &
		    EXT_CSD_BOOT_WP_STATUS_BOOT1_MASK) != 0));
	}

	/* Add RPMB partition, which also is of a fixed multiple of 128 KB. */
	size = ext_csd[EXT_CSD_RPMB_MULT] * MMC_BOOT_RPMB_BLOCK_SIZE;
	if (rev >= 5 && size > 0)
		mmcsd_add_part(sc, EXT_CSD_PART_CONFIG_ACC_RPMB,
		    MMCSD_FMT_RPMB, 0, size, ro);

	if (rev <= 3 || comp == FALSE)
		return (0);

	/*
	 * Add general purpose partitions, which are of a multiple of high
	 * capacity write protect groups, too.
	 */
	if ((ext_csd[EXT_CSD_PART_SUPPORT] & EXT_CSD_PART_SUPPORT_EN) != 0) {
		erase_size = ext_csd[EXT_CSD_ERASE_GRP_SIZE] * 1024 *
		    MMC_SECTOR_SIZE;
		wp_size = ext_csd[EXT_CSD_HC_WP_GRP_SIZE];
		for (i = 0; i < MMC_PART_GP_MAX; i++) {
			size = ext_csd[EXT_CSD_GP_SIZE_MULT + i * 3] +
			    (ext_csd[EXT_CSD_GP_SIZE_MULT + i * 3 + 1] << 8) +
			    (ext_csd[EXT_CSD_GP_SIZE_MULT + i * 3 + 2] << 16);
			if (size == 0)
				continue;
			mmcsd_add_part(sc, EXT_CSD_PART_CONFIG_ACC_GP0 + i,
			    MMCSD_FMT_GP, i, size * erase_size * wp_size, ro);
		}
	}
	return (0);
}

static uintmax_t
mmcsd_pretty_size(off_t size, char *unit)
{
	uintmax_t bytes;
	int i;

	/*
	 * Display in most natural units.  There's no card < 1MB.  However,
	 * RPMB partitions occasionally are smaller than that, though.  The
	 * SD standard goes to 2 GiB due to its reliance on FAT, but the data
	 * format supports up to 4 GiB and some card makers push it up to this
	 * limit.  The SDHC standard only goes to 32 GiB due to FAT32, but the
	 * data format supports up to 2 TiB however.  2048 GB isn't too ugly,
	 * so we note it in passing here and don't add the code to print TB).
	 * Since these cards are sold in terms of MB and GB not MiB and GiB,
	 * report them like that.  We also round to the nearest unit, since
	 * many cards are a few percent short, even of the power of 10 size.
	 */
	bytes = size;
	unit[0] = unit[1] = '\0';
	for (i = 0; i <= 2 && bytes >= 1000; i++) {
		bytes = (bytes + 1000 / 2 - 1) / 1000;
		switch (i) {
		case 0:
			unit[0] = 'k';
			break;
		case 1:
			unit[0] = 'M';
			break;
		case 2:
			unit[0] = 'G';
			break;
		default:
			break;
		}
	}
	return (bytes);
}

static struct cdevsw mmcsd_rpmb_cdevsw = {
	.d_version	= D_VERSION,
	.d_name		= "mmcsdrpmb",
	.d_ioctl	= mmcsd_ioctl_rpmb
};

static void
mmcsd_add_part(struct mmcsd_softc *sc, u_int type, const char *name, u_int cnt,
    off_t media_size, bool ro)
{
	struct make_dev_args args;
	device_t dev, mmcbus;
	const char *ext;
	const uint8_t *ext_csd;
	struct mmcsd_part *part;

	uintmax_t bytes;
	u_int gp;
	uint32_t speed;
	uint8_t extattr;
	bool enh;
	char unit[2];

	dev = sc->dev;
	mmcbus = sc->mmcbus;
	part = sc->part[type] = malloc(sizeof(*part), M_DEVBUF,
	    M_WAITOK | M_ZERO);
	part->sc = sc;
	part->cnt = cnt;
	part->type = type;
	part->ro = ro;
	snprintf(part->name, sizeof(part->name), name, device_get_unit(dev));

	MMCSD_IOCTL_LOCK_INIT(part);

	/*
	 * For the RPMB partition, allow IOCTL access only.
	 * NB: If ever attaching RPMB partitions to disk(9), the re-tuning
	 *     implementation and especially its pausing need to be revisited,
	 *     because then re-tuning requests may be issued by the IOCTL half
	 *     of this driver while re-tuning is already paused by the disk(9)
	 *     one and vice versa.
	 */
	if (type == EXT_CSD_PART_CONFIG_ACC_RPMB) {
		make_dev_args_init(&args);
		args.mda_flags = MAKEDEV_CHECKNAME | MAKEDEV_WAITOK;
		args.mda_devsw = &mmcsd_rpmb_cdevsw;
		args.mda_uid = UID_ROOT;
		args.mda_gid = GID_OPERATOR;
		args.mda_mode = 0640;
		args.mda_si_drv1 = part;
		if (make_dev_s(&args, &sc->rpmb_dev, "%s", part->name) != 0) {
			device_printf(dev, "Failed to make RPMB device\n");
			free(part, M_DEVBUF);
			return;
		}

	} else if (type != EXT_CSD_PART_CONFIG_ACC_DEFAULT) {
		printf("%s: Additional partition. This is currently not supported in RTEMS.", part->name);
	} else {
		MMCSD_DISK_LOCK_INIT(part);
		rtems_status_code status_code = rtems_media_server_disk_attach(
		    part->name, mmcsd_attach_worker, part);
		BSD_ASSERT(status_code == RTEMS_SUCCESSFUL);
	}

	bytes = mmcsd_pretty_size(media_size, unit);
	if (type == EXT_CSD_PART_CONFIG_ACC_DEFAULT) {
		speed = mmcbr_get_clock(mmcbus);
		printf("%s%d: %ju%sB <%s>%s at %s %d.%01dMHz/%dbit/%d-block\n",
		    part->name, cnt, bytes, unit, mmc_get_card_id_string(dev),
		    ro ? " (read-only)" : "", device_get_nameunit(mmcbus),
		    speed / 1000000, (speed / 100000) % 10,
		    mmcsd_bus_bit_width(dev), sc->max_data);
	} else if (type == EXT_CSD_PART_CONFIG_ACC_RPMB) {
		printf("%s: %ju%sB partition %d%s at %s\n", part->name, bytes,
		    unit, type, ro ? " (read-only)" : "",
		    device_get_nameunit(dev));
	} else {
		enh = false;
		ext = NULL;
		extattr = 0;
		if (type >= EXT_CSD_PART_CONFIG_ACC_GP0 &&
		    type <= EXT_CSD_PART_CONFIG_ACC_GP3) {
			ext_csd = sc->ext_csd;
			gp = type - EXT_CSD_PART_CONFIG_ACC_GP0;
			if ((ext_csd[EXT_CSD_PART_SUPPORT] &
			    EXT_CSD_PART_SUPPORT_ENH_ATTR_EN) != 0 &&
			    (ext_csd[EXT_CSD_PART_ATTR] &
			    (EXT_CSD_PART_ATTR_ENH_GP0 << gp)) != 0)
				enh = true;
			else if ((ext_csd[EXT_CSD_PART_SUPPORT] &
			    EXT_CSD_PART_SUPPORT_EXT_ATTR_EN) != 0) {
				extattr = (ext_csd[EXT_CSD_EXT_PART_ATTR +
				    (gp / 2)] >> (4 * (gp % 2))) & 0xF;
				switch (extattr) {
					case EXT_CSD_EXT_PART_ATTR_DEFAULT:
						break;
					case EXT_CSD_EXT_PART_ATTR_SYSTEMCODE:
						ext = "system code";
						break;
					case EXT_CSD_EXT_PART_ATTR_NPERSISTENT:
						ext = "non-persistent";
						break;
					default:
						ext = "reserved";
						break;
				}
			}
		}
		if (ext == NULL)
			printf("%s%d: %ju%sB partition %d%s%s at %s\n",
			    part->name, cnt, bytes, unit, type, enh ?
			    " enhanced" : "", ro ? " (read-only)" : "",
			    device_get_nameunit(dev));
		else
			printf("%s%d: %ju%sB partition %d extended 0x%x "
			    "(%s)%s at %s\n", part->name, cnt, bytes, unit,
			    type, extattr, ext, ro ? " (read-only)" : "",
			    device_get_nameunit(dev));
	}
}

static int
mmcsd_detach(device_t dev)
{
	BSD_PANIC("FIXME");
	return (0);
}

static int
mmcsd_shutdown(device_t dev)
{
	BSD_PANIC("FIXME");
	return (0);
}

static int
mmcsd_suspend(device_t dev)
{
	BSD_PANIC("FIXME");
	return (0);
}

static int
mmcsd_resume(device_t dev)
{
	BSD_PANIC("FIXME");
	return (0);
}

static int
mmcsd_ioctl_rpmb(struct cdev *dev, u_long cmd, caddr_t data,
    int fflag, struct thread *td)
{

	return (mmcsd_ioctl(dev->si_drv1, cmd, data, fflag, td));
}

static int
mmcsd_ioctl(struct mmcsd_part *part, u_long cmd, void *data, int fflag,
    struct thread *td)
{
	struct mmc_ioc_cmd *mic;
	struct mmc_ioc_multi_cmd *mimc;
	int i, err;
	u_long cnt, size;

	if ((fflag & FREAD) == 0)
		return (EBADF);

	err = priv_check(td, PRIV_DRIVER);
	if (err != 0)
		return (err);

	err = 0;
	switch (cmd) {
	case MMC_IOC_CMD:
		mic = data;
		err = mmcsd_ioctl_cmd(part, mic, fflag);
		break;
	case MMC_IOC_MULTI_CMD:
		mimc = data;
		if (mimc->num_of_cmds == 0)
			break;
		if (mimc->num_of_cmds > MMC_IOC_MAX_CMDS)
			return (EINVAL);
		cnt = mimc->num_of_cmds;
		size = sizeof(*mic) * cnt;
		mic = malloc(size, M_TEMP, M_WAITOK);
		err = copyin((const void *)mimc->cmds, mic, size);
		if (err == 0) {
			for (i = 0; i < cnt; i++) {
				err = mmcsd_ioctl_cmd(part, &mic[i], fflag);
				if (err != 0)
					break;
			}
		}
		free(mic, M_TEMP);
		break;
	default:
		return (ENOIOCTL);
	}
	return (err);
}

static int
mmcsd_ioctl_cmd(struct mmcsd_part *part, struct mmc_ioc_cmd *mic, int fflag)
{
	struct mmc_command cmd;
	struct mmc_data data;
	struct mmcsd_softc *sc;
	device_t dev, mmcbus;
	void *dp;
	u_long len;
	int err, retries;
	uint32_t status;
	uint16_t rca;

	if ((fflag & FWRITE) == 0 && mic->write_flag != 0)
		return (EBADF);

	if (part->ro == TRUE && mic->write_flag != 0)
		return (EROFS);

	/*
	 * We don't need to explicitly lock against the disk(9) half of this
	 * driver as MMCBUS_ACQUIRE_BUS() will serialize us.  However, it's
	 * necessary to protect against races with detachment and suspension,
	 * especially since it's required to switch away from RPMB partitions
	 * again after an access (see mmcsd_switch_part()).
	 */
	MMCSD_IOCTL_LOCK(part);
	while (part->ioctl != 0) {
		if (part->ioctl < 0) {
			MMCSD_IOCTL_UNLOCK(part);
			return (ENXIO);
		}
		msleep(part, &part->ioctl_mtx, 0, "mmcsd IOCTL", 0);
	}
	part->ioctl = 1;
	MMCSD_IOCTL_UNLOCK(part);

	err = 0;
	dp = NULL;
	len = mic->blksz * mic->blocks;
	if (len > MMC_IOC_MAX_BYTES) {
		err = EOVERFLOW;
		goto out;
	}
	if (len != 0) {
		dp = rtems_cache_aligned_malloc(len);
		err = copyin((void *)(uintptr_t)mic->data_ptr, dp, len);
		if (err != 0)
			goto out;
	}
	memset(&cmd, 0, sizeof(cmd));
	memset(&data, 0, sizeof(data));
	cmd.opcode = mic->opcode;
	cmd.arg = mic->arg;
	cmd.flags = mic->flags;
	if (len != 0) {
		data.len = len;
		data.data = dp;
		data.flags = mic->write_flag != 0 ? MMC_DATA_WRITE :
		    MMC_DATA_READ;
		cmd.data = &data;
	}
	sc = part->sc;
	rca = sc->rca;
	if (mic->is_acmd == 0) {
		/* Enforce/patch/restrict RCA-based commands */
		switch (cmd.opcode) {
		case MMC_SET_RELATIVE_ADDR:
		case MMC_SELECT_CARD:
			err = EPERM;
			goto out;
		case MMC_STOP_TRANSMISSION:
			if ((cmd.arg & 0x1) == 0)
				break;
			/* FALLTHROUGH */
		case MMC_SLEEP_AWAKE:
		case MMC_SEND_CSD:
		case MMC_SEND_CID:
		case MMC_SEND_STATUS:
		case MMC_GO_INACTIVE_STATE:
		case MMC_FAST_IO:
		case MMC_APP_CMD:
			cmd.arg = (cmd.arg & 0x0000FFFF) | (rca << 16);
			break;
		default:
			break;
		}
		/*
		 * No partition switching in userland; it's almost impossible
		 * to recover from that, especially if things go wrong.
		 */
		if (cmd.opcode == MMC_SWITCH_FUNC && dp != NULL &&
		    (((uint8_t *)dp)[EXT_CSD_PART_CONFIG] &
		    EXT_CSD_PART_CONFIG_ACC_MASK) != part->type) {
			err = EINVAL;
			goto out;
		}
	}
	dev = sc->dev;
	mmcbus = sc->mmcbus;
	MMCBUS_ACQUIRE_BUS(mmcbus, dev);
	err = mmcsd_switch_part(mmcbus, dev, rca, part->type);
	if (err != MMC_ERR_NONE)
		goto release;
	if (part->type == EXT_CSD_PART_CONFIG_ACC_RPMB) {
		err = mmcsd_set_blockcount(sc, mic->blocks,
		    mic->write_flag & (1 << 31));
		if (err != MMC_ERR_NONE)
			goto switch_back;
	}
	if (mic->write_flag != 0)
		sc->flags |= MMCSD_DIRTY;
	if (mic->is_acmd != 0)
		(void)mmc_wait_for_app_cmd(mmcbus, dev, rca, &cmd, 0);
	else
		(void)mmc_wait_for_cmd(mmcbus, dev, &cmd, 0);
	if (part->type == EXT_CSD_PART_CONFIG_ACC_RPMB) {
		/*
		 * If the request went to the RPMB partition, try to ensure
		 * that the command actually has completed.
		 */
		retries = MMCSD_CMD_RETRIES;
		do {
			err = mmc_send_status(mmcbus, dev, rca, &status);
			if (err != MMC_ERR_NONE)
				break;
			if (R1_STATUS(status) == 0 &&
			    R1_CURRENT_STATE(status) != R1_STATE_PRG)
				break;
			DELAY(1000);
		} while (retries-- > 0);
	}
	/*
	 * If EXT_CSD was changed, our copy is outdated now.  Specifically,
	 * the upper bits of EXT_CSD_PART_CONFIG used in mmcsd_switch_part(),
	 * so retrieve EXT_CSD again.
	 */
	if (cmd.opcode == MMC_SWITCH_FUNC) {
		err = mmc_send_ext_csd(mmcbus, dev, sc->ext_csd);
		if (err != MMC_ERR_NONE)
			goto release;
	}
switch_back:
	if (part->type == EXT_CSD_PART_CONFIG_ACC_RPMB) {
		/*
		 * If the request went to the RPMB partition, always switch
		 * back to the default partition (see mmcsd_switch_part()).
		 */
		err = mmcsd_switch_part(mmcbus, dev, rca,
		    EXT_CSD_PART_CONFIG_ACC_DEFAULT);
		if (err != MMC_ERR_NONE)
			goto release;
	}
	MMCBUS_RELEASE_BUS(mmcbus, dev);
	if (cmd.error != MMC_ERR_NONE) {
		switch (cmd.error) {
		case MMC_ERR_TIMEOUT:
			err = ETIMEDOUT;
			break;
		case MMC_ERR_BADCRC:
			err = EILSEQ;
			break;
		case MMC_ERR_INVALID:
			err = EINVAL;
			break;
		case MMC_ERR_NO_MEMORY:
			err = ENOMEM;
			break;
		default:
			err = EIO;
			break;
		}
		goto out;
	}
	memcpy(mic->response, cmd.resp, 4 * sizeof(uint32_t));
	if (mic->write_flag == 0 && len != 0) {
		err = copyout(dp, (void *)(uintptr_t)mic->data_ptr, len);
		if (err != 0)
			goto out;
	}
	goto out;

release:
	MMCBUS_RELEASE_BUS(mmcbus, dev);
	err = EIO;

out:
	MMCSD_IOCTL_LOCK(part);
	part->ioctl = 0;
	MMCSD_IOCTL_UNLOCK(part);
	wakeup(part);
	if (dp != NULL)
		free(dp, M_TEMP);
	return (err);
}

static int
mmcsd_set_blockcount(struct mmcsd_softc *sc, u_int count, bool reliable)
{
	struct mmc_command cmd;
	struct mmc_request req;

	memset(&req, 0, sizeof(req));
	memset(&cmd, 0, sizeof(cmd));
	cmd.mrq = &req;
	req.cmd = &cmd;
	cmd.opcode = MMC_SET_BLOCK_COUNT;
	cmd.arg = count & 0x0000FFFF;
	if (reliable)
		cmd.arg |= 1 << 31;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
	mmcbus_wait_for_request(sc->mmcbus, sc->dev, &req);
	return (cmd.error);
}

static int
mmcsd_switch_part(device_t bus, device_t dev, uint16_t rca, u_int part)
{
	struct mmcsd_softc *sc;
	int err;
	uint8_t	value;

	sc = device_get_softc(dev);

	if (sc->mode == mode_sd)
		return (MMC_ERR_NONE);

	/*
	 * According to section "6.2.2 Command restrictions" of the eMMC
	 * specification v5.1, CMD19/CMD21 aren't allowed to be used with
	 * RPMB partitions.  So we pause re-tuning along with triggering
	 * it up-front to decrease the likelihood of re-tuning becoming
	 * necessary while accessing an RPMB partition.  Consequently, an
	 * RPMB partition should immediately be switched away from again
	 * after an access in order to allow for re-tuning to take place
	 * anew.
	 */
	if (part == EXT_CSD_PART_CONFIG_ACC_RPMB)
		MMCBUS_RETUNE_PAUSE(sc->mmcbus, sc->dev, true);

	if (sc->part_curr == part)
		return (MMC_ERR_NONE);

	value = (sc->ext_csd[EXT_CSD_PART_CONFIG] &
	    ~EXT_CSD_PART_CONFIG_ACC_MASK) | part;
	/* Jump! */
	err = mmc_switch(bus, dev, rca, EXT_CSD_CMD_SET_NORMAL,
	    EXT_CSD_PART_CONFIG, value, sc->part_time, true);
	if (err != MMC_ERR_NONE) {
		if (part == EXT_CSD_PART_CONFIG_ACC_RPMB)
			MMCBUS_RETUNE_UNPAUSE(sc->mmcbus, sc->dev);
		return (err);
	}

	sc->ext_csd[EXT_CSD_PART_CONFIG] = value;
	if (sc->part_curr == EXT_CSD_PART_CONFIG_ACC_RPMB)
		MMCBUS_RETUNE_UNPAUSE(sc->mmcbus, sc->dev);
	sc->part_curr = part;
	return (MMC_ERR_NONE);
}

static const char *
mmcsd_errmsg(int e)
{

	if (e < 0 || e > MMC_ERR_MAX)
		return "Bad error code";
	return (errmsg[e]);
}

static int
mmcsd_bus_bit_width(device_t dev)
{

	if (mmc_get_bus_width(dev) == bus_width_1)
		return (1);
	if (mmc_get_bus_width(dev) == bus_width_4)
		return (4);
	return (8);
}

static device_method_t mmcsd_methods[] = {
	DEVMETHOD(device_probe, mmcsd_probe),
	DEVMETHOD(device_attach, mmcsd_attach),
	DEVMETHOD(device_detach, mmcsd_detach),
	DEVMETHOD(device_shutdown, mmcsd_shutdown),
	DEVMETHOD(device_suspend, mmcsd_suspend),
	DEVMETHOD(device_resume, mmcsd_resume),
	DEVMETHOD_END
};

static driver_t mmcsd_driver = {
	"mmcsd",
	mmcsd_methods,
	sizeof(struct mmcsd_softc),
};
static devclass_t mmcsd_devclass;

static int
mmcsd_handler(module_t mod __unused, int what, void *arg __unused)
{
	return (0);
}

DRIVER_MODULE(mmcsd, mmc, mmcsd_driver, mmcsd_devclass, mmcsd_handler, NULL);
MODULE_DEPEND(mmcsd, g_flashmap, 0, 0, 0);
MMC_DEPEND(mmcsd);
