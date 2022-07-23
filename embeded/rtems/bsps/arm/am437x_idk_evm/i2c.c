/*
 * CopyRight 2022 wtcat
 */
#include <stdlib.h>
#include <rtems/bspIo.h>
#include <rtems/counter.h>
#include <rtems/malloc.h>

#include <dev/i2c/i2c.h>

#include "bsp/platform_bus.h"
#include "bsp/io.h"


//#define DEBUG_ON

struct i2c_regs {
	uint32_t I2C_REVNB_LO;
	uint32_t I2C_REVNB_HI;
	uint32_t RESERVED_0[2];
	uint32_t I2C_SYSC;
	uint32_t RESERVED_1[4];
	uint32_t I2C_IRQSTATUS_RAW;
	uint32_t I2C_IRQSTATUS;
	uint32_t I2C_IRQENABLE_SET;
	uint32_t I2C_IRQENABLE_CLR;
	uint32_t I2C_WE;
	uint32_t I2C_DMARXENABLE_SET;
	uint32_t I2C_DMATXENABLE_SET;
	uint32_t I2C_DMARXENABLE_CLR;
	uint32_t I2C_DMATXENABLE_CLR;
	uint32_t I2C_DMARXWAKE_EN;
	uint32_t I2C_DMATXWAKE_EN;
	uint32_t RESERVED_2[16];
	uint32_t I2C_SYSS;
	uint32_t I2C_BUF;
	uint32_t I2C_CNT;
	uint32_t I2C_DATA;
	uint32_t RESERVED_3;
	uint32_t I2C_CON;
	uint32_t I2C_OA;
	uint32_t I2C_SA;
	uint32_t I2C_PSC;
	uint32_t I2C_SCLL;
	uint32_t I2C_SCLH;
	uint32_t I2C_SYSTEST;
	uint32_t I2C_BUFSTAT;
	uint32_t I2C_OA1;
	uint32_t I2C_OA2;
	uint32_t I2C_OA3;
	uint32_t I2C_ACTOA;
	uint32_t I2C_SBLOCK;
};

struct i2c_private {
	i2c_bus bus;
	struct drvmgr_dev *dev;
	i2c_msg *buffer;
	size_t buffer_pos;
	rtems_id thread;
	volatile struct i2c_regs *regs;
	uint32_t clkctrl;
	uint32_t con_reg;
	int error;
};

struct i2c_clockrate {
	uint32_t iclk;
	uint32_t fclk;
};

#define TRANSFER_TIMEOUT_COUNT 100
#define FIFO_THRESHOLD 5
#define min(l,r) ((l) < (r) ? (l) : (r))

#define AM437X_I2C_CON_XSA  (0x00000100u)
#define AM437X_I2C_CFG_10BIT_SLAVE_ADDR  AM437X_I2C_CON_XSA
#define AM437X_I2C_CON_XSA_SHIFT  (0x00000008u)
#define AM437X_I2C_CFG_7BIT_SLAVE_ADDR  (0 << AM437X_I2C_CON_XSA_SHIFT)
#define AM437X_I2C_CON_I2C_EN   (0x00008000u)
#define AM437X_I2C_CON_TRX   (0x00000200u)
#define AM437X_I2C_CON_MST   (0x00000400u)
#define AM437X_I2C_CON_STB   (0x00000800u)
#define AM437X_I2C_SYSC_AUTOIDLE   (0x00000001u)
#define AM437X_I2C_SYSC_SRST       (0x00000002u)
#define AM437X_I2C_SYSC_ENAWAKEUP  (0x00000004u)
#define AM437X_I2C_SYSS_RDONE      (0x00000001u)

#define AM437X_I2C_BUF_TXTRSH_SHIFT (0)
#define AM437X_I2C_BUF_TXTRSH_MASK  (0x0000003Fu)
#define AM437X_I2C_BUF_TXTRSH(X)    (((X) << AM437X_I2C_BUF_TXTRSH_SHIFT) \
                                     & AM437X_I2C_BUF_TXTRSH_MASK)
#define AM437X_I2C_BUF_TXFIFO_CLR   (0x00000040u)
#define AM437X_I2C_BUF_RXTRSH_SHIFT (8)
#define AM437X_I2C_BUF_RXTRSH_MASK  (0x00003F00u)
#define AM437X_I2C_BUF_RXTRSH(X)    (((X) << AM437X_I2C_BUF_RXTRSH_SHIFT) \
                                     & AM437X_I2C_BUF_RXTRSH_MASK)
#define AM437X_I2C_BUF_RXFIFO_CLR   (0x00004000u)

/* I2C status Register */
#define AM437X_I2C_IRQSTATUS_AL   (1 << 0)
#define AM437X_I2C_IRQSTATUS_NACK (1 << 1)
#define AM437X_I2C_IRQSTATUS_ARDY (1 << 2)
#define AM437X_I2C_IRQSTATUS_RRDY (1 << 3)
#define AM437X_I2C_IRQSTATUS_XRDY (1 << 4)
#define AM437X_I2C_IRQSTATUS_GC   (1 << 5)
#define AM437X_I2C_IRQSTATUS_STC  (1 << 6)
#define AM437X_I2C_IRQSTATUS_AERR (1 << 7)
#define AM437X_I2C_IRQSTATUS_BF   (1 << 8)
#define AM437X_I2C_IRQSTATUS_AAS  (1 << 9)
#define AM437X_I2C_IRQSTATUS_XUDF (1 << 10)
#define AM437X_I2C_IRQSTATUS_ROVR (1 << 11)
#define AM437X_I2C_IRQSTATUS_BB   (1 << 12)
#define AM437X_I2C_IRQSTATUS_RDR  (1 << 13)
#define AM437X_I2C_IRQSTATUS_XDR  (1 << 14)

#define AM437X_I2C_INT_RECV_READY AM437X_I2C_IRQSTATUS_RRDY
#define AM437X_I2C_CON_STOP  (0x00000002u)
#define AM437X_I2C_CON_START (0x00000001u)
#define AM437X_I2C_CFG_MST_RX AM437X_I2C_CON_MST
#define AM437X_I2C_CFG_MST_TX  (AM437X_I2C_CON_TRX | AM437X_I2C_CON_MST)
#define AM437X_CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L4_GCLK (0x00000020u)
#define AM437X_I2C_INT_STOP_CONDITION AM437X_I2C_IRQSTATUS_BF

#define I2C_SYSCLK       48000000
#define I2C_INTERNAL_CLK 12000000

#ifdef DEBUG_ON
#define devdbg(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define devdbg(...)
#endif

#ifdef DEBUG_ON
static void am437x_i2c_regs_dump(struct i2c_private *i2c) {
	volatile struct i2c_regs *regs = i2c->regs;
	printk("[I2C Register]:\n");
	printk("  I2C_REVNB_LO = 0x%x\n", regs->I2C_REVNB_LO);
	printk("  I2C_REVNB_HI = 0x%x\n", regs->I2C_REVNB_HI);
	printk("  I2C_SYSS = 0x%x\n", regs->I2C_SYSS);
}
#endif

static int am437x_i2c_set_clock(i2c_bus *bus, 
	unsigned long clock) {
	struct i2c_private *i2c = (struct i2c_private *)bus;
	uint32_t prescaler, divider;
	prescaler = (I2C_SYSCLK / I2C_INTERNAL_CLK) - 1;
	i2c->regs->I2C_PSC = prescaler;
	divider = I2C_INTERNAL_CLK / (2 * clock);
	i2c->regs->I2C_SCLL = divider - 7;
	i2c->regs->I2C_SCLH = divider - 5;
	return 0;
}

static int am437x_i2c_reset(struct i2c_private *i2c) {
	volatile struct i2c_regs *regs = i2c->regs;
	int timeout = 100;
	int err;

	i2c->con_reg = 0;
	regs->I2C_CON = i2c->con_reg;
	rtems_counter_delay_nanoseconds(50000000);
	regs->I2C_SYSC = AM437X_I2C_SYSC_SRST;
	rtems_counter_delay_nanoseconds(1000000);
	regs->I2C_CON = AM437X_I2C_CON_I2C_EN;
	while (!(regs->I2C_SYSS & AM437X_I2C_SYSS_RDONE) && timeout >= 0 ) {
		--timeout;
		rtems_counter_delay_nanoseconds(100000);
	}
	if ( timeout <= 0 ) {
		printk("[%s] ERROR(%s): Timeout in soft-reset\n", 
			__func__, i2c->dev->name);
		return ETIMEDOUT;
	}
	/* Disable again after reset */
	regs->I2C_CON = i2c->con_reg;
	err = am437x_i2c_set_clock(&i2c->bus, I2C_BUS_CLOCK_DEFAULT);
	if (err)
		return err;
	regs->I2C_BUF = AM437X_I2C_BUF_TXTRSH(FIFO_THRESHOLD) |
					AM437X_I2C_BUF_RXTRSH(FIFO_THRESHOLD);

	/* Enable the I2C controller in master mode. */
	i2c->con_reg |= AM437X_I2C_CON_I2C_EN | AM437X_I2C_CON_MST;
	regs->I2C_CON = i2c->con_reg;
	regs->I2C_IRQENABLE_SET =
		AM437X_I2C_IRQSTATUS_XDR | AM437X_I2C_IRQSTATUS_XRDY |
		AM437X_I2C_IRQSTATUS_RDR | AM437X_I2C_IRQSTATUS_RRDY |
		AM437X_I2C_IRQSTATUS_ARDY | AM437X_I2C_IRQSTATUS_NACK |
		AM437X_I2C_IRQSTATUS_AL;
	return 0;
}

/* Return true if done. */
static bool am437x_i2c_transfer_intr(struct i2c_private *i2c,
	uint32_t status) {
	volatile struct i2c_regs *regs = i2c->regs;
	size_t amount = 0;
	size_t i;

	/* Handle errors */
	if ((status & AM437X_I2C_IRQSTATUS_NACK) != 0) {
		printk("NACK\n");
		regs->I2C_IRQSTATUS = AM437X_I2C_IRQSTATUS_NACK;
		i2c->error = ENXIO;
	} else if ((status & AM437X_I2C_IRQSTATUS_AL) != 0) {
		printk("Arbitration lost\n");
		regs->I2C_IRQSTATUS = AM437X_I2C_IRQSTATUS_AL;
		i2c->error = ENXIO;
	}
	/* Transfer finished? */
	if ((status & AM437X_I2C_IRQSTATUS_ARDY) != 0) {
		devdbg("ARDY transaction complete\n");
		if (i2c->error != 0 && (i2c->buffer->flags & I2C_M_STOP) == 0) {
			regs->I2C_CON = i2c->con_reg | AM437X_I2C_CON_STOP;
		}
		regs->I2C_IRQSTATUS = AM437X_I2C_IRQSTATUS_ARDY |
								AM437X_I2C_IRQSTATUS_RDR |
								AM437X_I2C_IRQSTATUS_RRDY |
								AM437X_I2C_IRQSTATUS_XDR |
								AM437X_I2C_IRQSTATUS_XRDY;
		return true;
	}
	if (i2c->buffer->flags & I2C_M_RD) {
		if (status & AM437X_I2C_IRQSTATUS_RDR) {
			devdbg("RDR\n");
			/* last data received */
			amount = i2c->buffer->len - i2c->buffer_pos;
		} else if (status & AM437X_I2C_IRQSTATUS_RRDY) {
			devdbg("RRDY\n");
			/* FIFO threshold reached */
			amount = min(FIFO_THRESHOLD, i2c->buffer->len - i2c->buffer_pos);
		}

		devdbg("Read %d bytes\n", amount);
		for (i = 0; i < amount; i++) {
			i2c->buffer->buf[i2c->buffer_pos] = (uint8_t)(regs->I2C_DATA);
			++i2c->buffer_pos;
		}
		if (status & AM437X_I2C_IRQSTATUS_RDR)
			regs->I2C_IRQSTATUS =AM437X_I2C_IRQSTATUS_RDR;
		if (status & AM437X_I2C_IRQSTATUS_RRDY)
			regs->I2C_IRQSTATUS =AM437X_I2C_IRQSTATUS_RRDY;
	} else {
		if (status & AM437X_I2C_IRQSTATUS_XDR) {
			devdbg("XDR\n");
			/* Remaining TX data won't reach the FIFO threshold. */
			amount = i2c->buffer->len - i2c->buffer_pos;
		} else if (status & AM437X_I2C_IRQSTATUS_XRDY) {
			devdbg("XRDY\n");
			/* FIFO threshold reached */
			amount = min(FIFO_THRESHOLD, i2c->buffer->len - i2c->buffer_pos);
		}

		devdbg("Write %d bytes\n", amount);
		for (i = 0; i < amount; i++) {
			regs->I2C_DATA = i2c->buffer->buf[i2c->buffer_pos];
			++i2c->buffer_pos;
		}
		if (status & AM437X_I2C_IRQSTATUS_XDR)
			regs->I2C_IRQSTATUS = AM437X_I2C_IRQSTATUS_XDR;
		if (status & AM437X_I2C_IRQSTATUS_XRDY)
			regs->I2C_IRQSTATUS = AM437X_I2C_IRQSTATUS_XRDY;
	}
	return false;
}

static void am437x_i2c_interrupt(void *arg) {
	struct i2c_private *i2c = arg;
	volatile struct i2c_regs *regs = i2c->regs;
	uint32_t status = regs->I2C_IRQSTATUS;
	rtems_status_code sc;

	devdbg("interrupt: %08x\n", status);
	if (status == 0) {
		/* Why can this even happen? */
		return;
	}
	if (i2c->buffer == NULL) {
		devdbg("Buffer is NULL\n");
		i2c->error = EINVAL;
		goto _wakeup;
	}
	if (!am437x_i2c_transfer_intr(i2c, status)) 
		return;
_wakeup:
	sc = rtems_event_transient_send(i2c->thread);
	_Assert( sc == RTEMS_SUCCESSFUL );
	(void) sc;
}

static int am437x_i2c_transfer(i2c_bus *bus, i2c_msg *msgs, 
	uint32_t nmsgs) {
	struct i2c_private *i2c = (struct i2c_private *)bus;
	volatile struct i2c_regs *regs = i2c->regs;
	bool repstart = false;
	int timeout = 0;
	int err = 0;
	uint32_t reg;
	rtems_status_code sc;

	i2c->thread = rtems_task_self();
	for (size_t i = 0; i < nmsgs; i++) {
		i2c->buffer = &msgs[i];
		i2c->buffer_pos = 0;
		i2c->error = 0;
		devdbg("processing %2d/%d: addr: 0x%04x, flags: 0x%04x, len: %d, buf: %p\n",
			i, nmsgs, msgs[i].addr, msgs[i].flags, msgs[i].len, msgs[i].buf);
		if (i2c->buffer == NULL || i2c->buffer->buf == NULL ||
			i2c->buffer->len == 0) {
			err = EINVAL;
			break;
		}
		/*
		 * Send START when bus is busy on repeated starts.
		 * Otherwise wait some time.
		 */
		if (!repstart) {
			timeout = 0;
			while ((regs->I2C_IRQSTATUS_RAW & AM437X_I2C_IRQSTATUS_BB) != 0 &&
					timeout <= TRANSFER_TIMEOUT_COUNT) {
				++timeout;
				rtems_task_wake_after(RTEMS_MICROSECONDS_TO_TICKS(1000));
			}
			if (timeout > TRANSFER_TIMEOUT_COUNT) {
				err = EBUSY;
				break;
			}
			timeout = 0;
		} else {
			repstart = false;
		}
		if ((i2c->buffer->flags & I2C_M_STOP) == 0)
			repstart = true;
		regs->I2C_SA = i2c->buffer->addr;
		regs->I2C_CNT = i2c->buffer->len;
		regs->I2C_BUF |= AM437X_I2C_BUF_RXFIFO_CLR | AM437X_I2C_BUF_TXFIFO_CLR;
		reg = i2c->con_reg | AM437X_I2C_CON_START;
		if (!repstart) 
			reg |= AM437X_I2C_CON_STOP;
		if ((i2c->buffer->flags & I2C_M_RD) == 0) 
			reg |= AM437X_I2C_CON_TRX;
		
		/* Implicit stop on last message. */
		if (i == nmsgs - 1) 
			reg |= AM437X_I2C_CON_STOP;
		regs->I2C_CON = reg;
		sc = rtems_event_transient_receive(RTEMS_WAIT, i2c->bus.timeout);
		if ( sc != RTEMS_SUCCESSFUL ) {
			rtems_event_transient_clear();
			err = ETIMEDOUT;
			break;
		}
		if (i2c->error) {
			err = i2c->error;
			break;
		}
	}
	if (timeout == 0) {
		while ((regs->I2C_IRQSTATUS_RAW & AM437X_I2C_IRQSTATUS_BB) != 0 &&
				timeout <= TRANSFER_TIMEOUT_COUNT) {
			++timeout;
			rtems_task_wake_after(RTEMS_MICROSECONDS_TO_TICKS(1000));
		}
	}
	if ((regs->I2C_CON & AM437X_I2C_CON_MST) == 0)
		regs->I2C_CON = i2c->con_reg;
	i2c->buffer = NULL;
	return -err;
}

static void am437x_i2c_destroy(i2c_bus *bus) {
	struct i2c_private *i2c = (struct i2c_private *)bus;
	i2c->regs->I2C_IRQENABLE_CLR = 0xFFFF;
	i2c->regs->I2C_CON = 0;
	drvmgr_interrupt_unregister(i2c->dev, 0, am437x_i2c_interrupt, i2c);
	i2c_bus_destroy_and_free(bus);
}

static int i2c_bus_unite(struct drvmgr_drv *drv, 
	struct drvmgr_dev *dev) {
	return platform_bus_match(drv, dev, DRVMGR_BUS_TYPE_I2C);
}

static struct drvmgr_bus_ops i2c_bus_ops = {
	.init = {platform_bus_populate, },
	.unite = i2c_bus_unite,
};

static int i2c_probe(struct drvmgr_dev *dev) {
	union drvmgr_key_value *prop;
	struct i2c_private *i2c;
    struct dev_private *devp;
	int ret = -EINVAL;
    i2c = rtems_calloc(1, sizeof(struct i2c_private));
    if (i2c == NULL) 
        return -DRVMGR_NOMEM;
    devp = device_get_private(dev);
	prop = devcie_get_property(dev, "CLKCTRL");
	if (!prop) 
		goto _free;
	i2c->clkctrl = prop->i;
    i2c->regs = (volatile struct i2c_regs *)devp->base;
	i2c->dev = dev;
	dev->priv = i2c;

	/* Enable module and reset */
	writeb(0x2, i2c->clkctrl);
	ret = am437x_i2c_reset(i2c);
#ifdef DEBUG_ON
	am437x_i2c_regs_dump(i2c);
#endif
	if (ret) 
		goto _free;
    ret = drvmgr_interrupt_register(dev, 0, dev->name, 
		am437x_i2c_interrupt, i2c);
    if (ret) {
        devdbg("Register irq for %s failed\n", dev->name);
        goto _free;
    }
	ret = platform_bus_device_register(dev, &i2c_bus_ops, 
		DRVMGR_BUS_TYPE_I2C);
	if (ret) {
		devdbg("Register I2C bus device（%s） failed\n", dev->name);
		goto _remove_irq;
	}
	am437x_i2c_set_clock(&i2c->bus, 400000);
	i2c_bus_init(&i2c->bus);
	i2c->bus.transfer = am437x_i2c_transfer;
	i2c->bus.set_clock = am437x_i2c_set_clock;
	i2c->bus.destroy = am437x_i2c_destroy;
	ret = i2c_bus_register(&i2c->bus, platform_dev_filename(dev));
	if (ret)
		goto _destory;
	return 0;
_destory:
	i2c_bus_destroy(&i2c->bus);
_remove_irq:
	drvmgr_interrupt_unregister(dev, 0, am437x_i2c_interrupt, i2c);
_free:
	free(i2c);
	return ret;
}

static const struct dev_id id_table[] = {
    {.compatible = "ti,am4372-i2c", NULL},
	{.compatible = "ti,omap4-i2c", NULL},
    {NULL, NULL}
};

static struct drvmgr_drv_ops i2c_driver_ops = {
	.init = { i2c_probe, }
};
		
PLATFORM_DRIVER(i2c_bus) = {
	.drv = {
		.drv_id   = DRIVER_PLATFORM_ID,
		.name     = "i2c",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &i2c_driver_ops
	},
	.ids = id_table
};
