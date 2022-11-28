/*
 * CopyRight 2022 wtcat
 */
#include <stdint.h>
#include <stdlib.h>

#include <rtems/thread.h>
#include <rtems/bspIo.h>
#include <rtems/counter.h>
#include <rtems/malloc.h>
#include <rtems/rtems/intr.h>


#include "drivers/pinctrl.h"
#include "drivers/dma.h"
#include "drivers/i2c.h"
#include "drivers/clock.h"
#include "drivers/mio.h"
#include "drivers/ofw_platform_bus.h"
#undef LIST_HEAD

#include "base/list.h"

#include "stm32h7xx_ll_rcc.h"
#include <dt-bindings/clock/stm32h7-clks.h>


#ifndef __iomem
#define __iomem
#endif

#undef HZ
#define HZ RTEMS_MICROSECONDS_TO_TICKS(1000000)

/* STM32F7 I2C registers */
#define STM32F7_I2C_CR1				0x00
#define STM32F7_I2C_CR2				0x04
#define STM32F7_I2C_OAR1			0x08
#define STM32F7_I2C_OAR2			0x0C
#define STM32F7_I2C_PECR			0x20
#define STM32F7_I2C_TIMINGR			0x10
#define STM32F7_I2C_ISR				0x18
#define STM32F7_I2C_ICR				0x1C
#define STM32F7_I2C_RXDR			0x24
#define STM32F7_I2C_TXDR			0x28

/* STM32F7 I2C control 1 */
#define STM32F7_I2C_CR1_PECEN			BIT(23)
#define STM32F7_I2C_CR1_SMBHEN			BIT(20)
#define STM32F7_I2C_CR1_WUPEN			BIT(18)
#define STM32F7_I2C_CR1_SBC			BIT(16)
#define STM32F7_I2C_CR1_RXDMAEN			BIT(15)
#define STM32F7_I2C_CR1_TXDMAEN			BIT(14)
#define STM32F7_I2C_CR1_ANFOFF			BIT(12)
#define STM32F7_I2C_CR1_ERRIE			BIT(7)
#define STM32F7_I2C_CR1_TCIE			BIT(6)
#define STM32F7_I2C_CR1_STOPIE			BIT(5)
#define STM32F7_I2C_CR1_NACKIE			BIT(4)
#define STM32F7_I2C_CR1_ADDRIE			BIT(3)
#define STM32F7_I2C_CR1_RXIE			BIT(2)
#define STM32F7_I2C_CR1_TXIE			BIT(1)
#define STM32F7_I2C_CR1_PE			BIT(0)
#define STM32F7_I2C_ALL_IRQ_MASK		(STM32F7_I2C_CR1_ERRIE \
						| STM32F7_I2C_CR1_TCIE \
						| STM32F7_I2C_CR1_STOPIE \
						| STM32F7_I2C_CR1_NACKIE \
						| STM32F7_I2C_CR1_RXIE \
						| STM32F7_I2C_CR1_TXIE)
#define STM32F7_I2C_XFER_IRQ_MASK		(STM32F7_I2C_CR1_TCIE \
						| STM32F7_I2C_CR1_STOPIE \
						| STM32F7_I2C_CR1_NACKIE \
						| STM32F7_I2C_CR1_RXIE \
						| STM32F7_I2C_CR1_TXIE)

/* STM32F7 I2C control 2 */
#define STM32F7_I2C_CR2_PECBYTE			BIT(26)
#define STM32F7_I2C_CR2_RELOAD			BIT(24)
#define STM32F7_I2C_CR2_NBYTES_MASK		GENMASK(23, 16)
#define STM32F7_I2C_CR2_NBYTES(n)		(((n) & 0xff) << 16)
#define STM32F7_I2C_CR2_NACK			BIT(15)
#define STM32F7_I2C_CR2_STOP			BIT(14)
#define STM32F7_I2C_CR2_START			BIT(13)
#define STM32F7_I2C_CR2_HEAD10R			BIT(12)
#define STM32F7_I2C_CR2_ADD10			BIT(11)
#define STM32F7_I2C_CR2_RD_WRN			BIT(10)
#define STM32F7_I2C_CR2_SADD10_MASK		GENMASK(9, 0)
#define STM32F7_I2C_CR2_SADD10(n)		(((n) & \
						STM32F7_I2C_CR2_SADD10_MASK))
#define STM32F7_I2C_CR2_SADD7_MASK		GENMASK(7, 1)
#define STM32F7_I2C_CR2_SADD7(n)		(((n) & 0x7f) << 1)

/* STM32F7 I2C Own Address 1 */
#define STM32F7_I2C_OAR1_OA1EN			BIT(15)
#define STM32F7_I2C_OAR1_OA1MODE		BIT(10)
#define STM32F7_I2C_OAR1_OA1_10_MASK		GENMASK(9, 0)
#define STM32F7_I2C_OAR1_OA1_10(n)		(((n) & \
						STM32F7_I2C_OAR1_OA1_10_MASK))
#define STM32F7_I2C_OAR1_OA1_7_MASK		GENMASK(7, 1)
#define STM32F7_I2C_OAR1_OA1_7(n)		(((n) & 0x7f) << 1)
#define STM32F7_I2C_OAR1_MASK			(STM32F7_I2C_OAR1_OA1_7_MASK \
						| STM32F7_I2C_OAR1_OA1_10_MASK \
						| STM32F7_I2C_OAR1_OA1EN \
						| STM32F7_I2C_OAR1_OA1MODE)

/* STM32F7 I2C Own Address 2 */
#define STM32F7_I2C_OAR2_OA2EN			BIT(15)
#define STM32F7_I2C_OAR2_OA2MSK_MASK		GENMASK(10, 8)
#define STM32F7_I2C_OAR2_OA2MSK(n)		(((n) & 0x7) << 8)
#define STM32F7_I2C_OAR2_OA2_7_MASK		GENMASK(7, 1)
#define STM32F7_I2C_OAR2_OA2_7(n)		(((n) & 0x7f) << 1)
#define STM32F7_I2C_OAR2_MASK			(STM32F7_I2C_OAR2_OA2MSK_MASK \
						| STM32F7_I2C_OAR2_OA2_7_MASK \
						| STM32F7_I2C_OAR2_OA2EN)

/* STM32F7 I2C Interrupt Status */
#define STM32F7_I2C_ISR_ADDCODE_MASK		GENMASK(23, 17)
#define STM32F7_I2C_ISR_ADDCODE_GET(n) \
				(((n) & STM32F7_I2C_ISR_ADDCODE_MASK) >> 17)
#define STM32F7_I2C_ISR_DIR			BIT(16)
#define STM32F7_I2C_ISR_BUSY			BIT(15)
#define STM32F7_I2C_ISR_PECERR			BIT(11)
#define STM32F7_I2C_ISR_ARLO			BIT(9)
#define STM32F7_I2C_ISR_BERR			BIT(8)
#define STM32F7_I2C_ISR_TCR			BIT(7)
#define STM32F7_I2C_ISR_TC			BIT(6)
#define STM32F7_I2C_ISR_STOPF			BIT(5)
#define STM32F7_I2C_ISR_NACKF			BIT(4)
#define STM32F7_I2C_ISR_ADDR			BIT(3)
#define STM32F7_I2C_ISR_RXNE			BIT(2)
#define STM32F7_I2C_ISR_TXIS			BIT(1)
#define STM32F7_I2C_ISR_TXE			BIT(0)

/* STM32F7 I2C Interrupt Clear */
#define STM32F7_I2C_ICR_PECCF			BIT(11)
#define STM32F7_I2C_ICR_ARLOCF			BIT(9)
#define STM32F7_I2C_ICR_BERRCF			BIT(8)
#define STM32F7_I2C_ICR_STOPCF			BIT(5)
#define STM32F7_I2C_ICR_NACKCF			BIT(4)
#define STM32F7_I2C_ICR_ADDRCF			BIT(3)

/* STM32F7 I2C Timing */
#define STM32F7_I2C_TIMINGR_PRESC(n)		(((n) & 0xf) << 28)
#define STM32F7_I2C_TIMINGR_SCLDEL(n)		(((n) & 0xf) << 20)
#define STM32F7_I2C_TIMINGR_SDADEL(n)		(((n) & 0xf) << 16)
#define STM32F7_I2C_TIMINGR_SCLH(n)		(((n) & 0xff) << 8)
#define STM32F7_I2C_TIMINGR_SCLL(n)		((n) & 0xff)

#define STM32F7_I2C_MAX_LEN			0xff
#define STM32F7_I2C_DMA_LEN_MIN			0x16
enum {
	STM32F7_SLAVE_HOSTNOTIFY,
	STM32F7_SLAVE_7_10_BITS_ADDR,
	STM32F7_SLAVE_7_BITS_ADDR,
	STM32F7_I2C_MAX_SLAVE
};

#define STM32F7_I2C_DNF_DEFAULT			0
#define STM32F7_I2C_DNF_MAX			16

#define STM32F7_I2C_ANALOG_FILTER_ENABLE	1
#define STM32F7_I2C_ANALOG_FILTER_DELAY_MIN	50	/* ns */
#define STM32F7_I2C_ANALOG_FILTER_DELAY_MAX	260	/* ns */

#define STM32F7_I2C_RISE_TIME_DEFAULT		25	/* ns */
#define STM32F7_I2C_FALL_TIME_DEFAULT		10	/* ns */

#define STM32F7_PRESC_MAX			BIT(4)
#define STM32F7_SCLDEL_MAX			BIT(4)
#define STM32F7_SDADEL_MAX			BIT(4)
#define STM32F7_SCLH_MAX			BIT(8)
#define STM32F7_SCLL_MAX			BIT(8)

// #define STM32F7_AUTOSUSPEND_DELAY		(HZ / 100)



enum stm32_i2c_speed {
	STM32_I2C_SPEED_STANDARD	= 100000,
	STM32_I2C_SPEED_FAST		= 400000,
	STM32_I2C_SPEED_FAST_PLUS	= 1000000,
	STM32_I2C_SPEED_END,
};

struct i2c_timings {
	u32 bus_freq_hz;
	u32 scl_rise_ns;
	u32 scl_fall_ns;
	u32 scl_int_delay_ns;
	u32 sda_fall_ns;
	u32 sda_hold_ns;
	u32 digital_filter_width_ns;
	u32 analog_filter_cutoff_freq_hz;
};

/**
 * struct stm32f7_i2c_regs - i2c f7 registers backup
 * @cr1: Control register 1
 * @cr2: Control register 2
 * @oar1: Own address 1 register
 * @oar2: Own address 2 register
 * @tmgr: Timing register
 */
struct stm32f7_i2c_regs {
	u32 cr1;
	u32 cr2;
	u32 oar1;
	u32 oar2;
	u32 tmgr;
};

/**
 * struct stm32f7_i2c_spec - private i2c specification timing
 * @rate: I2C bus speed (Hz)
 * @fall_max: Max fall time of both SDA and SCL signals (ns)
 * @rise_max: Max rise time of both SDA and SCL signals (ns)
 * @hddat_min: Min data hold time (ns)
 * @vddat_max: Max data valid time (ns)
 * @sudat_min: Min data setup time (ns)
 * @l_min: Min low period of the SCL clock (ns)
 * @h_min: Min high period of the SCL clock (ns)
 */
struct stm32f7_i2c_spec {
	u32 rate;
	u32 fall_max;
	u32 rise_max;
	u32 hddat_min;
	u32 vddat_max;
	u32 sudat_min;
	u32 l_min;
	u32 h_min;
};

/**
 * struct stm32f7_i2c_setup - private I2C timing setup parameters
 * @speed_freq: I2C speed frequency  (Hz)
 * @clock_src: I2C clock source frequency (Hz)
 * @rise_time: Rise time (ns)
 * @fall_time: Fall time (ns)
 * @dnf: Digital filter coefficient (0-16)
 * @analog_filter: Analog filter delay (On/Off)
 * @fmp_clr_offset: Fast Mode Plus clear register offset from set register
 */
struct stm32f7_i2c_setup {
	u32 speed_freq;
	u32 clock_src;
	u32 rise_time;
	u32 fall_time;
	u8 dnf;
	bool analog_filter;
	u32 fmp_clr_offset;
};

/**
 * struct stm32f7_i2c_timings - private I2C output parameters
 * @node: List entry
 * @presc: Prescaler value
 * @scldel: Data setup time
 * @sdadel: Data hold time
 * @sclh: SCL high period (master mode)
 * @scll: SCL low period (master mode)
 */
struct stm32f7_i2c_timings {
	struct list_head node;
	u8 presc;
	u8 scldel;
	u8 sdadel;
	u8 sclh;
	u8 scll;
};

/**
 * struct stm32f7_i2c_msg - client specific data
 * @addr: 8-bit or 10-bit slave addr, including r/w bit
 * @count: number of bytes to be transferred
 * @buf: data buffer
 * @result: result of the transfer
 * @stop: last I2C msg to be sent, i.e. STOP to be generated
 * @smbus: boolean to know if the I2C IP is used in SMBus mode
 * @size: type of SMBus protocol
 * @read_write: direction of SMBus protocol
 * SMBus block read and SMBus block write - block read process call protocols
 * @smbus_buf: buffer to be used for SMBus protocol transfer. It will
 * contain a maximum of 32 bytes of data + byte command + byte count + PEC
 * This buffer has to be 32-bit aligned to be compliant with memory address
 * register in DMA mode.
 */
struct stm32f7_i2c_msg {
	u16 addr;
	u32 count;
	u8 *buf;
	int result;
	bool stop;
	// bool smbus;
	// int size;
	char read_write;
	// u8 smbus_buf[I2C_SMBUS_BLOCK_MAX + 3] __aligned(4);
};

/**
 * struct stm32f7_i2c_dev - private data of the controller
 * @adap: I2C adapter for this controller
 * @dev: device for this controller
 * @base: virtual memory area
 * @complete: completion of I2C message
 * @clk: hw i2c clock
 * @bus_rate: I2C clock frequency of the controller
 * @msg: Pointer to data to be written
 * @msg_num: number of I2C messages to be executed
 * @msg_id: message identifiant
 * @f7_msg: customized i2c msg for driver usage
 * @setup: I2C timing input setup
 * @timing: I2C computed timings
 * @slave: list of slave devices registered on the I2C bus
 * @slave_running: slave device currently used
 * @backup_regs: backup of i2c controller registers (for suspend/resume)
 * @slave_dir: transfer direction for the current slave device
 * @master_mode: boolean to know in which mode the I2C is running (master or
 * slave)
 * @dma: dma data
 * @use_dma: boolean to know if dma is used in the current transfer
 * @regmap: holds SYSCFG phandle for Fast Mode Plus bits
 * @fmp_sreg: register address for setting Fast Mode Plus bits
 * @fmp_creg: register address for clearing Fast Mode Plus bits
 * @fmp_mask: mask for Fast Mode Plus bits in set register
 * @wakeup_src: boolean to know if the device is a wakeup source
 * @smbus_mode: states that the controller is configured in SMBus mode
 * @host_notify_client: SMBus host-notify client
 */
struct stm32f7_i2c_dev {
	i2c_bus adap;
	rtems_interrupt_server_request thread_irq;
	struct drvmgr_dev *dev;
	void __iomem *base;
	rtems_id complete;
	struct drvmgr_dev *clk;
	int clkid;
	unsigned int bus_rate;
	struct i2c_msg *msg;
	unsigned int msg_num;
	unsigned int msg_id;
	struct stm32f7_i2c_msg f7_msg;
	struct stm32f7_i2c_setup setup;
	struct stm32f7_i2c_timings timing;
	int irq;
	int irq_err;
	
	rtems_binary_semaphore dma_complete;
	struct dma_chan *tx;
	struct dma_chan *rx;
	struct dma_chan *chan_using;
	bool use_dma;
	bool master_mode;
};

#define I2C_DEBUG
#ifdef I2C_DEBUG
#define devdbg(dev, fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define devdbg(...)
#endif

#define dev_info(...)
#define dev_warn dev_err
#define dev_err(dev, fmt, ...) \
	printk("%s: "fmt, (dev)->name, ##__VA_ARGS__)


#define NSEC_PER_SEC 1000000000ull

/*
 * All these values are coming from I2C Specification, Version 6.0, 4th of
 * April 2014.
 *
 * Table10. Characteristics of the SDA and SCL bus lines for Standard, Fast,
 * and Fast-mode Plus I2C-bus devices
 */
static struct stm32f7_i2c_spec stm32f7_i2c_specs[] = {
	{
		.rate = STM32_I2C_SPEED_STANDARD,
		.fall_max = 300,
		.rise_max = 1000,
		.hddat_min = 0,
		.vddat_max = 3450,
		.sudat_min = 250,
		.l_min = 4700,
		.h_min = 4000,
	},
	{
		.rate = STM32_I2C_SPEED_FAST,
		.fall_max = 300,
		.rise_max = 300,
		.hddat_min = 0,
		.vddat_max = 900,
		.sudat_min = 100,
		.l_min = 1300,
		.h_min = 600,
	},
	{
		.rate = STM32_I2C_SPEED_FAST_PLUS,
		.fall_max = 100,
		.rise_max = 120,
		.hddat_min = 0,
		.vddat_max = 450,
		.sudat_min = 50,
		.l_min = 500,
		.h_min = 260,
	},
};

static const struct stm32f7_i2c_setup stm32f7_setup = {
	.rise_time = STM32F7_I2C_RISE_TIME_DEFAULT,
	.fall_time = STM32F7_I2C_FALL_TIME_DEFAULT,
	.dnf = STM32F7_I2C_DNF_DEFAULT,
	.analog_filter = STM32F7_I2C_ANALOG_FILTER_ENABLE,
};


static inline u32 stm32f7_i2c_clkrate_get(struct stm32f7_i2c_dev *i2c_dev)
{
	u32 clksrc;

	switch (i2c_dev->clkid) {
	case I2C1_CK:
	case I2C2_CK:
	case I2C3_CK:
		clksrc = LL_RCC_I2C123_CLKSOURCE;
		break;
	case I2C4_CK:
		clksrc = LL_RCC_I2C4_CLKSOURCE;
		break;
	default:
		return 0;
	}
	return LL_RCC_GetI2CClockFreq(clksrc);
}

static inline bool stm32f7_i2c_use_dma(struct stm32f7_i2c_dev *i2c_dev, size_t transfer_size) 
{
	if (i2c_dev->tx && 
		i2c_dev->rx && 
		transfer_size > STM32F7_I2C_DMA_LEN_MIN)
		return true;
	return false;
}

static inline void stm32f7_i2c_set_bits(void __iomem *reg, u32 mask)
{
	writel_relaxed(readl_relaxed(reg) | mask, reg);
}

static inline void stm32f7_i2c_clr_bits(void __iomem *reg, u32 mask)
{
	writel_relaxed(readl_relaxed(reg) & ~mask, reg);
}

static void stm32f7_i2c_disable_irq(struct stm32f7_i2c_dev *i2c_dev, u32 mask)
{
	stm32f7_i2c_clr_bits(i2c_dev->base + STM32F7_I2C_CR1, mask);
}

static struct stm32f7_i2c_spec *stm32f7_get_specs(u32 rate)
{
	for (size_t i = 0; i < RTEMS_ARRAY_SIZE(stm32f7_i2c_specs); i++)
		if (rate <= stm32f7_i2c_specs[i].rate)
			return &stm32f7_i2c_specs[i];

	return NULL;
}

static void i2c_parse_timing(phandle_t node, char *prop_name, u32 *cur_val_p,
	u32 def_val, bool use_def) 
{
	pcell_t val;
	if (rtems_ofw_get_enc_prop(node, prop_name, &val, sizeof(val)) > 0)
		*cur_val_p = (u32)val;
	else if (use_def)
		*cur_val_p = def_val;
}

static void i2c_parse_fw_timings(struct drvmgr_dev *dev, struct i2c_timings *t, bool use_defaults)
{
	struct dev_private *devp = device_get_private(dev);
	phandle_t node = devp->np;
	bool u = use_defaults;
	u32 d;

	i2c_parse_timing(node, "clock-frequency", &t->bus_freq_hz,
			 STM32_I2C_SPEED_STANDARD, u);

	d = t->bus_freq_hz <= STM32_I2C_SPEED_STANDARD ? 1000 :
	    t->bus_freq_hz <= STM32_I2C_SPEED_FAST ? 300 : 120;
	i2c_parse_timing(node, "i2c-scl-rising-time-ns", &t->scl_rise_ns, d, u);

	d = t->bus_freq_hz <= STM32_I2C_SPEED_FAST ? 300 : 120;
	i2c_parse_timing(node, "i2c-scl-falling-time-ns", &t->scl_fall_ns, d, u);

	i2c_parse_timing(node, "i2c-scl-internal-delay-ns",
			 &t->scl_int_delay_ns, 0, u);
	i2c_parse_timing(node, "i2c-sda-falling-time-ns", &t->sda_fall_ns,
			 t->scl_fall_ns, u);
	i2c_parse_timing(node, "i2c-sda-hold-time-ns", &t->sda_hold_ns, 0, u);
	i2c_parse_timing(node, "i2c-digital-filter-width-ns",
			 &t->digital_filter_width_ns, 0, u);
	i2c_parse_timing(node, "i2c-analog-filter-cutoff-frequency",
			 &t->analog_filter_cutoff_freq_hz, 0, u);
}

#define	RATE_MIN(rate)	((rate) * 8 / 10)
static int stm32f7_i2c_compute_timing(struct stm32f7_i2c_dev *i2c_dev,
				      struct stm32f7_i2c_setup *setup,
				      struct stm32f7_i2c_timings *output)
{
	struct stm32f7_i2c_spec *specs;
	u32 p_prev = STM32F7_PRESC_MAX;
	u32 i2cclk = DIV_ROUND_CLOSEST(NSEC_PER_SEC,
				       setup->clock_src);
	u32 i2cbus = DIV_ROUND_CLOSEST(NSEC_PER_SEC,
				       setup->speed_freq);
	u32 clk_error_prev = i2cbus;
	u32 tsync;
	u32 af_delay_min, af_delay_max;
	u32 dnf_delay;
	u32 clk_min, clk_max;
	int sdadel_min, sdadel_max;
	int scldel_min;
	struct stm32f7_i2c_timings *v, *_v, *s;
	struct list_head solutions;
	u16 p, l, a, h;
	int ret = 0;

	specs = stm32f7_get_specs(setup->speed_freq);
	if (specs == NULL) {
		dev_err(i2c_dev->dev, "speed out of bound {%d}\n",
			setup->speed_freq);
		return -EINVAL;
	}

	if ((setup->rise_time > specs->rise_max) ||
	    (setup->fall_time > specs->fall_max)) {
		dev_err(i2c_dev->dev,
			"timings out of bound Rise{%d>%d}/Fall{%d>%d}\n",
			setup->rise_time, specs->rise_max,
			setup->fall_time, specs->fall_max);
		return -EINVAL;
	}

	if (setup->dnf > STM32F7_I2C_DNF_MAX) {
		dev_err(i2c_dev->dev,
			"DNF out of bound %d/%d\n",
			setup->dnf, STM32F7_I2C_DNF_MAX);
		return -EINVAL;
	}

	/*  Analog and Digital Filters */
	af_delay_min =
		(setup->analog_filter ?
		 STM32F7_I2C_ANALOG_FILTER_DELAY_MIN : 0);
	af_delay_max =
		(setup->analog_filter ?
		 STM32F7_I2C_ANALOG_FILTER_DELAY_MAX : 0);
	dnf_delay = setup->dnf * i2cclk;

	sdadel_min = specs->hddat_min + setup->fall_time -
		af_delay_min - (setup->dnf + 3) * i2cclk;

	sdadel_max = specs->vddat_max - setup->rise_time -
		af_delay_max - (setup->dnf + 4) * i2cclk;

	scldel_min = setup->rise_time + specs->sudat_min;

	if (sdadel_min < 0)
		sdadel_min = 0;
	if (sdadel_max < 0)
		sdadel_max = 0;

	devdbg(i2c_dev->dev, "SDADEL(min/max): %i/%i, SCLDEL(Min): %i\n",
		sdadel_min, sdadel_max, scldel_min);

	INIT_LIST_HEAD(&solutions);
	/* Compute possible values for PRESC, SCLDEL and SDADEL */
	for (p = 0; p < STM32F7_PRESC_MAX; p++) {
		for (l = 0; l < STM32F7_SCLDEL_MAX; l++) {
			u32 scldel = (l + 1) * (p + 1) * i2cclk;

			if (scldel < (u32)scldel_min)
				continue;

			for (a = 0; a < STM32F7_SDADEL_MAX; a++) {
				u32 sdadel = (a * (p + 1) + 1) * i2cclk;

				if (((sdadel >= (u32)sdadel_min) &&
				     (sdadel <= (u32)sdadel_max)) &&
				    (p != p_prev)) {
					v = rtems_malloc(sizeof(*v));
					if (!v) {
						ret = -ENOMEM;
						goto exit;
					}

					v->presc = p;
					v->scldel = l;
					v->sdadel = a;
					p_prev = p;

					list_add_tail(&v->node, &solutions);  
					break;
				}
			}

			if (p_prev == p)
				break;
		}
	}

	if (list_empty(&solutions)) {
		dev_err(i2c_dev->dev, "no Prescaler solution\n");
		ret = -EPERM;
		goto exit;
	}

	tsync = af_delay_min + dnf_delay + (2 * i2cclk);
	s = NULL;
	clk_max = NSEC_PER_SEC / RATE_MIN(setup->speed_freq);
	clk_min = NSEC_PER_SEC / setup->speed_freq;

	/*
	 * Among Prescaler possibilities discovered above figures out SCL Low
	 * and High Period. Provided:
	 * - SCL Low Period has to be higher than SCL Clock Low Period
	 *   defined by I2C Specification. I2C Clock has to be lower than
	 *   (SCL Low Period - Analog/Digital filters) / 4.
	 * - SCL High Period has to be lower than SCL Clock High Period
	 *   defined by I2C Specification
	 * - I2C Clock has to be lower than SCL High Period
	 */
	list_for_each_entry(v, &solutions, node) {
		u32 prescaler = (v->presc + 1) * i2cclk;

		for (l = 0; l < STM32F7_SCLL_MAX; l++) {
			u32 tscl_l = (l + 1) * prescaler + tsync;

			if ((tscl_l < specs->l_min) ||
			    (i2cclk >=
			     ((tscl_l - af_delay_min - dnf_delay) / 4))) {
				continue;
			}

			for (h = 0; h < STM32F7_SCLH_MAX; h++) {
				u32 tscl_h = (h + 1) * prescaler + tsync;
				u32 tscl = tscl_l + tscl_h +
					setup->rise_time + setup->fall_time;

				if ((tscl >= clk_min) && (tscl <= clk_max) &&
				    (tscl_h >= specs->h_min) &&
				    (i2cclk < tscl_h)) {
					int clk_error = tscl - i2cbus;

					if (clk_error < 0)
						clk_error = -clk_error;

					if (clk_error < (int)clk_error_prev) {
						clk_error_prev = clk_error;
						v->scll = l;
						v->sclh = h;
						s = v;
					}
				}
			}
		}
	}

	if (!s) {
		dev_err(i2c_dev->dev, "no solution at all\n");
		ret = -EPERM;
		goto exit;
	}

	output->presc = s->presc;
	output->scldel = s->scldel;
	output->sdadel = s->sdadel;
	output->scll = s->scll;
	output->sclh = s->sclh;

	devdbg(i2c_dev->dev,
		"Presc: %i, scldel: %i, sdadel: %i, scll: %i, sclh: %i\n",
		output->presc,
		output->scldel, output->sdadel,
		output->scll, output->sclh);

exit:
	/* Release list and memory */
	list_for_each_entry_safe(v, _v, &solutions, node) {
		list_del(&v->node);
		free(v);
	}

	return ret;
}

static u32 stm32f7_get_lower_rate(u32 rate)
{
	int i = RTEMS_ARRAY_SIZE(stm32f7_i2c_specs);

	while (--i)
		if (stm32f7_i2c_specs[i].rate < rate)
			break;

	return stm32f7_i2c_specs[i].rate;
}

static int stm32f7_i2c_setup_timing(struct stm32f7_i2c_dev *i2c_dev,
	struct stm32f7_i2c_setup *setup, u32 freq_hz)
{
	struct i2c_timings timings, *t = &timings;
	int ret = 0;

	t->bus_freq_hz = freq_hz; //STM32_I2C_SPEED_STANDARD;
	t->scl_rise_ns = i2c_dev->setup.rise_time;
	t->scl_fall_ns = i2c_dev->setup.fall_time;

	i2c_parse_fw_timings(i2c_dev->dev, t, false);
	if (t->bus_freq_hz > STM32_I2C_SPEED_FAST_PLUS) {
		dev_err(i2c_dev->dev, "Invalid bus speed (%i>%i)\n",
			t->bus_freq_hz, STM32_I2C_SPEED_FAST_PLUS);
		return -EINVAL;
	}

	setup->speed_freq = t->bus_freq_hz;
	i2c_dev->setup.rise_time = t->scl_rise_ns;
	i2c_dev->setup.fall_time = t->scl_fall_ns;
	
	setup->clock_src = stm32f7_i2c_clkrate_get(i2c_dev);
	if (!setup->clock_src) {
		dev_err(i2c_dev->dev, "clock rate is 0\n");
		return -EINVAL;
	}

	do {
		ret = stm32f7_i2c_compute_timing(i2c_dev, setup,
						 &i2c_dev->timing);
		if (ret) {
			dev_err(i2c_dev->dev,
				"failed to compute I2C timings.\n");
			if (setup->speed_freq <= STM32_I2C_SPEED_STANDARD)
				break;
			setup->speed_freq = stm32f7_get_lower_rate(setup->speed_freq);
			dev_warn(i2c_dev->dev, "downgrade I2C Speed Freq to (%i)\n", setup->speed_freq);
		}
	} while (ret);

	if (ret) {
		dev_err(i2c_dev->dev, "Impossible to compute I2C timings.\n");
		return ret;
	}

	devdbg(i2c_dev->dev, "I2C Speed(%i), Clk Source(%i)\n",
		setup->speed_freq, setup->clock_src);
	devdbg(i2c_dev->dev, "I2C Rise(%i) and Fall(%i) Time\n",
		setup->rise_time, setup->fall_time);
	devdbg(i2c_dev->dev, "I2C Analog Filter(%s), DNF(%i)\n",
		(setup->analog_filter ? "On" : "Off"), setup->dnf);

	i2c_dev->bus_rate = setup->speed_freq;

	return 0;
}

static void stm32f7_i2c_disable_dma_req(struct stm32f7_i2c_dev *i2c_dev)
{
	void __iomem *base = i2c_dev->base;
	u32 mask = STM32F7_I2C_CR1_RXDMAEN | STM32F7_I2C_CR1_TXDMAEN;

	stm32f7_i2c_clr_bits(base + STM32F7_I2C_CR1, mask);
}

static void stm32f7_i2c_hw_config(struct stm32f7_i2c_dev *i2c_dev)
{
	struct stm32f7_i2c_timings *t = &i2c_dev->timing;
	u32 timing = 0;

	/* Timing settings */
	timing |= STM32F7_I2C_TIMINGR_PRESC(t->presc);
	timing |= STM32F7_I2C_TIMINGR_SCLDEL(t->scldel);
	timing |= STM32F7_I2C_TIMINGR_SDADEL(t->sdadel);
	timing |= STM32F7_I2C_TIMINGR_SCLH(t->sclh);
	timing |= STM32F7_I2C_TIMINGR_SCLL(t->scll);
	writel_relaxed(timing, i2c_dev->base + STM32F7_I2C_TIMINGR);

	/* Enable I2C */
	if (i2c_dev->setup.analog_filter)
		stm32f7_i2c_clr_bits(i2c_dev->base + STM32F7_I2C_CR1,
				     STM32F7_I2C_CR1_ANFOFF);
	else
		stm32f7_i2c_set_bits(i2c_dev->base + STM32F7_I2C_CR1,
				     STM32F7_I2C_CR1_ANFOFF);
	stm32f7_i2c_set_bits(i2c_dev->base + STM32F7_I2C_CR1,
			     STM32F7_I2C_CR1_PE);
}

static void stm32f7_i2c_write_tx_data(struct stm32f7_i2c_dev *i2c_dev)
{
	struct stm32f7_i2c_msg *f7_msg = &i2c_dev->f7_msg;
	void __iomem *base = i2c_dev->base;

	if (f7_msg->count) {
		writeb_relaxed(*f7_msg->buf++, base + STM32F7_I2C_TXDR);
		f7_msg->count--;
	}
}

static void stm32f7_i2c_read_rx_data(struct stm32f7_i2c_dev *i2c_dev)
{
	struct stm32f7_i2c_msg *f7_msg = &i2c_dev->f7_msg;
	void __iomem *base = i2c_dev->base;

	if (f7_msg->count) {
		*f7_msg->buf++ = readb_relaxed(base + STM32F7_I2C_RXDR);
		f7_msg->count--;
	} else {
		/* Flush RX buffer has no data is expected */
		readb_relaxed(base + STM32F7_I2C_RXDR);
	}
}

static void stm32f7_i2c_reload(struct stm32f7_i2c_dev *i2c_dev)
{
	struct stm32f7_i2c_msg *f7_msg = &i2c_dev->f7_msg;
	u32 cr2;

	if (i2c_dev->use_dma)
		f7_msg->count -= STM32F7_I2C_MAX_LEN;

	cr2 = readl_relaxed(i2c_dev->base + STM32F7_I2C_CR2);

	cr2 &= ~STM32F7_I2C_CR2_NBYTES_MASK;
	if (f7_msg->count > STM32F7_I2C_MAX_LEN) {
		cr2 |= STM32F7_I2C_CR2_NBYTES(STM32F7_I2C_MAX_LEN);
	} else {
		cr2 &= ~STM32F7_I2C_CR2_RELOAD;
		cr2 |= STM32F7_I2C_CR2_NBYTES(f7_msg->count);
	}

	writel_relaxed(cr2, i2c_dev->base + STM32F7_I2C_CR2);
}

static int stm32f7_i2c_release_bus(i2c_bus *i2c_adap)
{
	struct stm32f7_i2c_dev *i2c_dev = (struct stm32f7_i2c_dev *)i2c_adap;

	// dev_info(i2c_dev->dev, "Trying to recover bus\n");

	stm32f7_i2c_clr_bits(i2c_dev->base + STM32F7_I2C_CR1,
			     STM32F7_I2C_CR1_PE);

	stm32f7_i2c_hw_config(i2c_dev);

	return 0;
}

static int stm32f7_i2c_wait_free_bus(struct stm32f7_i2c_dev *i2c_dev)
{
	u32 status;
	int ret, retry = 3;

	status = readl_relaxed(i2c_dev->base + STM32F7_I2C_ISR);
	while (!(status & STM32F7_I2C_ISR_BUSY)) {
		if (retry == 0)
			goto _err;
		rtems_task_wake_after(1);
		status = readl_relaxed(i2c_dev->base + STM32F7_I2C_ISR);
		retry--;
	}
	// ret = readl_relaxed_poll_timeout(i2c_dev->base + STM32F7_I2C_ISR,
	// 				 status,
	// 				 !(status & STM32F7_I2C_ISR_BUSY),
	// 				 10, 1000);

	return 0;
_err:
	dev_info(i2c_dev->dev, "bus busy\n");
	ret = stm32f7_i2c_release_bus(&i2c_dev->adap);
	if (ret) {
		dev_err(i2c_dev->dev, "Failed to recover the bus (%d)\n", ret);
		return ret;
	}

	return -EBUSY;
}

static int stm32f7_i2c_dma_transfer(struct stm32f7_i2c_dev *i2c_dev, void *buf, 
	size_t size, int rd) 
{
	struct dma_chan *chan;
	dma_addr_t src, dst;

	if (rd) {
		chan = i2c_dev->rx;
		src = (dma_addr_t)(i2c_dev->base + STM32F7_I2C_RXDR);
		dst = (dma_addr_t)buf;
	} else {
		chan = i2c_dev->tx;
		src = (dma_addr_t)buf;
		dst = (dma_addr_t)(i2c_dev->base + STM32F7_I2C_TXDR);
	}

	i2c_dev->chan_using = chan;
	return dma_chan_reload(chan, src, dst, size);
}

static void stm32f7_i2c_xfer_msg(struct stm32f7_i2c_dev *i2c_dev,
				 struct i2c_msg *msg)
{
	struct stm32f7_i2c_msg *f7_msg = &i2c_dev->f7_msg;
	void __iomem *base = i2c_dev->base;
	u32 cr1, cr2;
	int ret;

	f7_msg->addr = msg->addr;
	f7_msg->buf = msg->buf;
	f7_msg->count = msg->len;
	f7_msg->result = 0;
	f7_msg->stop = (i2c_dev->msg_id >= i2c_dev->msg_num - 1);

	// reinit_completion(&i2c_dev->complete);

	cr1 = readl_relaxed(base + STM32F7_I2C_CR1);
	cr2 = readl_relaxed(base + STM32F7_I2C_CR2);

	/* Set transfer direction */
	cr2 &= ~STM32F7_I2C_CR2_RD_WRN;
	if (msg->flags & I2C_M_RD)
		cr2 |= STM32F7_I2C_CR2_RD_WRN;

	/* Set slave address */
	cr2 &= ~(STM32F7_I2C_CR2_HEAD10R | STM32F7_I2C_CR2_ADD10);
	if (msg->flags & I2C_M_TEN) {
		cr2 &= ~STM32F7_I2C_CR2_SADD10_MASK;
		cr2 |= STM32F7_I2C_CR2_SADD10(f7_msg->addr);
		cr2 |= STM32F7_I2C_CR2_ADD10;
	} else {
		cr2 &= ~STM32F7_I2C_CR2_SADD7_MASK;
		cr2 |= STM32F7_I2C_CR2_SADD7(f7_msg->addr);
	}

	/* Set nb bytes to transfer and reload if needed */
	cr2 &= ~(STM32F7_I2C_CR2_NBYTES_MASK | STM32F7_I2C_CR2_RELOAD);
	if (f7_msg->count > STM32F7_I2C_MAX_LEN) {
		cr2 |= STM32F7_I2C_CR2_NBYTES(STM32F7_I2C_MAX_LEN);
		cr2 |= STM32F7_I2C_CR2_RELOAD;
	} else {
		cr2 |= STM32F7_I2C_CR2_NBYTES(f7_msg->count);
	}

	/* Enable NACK, STOP, error and transfer complete interrupts */
	cr1 |= STM32F7_I2C_CR1_ERRIE | STM32F7_I2C_CR1_TCIE |
		STM32F7_I2C_CR1_STOPIE | STM32F7_I2C_CR1_NACKIE;

	/* Clear DMA req and TX/RX interrupt */
	cr1 &= ~(STM32F7_I2C_CR1_RXIE | STM32F7_I2C_CR1_TXIE |
			STM32F7_I2C_CR1_RXDMAEN | STM32F7_I2C_CR1_TXDMAEN);

	/* Configure DMA or enable RX/TX interrupt */
	i2c_dev->use_dma = false;
	if (stm32f7_i2c_use_dma(i2c_dev, f7_msg->count)) {
		ret = stm32f7_i2c_dma_transfer(i2c_dev, f7_msg->buf, f7_msg->count, 
			msg->flags & I2C_M_RD);
		if (!ret)
			i2c_dev->use_dma = true;
		else
			dev_warn(i2c_dev->dev, "can't use DMA\n");
	}

	if (!i2c_dev->use_dma) {
		if (msg->flags & I2C_M_RD)
			cr1 |= STM32F7_I2C_CR1_RXIE;
		else
			cr1 |= STM32F7_I2C_CR1_TXIE;
	} else {
		if (msg->flags & I2C_M_RD)
			cr1 |= STM32F7_I2C_CR1_RXDMAEN;
		else
			cr1 |= STM32F7_I2C_CR1_TXDMAEN;
	}

	/* Configure Start/Repeated Start */
	cr2 |= STM32F7_I2C_CR2_START;

	i2c_dev->master_mode = true;

	/* Write configurations registers */
	writel_relaxed(cr1, base + STM32F7_I2C_CR1);
	writel_relaxed(cr2, base + STM32F7_I2C_CR2);
}

static void __isr stm32f7_i2c_isr_event(void *data)
{
	struct stm32f7_i2c_dev *i2c_dev = data;
	struct stm32f7_i2c_msg *f7_msg = &i2c_dev->f7_msg;
	void __iomem *base = i2c_dev->base;
	u32 status, mask;
	bool wakeup_tsr = false;
	// int ret = IRQ_HANDLED;

	/* Check if the interrupt if for a slave device */
	// if (!i2c_dev->master_mode) {
	// 	ret = stm32f7_i2c_slave_isr_event(i2c_dev);
	// 	return ret;
	// }

	status = readl_relaxed(i2c_dev->base + STM32F7_I2C_ISR);

	/* Tx empty */
	if (status & STM32F7_I2C_ISR_TXIS)
		stm32f7_i2c_write_tx_data(i2c_dev);

	/* RX not empty */
	if (status & STM32F7_I2C_ISR_RXNE)
		stm32f7_i2c_read_rx_data(i2c_dev);

	/* NACK received */
	if (status & STM32F7_I2C_ISR_NACKF) {
		devdbg(i2c_dev->dev, "<%s>: Receive NACK (addr %x)\n",
			__func__, f7_msg->addr);
		writel_relaxed(STM32F7_I2C_ICR_NACKCF, base + STM32F7_I2C_ICR);
		f7_msg->result = -ENXIO;
	}

	/* STOP detection flag */
	if (status & STM32F7_I2C_ISR_STOPF) {
		/* Disable interrupts */
		mask = STM32F7_I2C_ALL_IRQ_MASK;
		stm32f7_i2c_disable_irq(i2c_dev, mask);

		/* Clear STOP flag */
		writel_relaxed(STM32F7_I2C_ICR_STOPCF, base + STM32F7_I2C_ICR);

		if (i2c_dev->use_dma) {
			wakeup_tsr = true;
			// ret = IRQ_WAKE_THREAD;
		} else {
			i2c_dev->master_mode = false;
			rtems_event_transient_send(i2c_dev->complete);
			// complete(&i2c_dev->complete);
		}
	}

	/* Transfer complete */
	if (status & STM32F7_I2C_ISR_TC) {
		if (f7_msg->stop) {
			mask = STM32F7_I2C_CR2_STOP;
			stm32f7_i2c_set_bits(base + STM32F7_I2C_CR2, mask);
		} else if (i2c_dev->use_dma) {
			wakeup_tsr = true;
			// ret = IRQ_WAKE_THREAD;
		} else {
			i2c_dev->msg_id++;
			i2c_dev->msg++;
			stm32f7_i2c_xfer_msg(i2c_dev, i2c_dev->msg);
		}
	}

	if (status & STM32F7_I2C_ISR_TCR)
		stm32f7_i2c_reload(i2c_dev);

	if (wakeup_tsr) 
		rtems_interrupt_server_request_submit(&i2c_dev->thread_irq);
}

static void __isr stm32f7_i2c_isr_event_thread(void *data)
{
	struct stm32f7_i2c_dev *i2c_dev = data;
	struct stm32f7_i2c_msg *f7_msg = &i2c_dev->f7_msg;
	// struct stm32_i2c_dma *dma = i2c_dev->dma;
	u32 status;
	int ret;

	/*
	 * Wait for dma transfer completion before sending next message or
	 * notity the end of xfer to the client
	 */
	ret = rtems_binary_semaphore_wait_timed_ticks(&i2c_dev->dma_complete, HZ);
	// ret = wait_for_completion_timeout(&i2c_dev->dma->dma_complete, HZ);
	if (ret) {
		devdbg(i2c_dev->dev, "<%s>: Timed out\n", __func__);
		stm32f7_i2c_disable_dma_req(i2c_dev);
		// dmaengine_terminate_all(dma->chan_using);
		f7_msg->result = -ETIMEDOUT;
	}

	status = readl_relaxed(i2c_dev->base + STM32F7_I2C_ISR);

	if (status & STM32F7_I2C_ISR_TC) {
		i2c_dev->msg_id++;
		i2c_dev->msg++;
		stm32f7_i2c_xfer_msg(i2c_dev, i2c_dev->msg);
	} else {
		i2c_dev->master_mode = false;
		rtems_event_transient_send(i2c_dev->complete);
		// complete(&i2c_dev->complete);
	}
}

static void __isr stm32f7_i2c_isr_error(void *data)
{
	struct stm32f7_i2c_dev *i2c_dev = data;
	struct stm32f7_i2c_msg *f7_msg = &i2c_dev->f7_msg;
	void __iomem *base = i2c_dev->base;
	struct drvmgr_dev *dev = i2c_dev->dev;
	// struct stm32_i2c_dma *dma = i2c_dev->dma;
	u32 status;

	status = readl_relaxed(i2c_dev->base + STM32F7_I2C_ISR);

	/* Bus error */
	if (status & STM32F7_I2C_ISR_BERR) {
		dev_err(dev, "<%s>: Bus error\n", __func__);
		writel_relaxed(STM32F7_I2C_ICR_BERRCF, base + STM32F7_I2C_ICR);
		stm32f7_i2c_release_bus(&i2c_dev->adap);
		f7_msg->result = -EIO;
	}

	/* Arbitration loss */
	if (status & STM32F7_I2C_ISR_ARLO) {
		devdbg(dev, "<%s>: Arbitration loss\n", __func__);
		writel_relaxed(STM32F7_I2C_ICR_ARLOCF, base + STM32F7_I2C_ICR);
		f7_msg->result = -EAGAIN;
	}

	if (status & STM32F7_I2C_ISR_PECERR) {
		dev_err(dev, "<%s>: PEC error in reception\n", __func__);
		writel_relaxed(STM32F7_I2C_ICR_PECCF, base + STM32F7_I2C_ICR);
		f7_msg->result = -EINVAL;
	}

	/*if (!i2c_dev->slave_running) */{
		u32 mask;
		/* Disable interrupts */
		// if (stm32f7_i2c_is_slave_registered(i2c_dev))
		// 	mask = STM32F7_I2C_XFER_IRQ_MASK;
		// else
			mask = STM32F7_I2C_ALL_IRQ_MASK;
		stm32f7_i2c_disable_irq(i2c_dev, mask);
	}

	/* Disable dma */
	if (i2c_dev->use_dma) {
		stm32f7_i2c_disable_dma_req(i2c_dev);
		// dmaengine_terminate_all(dma->chan_using);
	}

	// i2c_dev->master_mode = false;
	rtems_event_transient_send(i2c_dev->complete); //complete(&i2c_dev->complete);
}


static void stm32f7_i2c_dma_callback(void *arg)
{
	struct stm32f7_i2c_dev *i2c_dev = (struct stm32f7_i2c_dev *)arg;
	// struct stm32_i2c_dma *dma = i2c_dev->dma;
	// struct device *dev = dma->chan_using->device->dev;

	stm32f7_i2c_disable_dma_req(i2c_dev);
	// dma_unmap_single(dev, dma->dma_buf, dma->dma_len, dma->dma_data_dir);
	rtems_binary_semaphore_post(&i2c_dev->dma_complete);
	// complete(&i2c_dev->dma_complete);
}

static void __isr stm32h7_i2c_dma_isr(struct drvmgr_dev *dev, void *arg, 
	uint32_t channel, int status) 
{
    struct stm32f7_i2c_dev *i2c_dev = (struct stm32f7_i2c_dev *)arg;

	stm32f7_i2c_disable_dma_req(i2c_dev);
	rtems_binary_semaphore_post(&i2c_dev->dma_complete);
    (void) dev;
    (void) channel;
	(void) status;
}

static void stm32f7_i2c_dma_open(struct stm32f7_i2c_dev *i2c_dev) 
{
    int ret;
    if (i2c_dev->tx) {
        struct dma_config *config = &i2c_dev->tx->config;
        struct dma_chan *tx = i2c_dev->tx;
        struct dma_block_config txblk = {0};

        config->channel_direction = MEMORY_TO_PERIPHERAL;
        config->dest_data_size = 1;
        config->dest_burst_length = 1;
        config->source_burst_length = 1;
        config->source_data_size = 1;
        config->channel_priority = 0;
        config->dma_callback = stm32h7_i2c_dma_isr;
        config->user_data = i2c_dev;
        config->head_block = &txblk;
        config->block_count = 1;
        txblk.block_size = 1;
        txblk.source_address = (dma_addr_t)0;
        txblk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
        txblk.dest_address = (dma_addr_t)(i2c_dev->base + STM32F7_I2C_TXDR);
        txblk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
        ret = dma_configure(tx->dev, tx->channel, &tx->config);
        if (ret) {
            printk("Configure UART(%s) DMA-TX failed: %d\n", i2c_dev->dev->name, ret);
			dma_chan_release(i2c_dev->tx);
            i2c_dev->tx = NULL;
        }
    }
    if (i2c_dev->rx) {
        struct dma_config *config = &i2c_dev->rx->config;
        struct dma_chan *rx = i2c_dev->rx;
        struct dma_block_config rxblk = {0};

        config->channel_direction = PERIPHERAL_TO_MEMORY;
        config->dest_data_size = 1;
        config->dest_burst_length = 1;
        config->source_burst_length = 1;
        config->source_data_size = 1;
        config->complete_callback_en = 1;
        config->channel_priority = 1;
		config->dma_callback = stm32h7_i2c_dma_isr;
        config->user_data = i2c_dev;
        config->head_block = &rxblk;
        config->block_count = 1;
        rxblk.block_size = 1;
        rxblk.source_address = (dma_addr_t)(i2c_dev->base + STM32F7_I2C_RXDR);
        rxblk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
        rxblk.dest_address = (dma_addr_t)0;
        rxblk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
        ret = dma_configure(rx->dev, rx->channel, &rx->config);
        if (ret) {
            printk("Configure UART(%s) DMA-RX failed: %d\n", i2c_dev->dev->name, ret);
			dma_chan_stop(i2c_dev->rx);
			dma_chan_release(i2c_dev->rx);
            i2c_dev->rx = NULL;
            return;
        }
    }
}

static void stm32f7_i2c_dma_close(struct stm32f7_i2c_dev *i2c_dev) 
{
    if (i2c_dev->rx)
		dma_chan_stop(i2c_dev->rx);
    if (i2c_dev->tx)
        dma_chan_stop(i2c_dev->tx);
}

static int __fastcode stm32f7_i2c_xfer(i2c_bus *i2c_adap,
			    i2c_msg msgs[], u32 num)
{
	struct stm32f7_i2c_dev *i2c_dev = (struct stm32f7_i2c_dev *)i2c_adap;
	struct stm32f7_i2c_msg *f7_msg = &i2c_dev->f7_msg;
	rtems_status_code sc;
	int ret;

	i2c_dev->msg = msgs;
	i2c_dev->msg_num = num;
	i2c_dev->msg_id = 0;
	// f7_msg->smbus = false;

	// ret = pm_runtime_get_sync(i2c_dev->dev);
	// if (ret < 0)
	// 	return ret;

	ret = stm32f7_i2c_wait_free_bus(i2c_dev);
	if (ret)
		goto pm_free;

	i2c_dev->complete = rtems_task_self();
	stm32f7_i2c_xfer_msg(i2c_dev, msgs);

	sc = rtems_event_transient_receive(RTEMS_WAIT|RTEMS_EVENT_ALL, 
		i2c_dev->adap.timeout);
	// time_left = wait_for_completion_timeout(&i2c_dev->complete,
	// 					i2c_dev->adap.timeout);
	ret = f7_msg->result;
	if (sc != RTEMS_SUCCESSFUL) {
		devdbg(i2c_dev->dev, "Access to slave 0x%x timed out\n", i2c_dev->msg->addr);
		if (i2c_dev->use_dma)
			dma_chan_stop(i2c_dev->chan_using);
		ret = -ETIMEDOUT;
	}

pm_free:
	// pm_runtime_mark_last_busy(i2c_dev->dev);
	// pm_runtime_put_autosuspend(i2c_dev->dev);
	return (ret < 0) ? ret : 0;
}

static int stm32f7_i2c_interrupt_install(struct stm32f7_i2c_dev *i2c_dev, u32 irq_svr) {
	int err;
	rtems_interrupt_server_request_initialize(irq_svr, &i2c_dev->thread_irq, 
		stm32f7_i2c_isr_event_thread, i2c_dev);
	rtems_interrupt_server_request_set_vector(&i2c_dev->thread_irq, i2c_dev->irq);
    err = drvmgr_interrupt_register(i2c_dev->dev, IRQF_HARD(i2c_dev->irq), 
		i2c_dev->dev->name, stm32f7_i2c_isr_event, i2c_dev);
	if (!err) {
		err = drvmgr_interrupt_register(i2c_dev->dev, IRQF_HARD(i2c_dev->irq_err), 
			i2c_dev->dev->name, stm32f7_i2c_isr_error, i2c_dev);
		if (err) {
			drvmgr_interrupt_unregister(i2c_dev->dev, IRQF_HARD(i2c_dev->irq), 
				stm32f7_i2c_isr_event, i2c_dev);
		}
	}
	return err;
}

static void stm32f7_i2c_interrupt_uninstall(struct stm32f7_i2c_dev *i2c_dev) {
	drvmgr_interrupt_unregister(i2c_dev->dev, IRQF_HARD(i2c_dev->irq), 
		stm32f7_i2c_isr_event, i2c_dev);
	drvmgr_interrupt_unregister(i2c_dev->dev, IRQF_HARD(i2c_dev->irq_err), 
		stm32f7_i2c_isr_error, i2c_dev);
}

static int stm32f7_i2c_set_clock(i2c_bus *bus, unsigned long hz) 
{
	struct stm32f7_i2c_dev *i2c_dev = (struct stm32f7_i2c_dev *)bus;
	rtems_interrupt_level level;
	int err;

	err = stm32f7_i2c_setup_timing(i2c_dev, &i2c_dev->setup, hz);
	if (!err) {
		rtems_interrupt_local_disable(level);
		stm32f7_i2c_hw_config(i2c_dev);
		rtems_interrupt_local_enable(level);
	}
	return err;
}

static void stm32f7_i2c_destroy(i2c_bus *bus) 
{
	struct stm32f7_i2c_dev *i2c_dev = (struct stm32f7_i2c_dev *)bus;

	clk_disable(i2c_dev->clk, &i2c_dev->clkid);
	stm32f7_i2c_interrupt_uninstall(i2c_dev);
	i2c_bus_destroy_and_free(bus);
}

static int stm32f7_i2c_bus_unite(struct drvmgr_drv *drv, struct drvmgr_dev *dev) {
	return ofw_platform_bus_match(drv, dev, DRVMGR_BUS_TYPE_I2C);
}

static struct drvmgr_bus_ops stm32h7_i2c_bus = {
	.init = {
		ofw_platform_bus_populate_device,
	},
	.unite = stm32f7_i2c_bus_unite
};

static int stm32f7_i2c_preprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
	struct stm32f7_i2c_dev *i2c_dev;
    rtems_ofw_memory_area reg;
    rtems_vector_number irqs[2];
    int ret;

	ret = rtems_ofw_get_reg(devp->np, &reg, sizeof(reg));
	if (ret < 0) 
		return -ENOSTR;
	ret = rtems_ofw_get_interrupts(devp->np, irqs, sizeof(irqs));
	if (ret < 0) 
		return -ENOSTR;
	i2c_dev = rtems_calloc(1, sizeof(struct stm32f7_i2c_dev));
	if (!i2c_dev)
		return -ENOMEM;

	dev->priv = i2c_dev;
	devp->devops = &i2c_dev->adap;
	i2c_dev->dev = dev;
	i2c_dev->base = (void *)reg.start;
	i2c_dev->irq = (int)irqs[0];
	i2c_dev->irq_err = (int)irqs[1];
	i2c_dev->setup = stm32f7_setup;
	
	rtems_binary_semaphore_init(&i2c_dev->dma_complete, "i2c-dma");
	i2c_bus_init(&i2c_dev->adap);
	i2c_dev->adap.transfer = stm32f7_i2c_xfer;
	i2c_dev->adap.set_clock = stm32f7_i2c_set_clock;
	i2c_dev->adap.destroy = stm32f7_i2c_destroy;
    devdbg(i2c_dev, "%s: %s reg<0x%x> irq<%d>\n", __func__, dev->name, reg.start, irqs[0]);
    return ofw_platform_bus_device_register(dev, &stm32h7_i2c_bus, 
    	DRVMGR_BUS_TYPE_I2C);
}

static int stm32f7_i2c_probe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
	struct stm32f7_i2c_dev *i2c_dev = dev->priv;
	int err;

    i2c_dev->clk = ofw_clock_request(devp->np, NULL, (pcell_t *)&i2c_dev->clkid, 
        sizeof(i2c_dev->clkid));
    if (!i2c_dev->clk) {
        printk("%s: reqeust clock failed!\n", __func__);
        err = -ENODEV;
		goto _err;
    }

	err = stm32f7_i2c_interrupt_install(i2c_dev, 0);
	if (err) {
		printk("%s: %s request irq failed(%d)\n", __func__, dev->name, err);
		goto _err;
	}

    /* Enable clock */
    clk_enable(i2c_dev->clk, &i2c_dev->clkid);

	err = stm32f7_i2c_set_clock(&i2c_dev->adap, STM32_I2C_SPEED_FAST);
	if (err) {
		printk("%s: %s set clock failed(%d)\n", __func__, dev->name, err);
		goto _err;
	}

	return i2c_bus_register(&i2c_dev->adap, dev->name);
_err:
	stm32f7_i2c_interrupt_uninstall(i2c_dev);
	free(i2c_dev);
	return err;
}

static int stm32f7_i2c_extprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct stm32f7_i2c_dev *i2c_dev = dev->priv;
    pcell_t specs[3];
  
    if (pinctrl_simple_set("/dev/pinctrl", dev)) 
        rtems_panic("%s: %s configure pins failed\n", __func__, 
            dev->name);
 
    i2c_dev->tx = ofw_dma_chan_request(devp->np, "tx", 
        specs, sizeof(specs));
    if (i2c_dev->tx) 
        i2c_dev->tx->config.dma_slot = specs[0];

    i2c_dev->rx = ofw_dma_chan_request(devp->np, "rx", 
        specs, sizeof(specs));
    if (i2c_dev->rx) 
        i2c_dev->rx->config.dma_slot = specs[0];

    return 0;
}

static struct drvmgr_drv_ops stm32h7_i2c_driver = {
	.init = {
        stm32f7_i2c_preprobe,
		stm32f7_i2c_probe,
        stm32f7_i2c_extprobe
	},
};

static const struct dev_id id_table[] = {
    {.compatible = "st,stm32f7-i2c", .data = NULL},
    {NULL, NULL}
};

OFW_PLATFORM_DRIVER(stm32h7_i2c) = {
	.drv = {
		.drv_id   = DRIVER_I2C_ID,
		.name     = "i2c",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &stm32h7_i2c_driver
	},
    .ids = id_table
};
