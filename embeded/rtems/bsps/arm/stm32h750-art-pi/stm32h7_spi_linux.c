/*
 * Copyright 2022 wtcat
 */
#define pr_fmt(fmt) "<spi>: "fmt
#define CONFIG_LOGLEVEL  LOGLEVEL_DEBUG//LOGLEVEL_INFO
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include <rtems.h>
#include <rtems/malloc.h>
#include <rtems/counter.h>
#include <dev/spi/spi.h>
#include <bsp/irq-generic.h>

#include "stm32/stm32_com.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_rcc.h"
#undef GPIO_PULLDOWN
#undef GPIO_PULLUP

#include "drivers/clock.h"
#include "drivers/pinctrl.h"
#include "drivers/dma.h"
#include "drivers/spi.h"
#include "drivers/gpio.h"
#include "drivers/ofw_platform_bus.h"

#include "asm/io.h"
#include "base/log.h"

#define SPI_DEBUG

#define SPI_BPW_RANGE_MASK(min, max) \
	GENMASK((max) - 1, (min) - 1)

/* STM32H7 SPI registers */
#define STM32H7_SPI_CR1			0x00
#define STM32H7_SPI_CR2			0x04
#define STM32H7_SPI_CFG1		0x08
#define STM32H7_SPI_CFG2		0x0C
#define STM32H7_SPI_IER			0x10
#define STM32H7_SPI_SR			0x14
#define STM32H7_SPI_IFCR		0x18
#define STM32H7_SPI_TXDR		0x20
#define STM32H7_SPI_RXDR		0x30
#define STM32H7_SPI_I2SCFGR		0x50

/* STM32H7_SPI_CR1 bit fields */
#define STM32H7_SPI_CR1_SPE		BIT(0)
#define STM32H7_SPI_CR1_MASRX		BIT(8)
#define STM32H7_SPI_CR1_CSTART		BIT(9)
#define STM32H7_SPI_CR1_CSUSP		BIT(10)
#define STM32H7_SPI_CR1_HDDIR		BIT(11)
#define STM32H7_SPI_CR1_SSI		BIT(12)

/* STM32H7_SPI_CR2 bit fields */
#define STM32H7_SPI_CR2_TSIZE_SHIFT	0
#define STM32H7_SPI_CR2_TSIZE		GENMASK(15, 0)

/* STM32H7_SPI_CFG1 bit fields */
#define STM32H7_SPI_CFG1_DSIZE_SHIFT	0
#define STM32H7_SPI_CFG1_DSIZE		GENMASK(4, 0)
#define STM32H7_SPI_CFG1_FTHLV_SHIFT	5
#define STM32H7_SPI_CFG1_FTHLV		GENMASK(8, 5)
#define STM32H7_SPI_CFG1_RXDMAEN	BIT(14)
#define STM32H7_SPI_CFG1_TXDMAEN	BIT(15)
#define STM32H7_SPI_CFG1_MBR_SHIFT	28
#define STM32H7_SPI_CFG1_MBR		GENMASK(30, 28)
#define STM32H7_SPI_CFG1_MBR_MIN	0
#define STM32H7_SPI_CFG1_MBR_MAX	(GENMASK(30, 28) >> 28)

/* STM32H7_SPI_CFG2 bit fields */
#define STM32H7_SPI_CFG2_MIDI_SHIFT	4
#define STM32H7_SPI_CFG2_MIDI		GENMASK(7, 4)
#define STM32H7_SPI_CFG2_COMM_SHIFT	17
#define STM32H7_SPI_CFG2_COMM		GENMASK(18, 17)
#define STM32H7_SPI_CFG2_SP_SHIFT	19
#define STM32H7_SPI_CFG2_SP		GENMASK(21, 19)
#define STM32H7_SPI_CFG2_MASTER		BIT(22)
#define STM32H7_SPI_CFG2_LSBFRST	BIT(23)
#define STM32H7_SPI_CFG2_CPHA		BIT(24)
#define STM32H7_SPI_CFG2_CPOL		BIT(25)
#define STM32H7_SPI_CFG2_SSM		BIT(26)
#define STM32H7_SPI_CFG2_AFCNTR		BIT(31)

/* STM32H7_SPI_IER bit fields */
#define STM32H7_SPI_IER_RXPIE		BIT(0)
#define STM32H7_SPI_IER_TXPIE		BIT(1)
#define STM32H7_SPI_IER_DXPIE		BIT(2)
#define STM32H7_SPI_IER_EOTIE		BIT(3)
#define STM32H7_SPI_IER_TXTFIE		BIT(4)
#define STM32H7_SPI_IER_OVRIE		BIT(6)
#define STM32H7_SPI_IER_MODFIE		BIT(9)
#define STM32H7_SPI_IER_ALL		GENMASK(10, 0)

/* STM32H7_SPI_SR bit fields */
#define STM32H7_SPI_SR_RXP		BIT(0)
#define STM32H7_SPI_SR_TXP		BIT(1)
#define STM32H7_SPI_SR_EOT		BIT(3)
#define STM32H7_SPI_SR_OVR		BIT(6)
#define STM32H7_SPI_SR_MODF		BIT(9)
#define STM32H7_SPI_SR_SUSP		BIT(11)
#define STM32H7_SPI_SR_RXPLVL_SHIFT	13
#define STM32H7_SPI_SR_RXPLVL		GENMASK(14, 13)
#define STM32H7_SPI_SR_RXWNE		BIT(15)

/* STM32H7_SPI_IFCR bit fields */
#define STM32H7_SPI_IFCR_ALL		GENMASK(11, 3)

/* STM32H7_SPI_I2SCFGR bit fields */
#define STM32H7_SPI_I2SCFGR_I2SMOD	BIT(0)

/* STM32H7 SPI Master Baud Rate min/max divisor */
#define STM32H7_SPI_MBR_DIV_MIN		(2 << STM32H7_SPI_CFG1_MBR_MIN)
#define STM32H7_SPI_MBR_DIV_MAX		(2 << STM32H7_SPI_CFG1_MBR_MAX)

/* STM32H7 SPI Communication mode */
#define STM32H7_SPI_FULL_DUPLEX		0
#define STM32H7_SPI_SIMPLEX_TX		1
#define STM32H7_SPI_SIMPLEX_RX		2
#define STM32H7_SPI_HALF_DUPLEX		3

/* SPI Communication type */
#define SPI_FULL_DUPLEX		0
#define SPI_SIMPLEX_TX		1
#define SPI_SIMPLEX_RX		2
#define SPI_3WIRE_TX		3
#define SPI_3WIRE_RX		4

#define SPI_1HZ_NS		1000000000

/*
 * use PIO for small transfers, avoiding DMA setup/teardown overhead for drivers
 * without fifo buffers.
 */
#define SPI_DMA_MIN_BYTES	16

/**
 * struct stm32_spi_reg - stm32 SPI register & bitfield desc
 * @reg:		register offset
 * @mask:		bitfield mask
 * @shift:		left shift
 */
struct stm32_spi_reg {
	int reg;
	int mask;
	int shift;
};

/**
 * struct stm32_spi_regspec - stm32 registers definition, compatible dependent data
 * @en: enable register and SPI enable bit
 * @dma_rx_en: SPI DMA RX enable register end SPI DMA RX enable bit
 * @dma_tx_en: SPI DMA TX enable register end SPI DMA TX enable bit
 * @cpol: clock polarity register and polarity bit
 * @cpha: clock phase register and phase bit
 * @lsb_first: LSB transmitted first register and bit
 * @br: baud rate register and bitfields
 * @dma_rx: SPI RX data register
 * @dma_tx: SPI TX data register
 */
struct stm32_spi_regspec {
	const struct stm32_spi_reg en;
	const struct stm32_spi_reg dma_rx_en;
	const struct stm32_spi_reg dma_tx_en;
	const struct stm32_spi_reg cpol;
	const struct stm32_spi_reg cpha;
	const struct stm32_spi_reg lsb_first;
	const struct stm32_spi_reg br;
	const struct stm32_spi_reg dma_rx;
	const struct stm32_spi_reg dma_tx;
};

struct stm32_spi;

/**
 * struct stm32_spi_cfg - stm32 compatible configuration data
 * @regs: registers descriptions
 * @get_fifo_size: routine to get fifo size
 * @get_bpw_mask: routine to get bits per word mask
 * @disable: routine to disable controller
 * @config: routine to configure controller as SPI Master
 * @set_bpw: routine to configure registers to for bits per word
 * @set_mode: routine to configure registers to desired mode
 * @set_data_idleness: optional routine to configure registers to desired idle
 * time between frames (if driver has this functionality)
 * @set_number_of_data: optional routine to configure registers to desired
 * number of data (if driver has this functionality)
 * @can_dma: routine to determine if the transfer is eligible for DMA use
 * @transfer_one_dma_start: routine to start transfer a single spi_transfer
 * using DMA
 * @dma_rx_cb: routine to call after DMA RX channel operation is complete
 * @dma_tx_cb: routine to call after DMA TX channel operation is complete
 * @transfer_one_irq: routine to configure interrupts for driver
 * @irq_handler_event: Interrupt handler for SPI controller events
 * @irq_handler_thread: thread of interrupt handler for SPI controller
 * @baud_rate_div_min: minimum baud rate divisor
 * @baud_rate_div_max: maximum baud rate divisor
 * @has_fifo: boolean to know if fifo is used for driver
 * @has_startbit: boolean to know if start bit is used to start transfer
 */
struct stm32_spi_cfg {
	const struct stm32_spi_regspec *regs;
	int (*get_fifo_size)(struct stm32_spi *spi);
	int (*get_bpw_mask)(struct stm32_spi *spi);
	void (*disable)(struct stm32_spi *spi);
	int (*config)(struct stm32_spi *spi);
	void (*set_bpw)(struct stm32_spi *spi);
	int (*set_mode)(struct stm32_spi *spi, unsigned int comm_type);
	void (*set_data_idleness)(struct stm32_spi *spi, uint32_t length);
	int (*set_number_of_data)(struct stm32_spi *spi, uint32_t length);
	void (*transfer_one_dma_start)(struct stm32_spi *spi);
	dma_transfer_cb_t dma_rx_cb;
	dma_transfer_cb_t dma_tx_cb;
	int (*transfer_one_irq)(struct stm32_spi *spi);
	void (*irq_handler_event)(void *dev_id);
	void (*irq_handler_thread)(void *dev_id);
	unsigned int baud_rate_div_min;
	unsigned int baud_rate_div_max;
	bool has_fifo;
};

/**
 * struct stm32_spi - private data of the SPI controller
 * @dev: driver model representation of the controller
 * @master: controller master interface
 * @cfg: compatible configuration data
 * @base: virtual memory area
 * @clk: hw kernel clock feeding the SPI clock generator
 * @clk_rate: rate of the hw kernel clock feeding the SPI clock generator
 * @rst: SPI controller reset line
 * @lock: prevent I/O concurrent access
 * @irq: SPI controller interrupt line
 * @fifo_size: size of the embedded fifo in bytes
 * @cur_midi: master inter-data idleness in ns
 * @cur_speed: speed configured in Hz
 * @cur_bpw: number of bits in a single SPI data frame
 * @cur_fthlv: fifo threshold level (data frames in a single data packet)
 * @cur_comm: SPI communication mode
 * @cur_xferlen: current transfer length in bytes
 * @cur_usedma: boolean to know if dma is used in current transfer
 * @tx_buf: data to be written, or NULL
 * @rx_buf: data to be read, or NULL
 * @tx_len: number of data to be written in bytes
 * @rx_len: number of data to be read in bytes
 * @dma_tx: dma channel for TX transfer
 * @dma_rx: dma channel for RX transfer
 * @phys_addr: SPI registers physical base address
 */
struct stm32_spi {
    spi_bus bus;
	rtems_interrupt_server_request req;
    struct drvmgr_dev *dev;
	const struct stm32_spi_cfg *cfg;
	SPI_TypeDef *base;
    rtems_id thread;
	uint32_t clk_rate;
	char lock; /* prevent I/O concurrent access */
	unsigned int fifo_size;
    struct drvmgr_dev *clk;
    int clkid;
    int irq;

	unsigned int cur_midi;
	unsigned int cur_speed;
	unsigned int cur_bpw;
	unsigned int cur_fthlv;
	unsigned int cur_comm;
	unsigned int cur_xferlen;
	bool cur_usedma;

	int cs_num;
    struct gpio_pin *cs_gpios;
	const void *tx_buf;
	void *rx_buf;
	int tx_len;
	int rx_len;
	struct dma_chan *dma_tx;
	struct dma_chan *dma_rx;
	dma_addr_t phys_addr;
};

#define MTX_LOCK(_l) 
#define MTX_UNLOCK(_l)
#define MTX_INIT(_L)

#define	DIV_ROUND_UP(x, n)	howmany(x, n)
#define spin_lock_irqsave(_lock, _flag) (void)(_flag)
#define spin_unlock_irqrestore(_lock, _flag) (void)(_flag)

#define dev_dbg(dev, fmt, ...)  pr_dbg(fmt, ##__VA_ARGS__)
#define dev_info(dev, fmt, ...) pr_info(fmt, ##__VA_ARGS__)
#define dev_warn(dev, fmt, ...) pr_warn(fmt, ##__VA_ARGS__)
#define dev_err(dev, fmt, ...)  pr_err(fmt, ##__VA_ARGS__)


#define readl_relaxed_poll_timeout_atomic(reg_base, var, cond, delay_us, max_time) ({\
	int __remain = max_time; \
	int __ret; \
	(var) = readl_relaxed(reg_base); \
	while (cond) {\
		if (__remain <= 0) \
			break; \
		rtems_counter_delay_nanoseconds(delay_us * 1000); \
		__remain -= delay_us; \
		(var) = readl_relaxed(reg_base); \
	} \
	__ret = __remain > 0; \
	__ret; })
	


static const struct stm32_spi_regspec stm32h7_spi_regspec = {
	/* SPI data transfer is enabled but spi_ker_ck is idle.
	 * CFG1 and CFG2 registers are write protected when SPE is enabled.
	 */
	.en = { STM32H7_SPI_CR1, STM32H7_SPI_CR1_SPE },

	.dma_rx_en = { STM32H7_SPI_CFG1, STM32H7_SPI_CFG1_RXDMAEN },
	.dma_tx_en = { STM32H7_SPI_CFG1, STM32H7_SPI_CFG1_TXDMAEN },

	.cpol = { STM32H7_SPI_CFG2, STM32H7_SPI_CFG2_CPOL },
	.cpha = { STM32H7_SPI_CFG2, STM32H7_SPI_CFG2_CPHA },
	.lsb_first = { STM32H7_SPI_CFG2, STM32H7_SPI_CFG2_LSBFRST },
	.br = { STM32H7_SPI_CFG1, STM32H7_SPI_CFG1_MBR,
		STM32H7_SPI_CFG1_MBR_SHIFT },

	.dma_rx = { STM32H7_SPI_RXDR },
	.dma_tx = { STM32H7_SPI_TXDR },
};

static inline void stm32_spi_set_cs(struct stm32_spi *spi, int cs) {
	pr_dbg("%s: spi select chip (cs = %d)\n", __func__, cs);
	gpiod_pin_assert(&spi->cs_gpios[cs]);
    
}

static inline void stm32_spi_clr_cs(struct stm32_spi *spi, int cs) {
	pr_dbg("%s: spi deselect chip (cs = %d)\n", __func__, cs);
	gpiod_pin_deassert(&spi->cs_gpios[cs]);
}

static inline void stm32_spi_set_bits(struct stm32_spi *spi,
				      uint32_t offset, uint32_t bits)
{
	writel_relaxed(readl_relaxed(spi->base + offset) | bits,
		       spi->base + offset);
}

static inline void stm32_spi_clr_bits(struct stm32_spi *spi,
				      uint32_t offset, uint32_t bits)
{
	writel_relaxed(readl_relaxed(spi->base + offset) & ~bits,
		       spi->base + offset);
}

/**
 * stm32h7_spi_get_fifo_size - Return fifo size
 * @spi: pointer to the spi controller data structure
 */
static int stm32h7_spi_get_fifo_size(struct stm32_spi *spi)
{
	unsigned long flags;
	uint32_t count = 0;

	spin_lock_irqsave(&spi->lock, flags);

	stm32_spi_set_bits(spi, STM32H7_SPI_CR1, STM32H7_SPI_CR1_SPE);

	while (readl_relaxed(spi->base + STM32H7_SPI_SR) & STM32H7_SPI_SR_TXP)
		writeb_relaxed(++count, spi->base + STM32H7_SPI_TXDR);

	stm32_spi_clr_bits(spi, STM32H7_SPI_CR1, STM32H7_SPI_CR1_SPE);

	spin_unlock_irqrestore(&spi->lock, flags);

	dev_dbg(spi->dev, "%d x 8-bit fifo size\n", count);

	return count;
}

/**
 * stm32h7_spi_get_bpw_mask - Return bits per word mask
 * @spi: pointer to the spi controller data structure
 */
static int stm32h7_spi_get_bpw_mask(struct stm32_spi *spi)
{
	unsigned long flags;
	uint32_t cfg1, max_bpw;

	spin_lock_irqsave(&spi->lock, flags);

	/*
	 * The most significant bit at DSIZE bit field is reserved when the
	 * maximum data size of periperal instances is limited to 16-bit
	 */
	stm32_spi_set_bits(spi, STM32H7_SPI_CFG1, STM32H7_SPI_CFG1_DSIZE);

	cfg1 = readl_relaxed(spi->base + STM32H7_SPI_CFG1);
	max_bpw = (cfg1 & STM32H7_SPI_CFG1_DSIZE) >>
		  STM32H7_SPI_CFG1_DSIZE_SHIFT;
	max_bpw += 1;

	spin_unlock_irqrestore(&spi->lock, flags);

	dev_dbg(spi->dev, "%d-bit maximum data frame\n", max_bpw);

	return SPI_BPW_RANGE_MASK(4, max_bpw);
}

/**
 * stm32_spi_prepare_mbr - Determine baud rate divisor value
 * @spi: pointer to the spi controller data structure
 * @speed_hz: requested speed
 * @min_div: minimum baud rate divisor
 * @max_div: maximum baud rate divisor
 *
 * Return baud rate divisor value in case of success or -EINVAL
 */
static int stm32_spi_prepare_mbr(struct stm32_spi *spi, uint32_t speed_hz,
				 uint32_t min_div, uint32_t max_div)
{
	uint32_t div, mbrdiv;

	/* Ensure spi->clk_rate is even */
	div = DIV_ROUND_UP(spi->clk_rate & ~0x1, speed_hz);

	/*
	 * SPI framework set xfer->speed_hz to master->max_speed_hz if
	 * xfer->speed_hz is greater than master->max_speed_hz, and it returns
	 * an error when xfer->speed_hz is lower than master->min_speed_hz, so
	 * no need to check it there.
	 * However, we need to ensure the following calculations.
	 */
	if ((div < min_div) || (div > max_div))
		return -EINVAL;

	/* Determine the first power of 2 greater than or equal to div */
	if (div & (div - 1))
		mbrdiv = fls(div);
	else
		mbrdiv = fls(div) - 1;

	spi->cur_speed = spi->clk_rate / (1 << mbrdiv);

	return mbrdiv - 1;
}

/**
 * stm32h7_spi_prepare_fthlv - Determine FIFO threshold level
 * @spi: pointer to the spi controller data structure
 * @xfer_len: length of the message to be transferred
 */
static uint32_t stm32h7_spi_prepare_fthlv(struct stm32_spi *spi, uint32_t xfer_len)
{
	uint32_t fthlv, half_fifo, packet;

	/* data packet should not exceed 1/2 of fifo space */
	half_fifo = (spi->fifo_size / 2);

	/* data_packet should not exceed transfer length */
	if (half_fifo > xfer_len)
		packet = xfer_len;
	else
		packet = half_fifo;

	if (spi->cur_bpw <= 8)
		fthlv = packet;
	else if (spi->cur_bpw <= 16)
		fthlv = packet / 2;
	else
		fthlv = packet / 4;

	/* align packet size with data registers access */
	if (spi->cur_bpw > 8)
		fthlv -= (fthlv % 2); /* multiple of 2 */
	else
		fthlv -= (fthlv % 4); /* multiple of 4 */

	if (!fthlv)
		fthlv = 1;

	return fthlv;
}

/**
 * stm32h7_spi_write_txfifo - Write bytes in Transmit Data Register
 * @spi: pointer to the spi controller data structure
 *
 * Read from tx_buf depends on remaining bytes to avoid to read beyond
 * tx_buf end.
 */
static void stm32h7_spi_write_txfifo(struct stm32_spi *spi)
{
	while ((spi->tx_len > 0) &&
		       (readl_relaxed(spi->base + STM32H7_SPI_SR) &
			STM32H7_SPI_SR_TXP)) {
		uint32_t offs = spi->cur_xferlen - spi->tx_len;

		if (spi->tx_len >= (int)sizeof(uint32_t)) {
			const uint32_t *tx_buf32 = (const uint32_t *)(spi->tx_buf + offs);

			writel_relaxed(*tx_buf32, spi->base + STM32H7_SPI_TXDR);
			spi->tx_len -= sizeof(uint32_t);
		} else if (spi->tx_len >= (int)sizeof(uint16_t)) {
			const uint16_t *tx_buf16 = (const uint16_t *)(spi->tx_buf + offs);

			writew_relaxed(*tx_buf16, spi->base + STM32H7_SPI_TXDR);
			spi->tx_len -= sizeof(uint16_t);
		} else {
			const uint8_t *tx_buf8 = (const uint8_t *)(spi->tx_buf + offs);

			writeb_relaxed(*tx_buf8, spi->base + STM32H7_SPI_TXDR);
			spi->tx_len -= sizeof(uint8_t);
		}
	}

	dev_dbg(spi->dev, "%s: %d bytes left\n", __func__, spi->tx_len);
}

/**
 * stm32h7_spi_read_rxfifo - Read bytes in Receive Data Register
 * @spi: pointer to the spi controller data structure
 * @flush: boolean indicating that FIFO should be flushed
 *
 * Write in rx_buf depends on remaining bytes to avoid to write beyond
 * rx_buf end.
 */
static void stm32h7_spi_read_rxfifo(struct stm32_spi *spi, bool flush)
{
	uint32_t sr = readl_relaxed(spi->base + STM32H7_SPI_SR);
	uint32_t rxplvl = (sr & STM32H7_SPI_SR_RXPLVL) >>
		     STM32H7_SPI_SR_RXPLVL_SHIFT;

	while ((spi->rx_len > 0) &&
	       ((sr & STM32H7_SPI_SR_RXP) ||
		(flush && ((sr & STM32H7_SPI_SR_RXWNE) || (rxplvl > 0))))) {
		uint32_t offs = spi->cur_xferlen - spi->rx_len;

		if ((spi->rx_len >= (int)sizeof(uint32_t)) ||
		    (flush && (sr & STM32H7_SPI_SR_RXWNE))) {
			uint32_t *rx_buf32 = (uint32_t *)(spi->rx_buf + offs);

			*rx_buf32 = readl_relaxed(spi->base + STM32H7_SPI_RXDR);
			spi->rx_len -= sizeof(uint32_t);
		} else if ((spi->rx_len >= (int)sizeof(uint16_t)) ||
			   (flush && (rxplvl >= 2 || spi->cur_bpw > 8))) {
			uint16_t *rx_buf16 = (uint16_t *)(spi->rx_buf + offs);

			*rx_buf16 = readw_relaxed(spi->base + STM32H7_SPI_RXDR);
			spi->rx_len -= sizeof(uint16_t);
		} else {
			uint8_t *rx_buf8 = (uint8_t *)(spi->rx_buf + offs);

			*rx_buf8 = readb_relaxed(spi->base + STM32H7_SPI_RXDR);
			spi->rx_len -= sizeof(uint8_t);
		}

		sr = readl_relaxed(spi->base + STM32H7_SPI_SR);
		rxplvl = (sr & STM32H7_SPI_SR_RXPLVL) >>
			 STM32H7_SPI_SR_RXPLVL_SHIFT;
	}

	dev_dbg(spi->dev, "%s%s: %d bytes left\n", __func__,
		flush ? "(flush)" : "", spi->rx_len);
}

/**
 * stm32_spi_enable - Enable SPI controller
 * @spi: pointer to the spi controller data structure
 */
static void stm32_spi_enable(struct stm32_spi *spi)
{
	dev_dbg(spi->dev, "enable controller\n");

	stm32_spi_set_bits(spi, spi->cfg->regs->en.reg,
			   spi->cfg->regs->en.mask);
}

/**
 * stm32h7_spi_disable - Disable SPI controller
 * @spi: pointer to the spi controller data structure
 *
 * RX-Fifo is flushed when SPI controller is disabled. To prevent any data
 * loss, use stm32h7_spi_read_rxfifo(flush) to read the remaining bytes in
 * RX-Fifo.
 * Normally, if TSIZE has been configured, we should relax the hardware at the
 * reception of the EOT interrupt. But in case of error, EOT will not be
 * raised. So the subsystem unprepare_message call allows us to properly
 * complete the transfer from an hardware point of view.
 */
static void stm32h7_spi_disable(struct stm32_spi *spi)
{
	unsigned long flags;
	uint32_t cr1, sr;

	dev_dbg(spi->dev, "disable controller\n");

	spin_lock_irqsave(&spi->lock, flags);

	cr1 = readl_relaxed(spi->base + STM32H7_SPI_CR1);

	if (!(cr1 & STM32H7_SPI_CR1_SPE)) {
		spin_unlock_irqrestore(&spi->lock, flags);
		return;
	}

	/* Wait on EOT or suspend the flow */
	if (readl_relaxed_poll_timeout_atomic(spi->base + STM32H7_SPI_SR,
					      sr, !(sr & STM32H7_SPI_SR_EOT),
					      10, 100000) < 0) {
		if (cr1 & STM32H7_SPI_CR1_CSTART) {
			writel_relaxed(cr1 | STM32H7_SPI_CR1_CSUSP,
				       spi->base + STM32H7_SPI_CR1);
			if (readl_relaxed_poll_timeout_atomic(
						spi->base + STM32H7_SPI_SR,
						sr, !(sr & STM32H7_SPI_SR_SUSP),
						10, 100000) < 0)
				dev_warn(spi->dev,
					 "Suspend request timeout\n");
		}
	}

	if (!spi->cur_usedma && spi->rx_buf && (spi->rx_len > 0))
		stm32h7_spi_read_rxfifo(spi, true);

	if (spi->cur_usedma && spi->dma_tx)
		dma_chan_stop(spi->dma_tx);
	if (spi->cur_usedma && spi->dma_rx)
		dma_chan_stop(spi->dma_rx);

	stm32_spi_clr_bits(spi, STM32H7_SPI_CR1, STM32H7_SPI_CR1_SPE);

	stm32_spi_clr_bits(spi, STM32H7_SPI_CFG1, STM32H7_SPI_CFG1_TXDMAEN |
						STM32H7_SPI_CFG1_RXDMAEN);

	/* Disable interrupts and clear status flags */
	writel_relaxed(0, spi->base + STM32H7_SPI_IER);
	writel_relaxed(STM32H7_SPI_IFCR_ALL, spi->base + STM32H7_SPI_IFCR);

	spin_unlock_irqrestore(&spi->lock, flags);
}

/**
 * stm32_spi_can_dma - Determine if the transfer is eligible for DMA use
 * @master: controller master interface
 * @spi_dev: pointer to the spi device
 * @transfer: pointer to spi transfer
 *
 * If driver has fifo and the current transfer size is greater than fifo size,
 * use DMA. Otherwise use DMA for transfer longer than defined DMA min bytes.
 */
static bool stm32_spi_can_dma(struct stm32_spi *spi,
			      struct spi_ioc_transfer *transfer)
{
	unsigned int dma_size;

	if (spi->cfg->has_fifo)
		dma_size = spi->fifo_size;
	else
		dma_size = SPI_DMA_MIN_BYTES;

	dev_dbg(spi->dev, "%s: %s\n", __func__,
		(transfer->len > dma_size) ? "true" : "false");

	return (transfer->len > dma_size);
}

/**
 * stm32h7_spi_irq_thread - Thread of interrupt handler for SPI controller
 * @irq: interrupt line
 * @dev_id: SPI controller master interface
 */
static void stm32h7_spi_irq_thread(void *dev_id)
{
	struct stm32_spi *spi = (struct stm32_spi *)dev_id;
	uint32_t sr, ier, mask;
	unsigned long flags;
	bool end = false;

	spin_lock_irqsave(&spi->lock, flags);

	sr = readl_relaxed(spi->base + STM32H7_SPI_SR);
	ier = readl_relaxed(spi->base + STM32H7_SPI_IER);

	mask = ier;
	/* EOTIE is triggered on EOT, SUSP and TXC events. */
	mask |= STM32H7_SPI_SR_SUSP;
	/*
	 * When TXTF is set, DXPIE and TXPIE are cleared. So in case of
	 * Full-Duplex, need to poll RXP event to know if there are remaining
	 * data, before disabling SPI.
	 */
	if (spi->rx_buf && !spi->cur_usedma)
		mask |= STM32H7_SPI_SR_RXP;

	if (!(sr & mask)) {
		dev_dbg(spi->dev, "spurious IT (sr=0x%08x, ier=0x%08x)\n",
			sr, ier);
		spin_unlock_irqrestore(&spi->lock, flags);
		return;
	}

	if (sr & STM32H7_SPI_SR_SUSP) {
		dev_err(spi->dev, "spi communication is suspended\n");
#if 0
		static DEFINE_RATELIMIT_STATE(rs,
					      DEFAULT_RATELIMIT_INTERVAL * 10,
					      1);
		if (__ratelimit(&rs))
			dev_dbg_ratelimited(spi->dev, "Communication suspended\n");
		if (!spi->cur_usedma && (spi->rx_buf && (spi->rx_len > 0)))
			stm32h7_spi_read_rxfifo(spi, false);
		/*
		 * If communication is suspended while using DMA, it means
		 * that something went wrong, so stop the current transfer
		 */
		if (spi->cur_usedma)
			end = true;
#endif
	}

	if (sr & STM32H7_SPI_SR_MODF) {
		dev_warn(spi->dev, "Mode fault: transfer aborted\n");
		end = true;
	}

	if (sr & STM32H7_SPI_SR_OVR) {
		dev_warn(spi->dev, "Overrun: received value discarded\n");
		if (!spi->cur_usedma && (spi->rx_buf && (spi->rx_len > 0)))
			stm32h7_spi_read_rxfifo(spi, false);
		/*
		 * If overrun is detected while using DMA, it means that
		 * something went wrong, so stop the current transfer
		 */
		if (spi->cur_usedma)
			end = true;
	}

	if (sr & STM32H7_SPI_SR_EOT) {
		if (!spi->cur_usedma && (spi->rx_buf && (spi->rx_len > 0)))
			stm32h7_spi_read_rxfifo(spi, true);
		end = true;
	}

	if (sr & STM32H7_SPI_SR_TXP)
		if (!spi->cur_usedma && (spi->tx_buf && (spi->tx_len > 0)))
			stm32h7_spi_write_txfifo(spi);

	if (sr & STM32H7_SPI_SR_RXP)
		if (!spi->cur_usedma && (spi->rx_buf && (spi->rx_len > 0)))
			stm32h7_spi_read_rxfifo(spi, false);

	writel_relaxed(sr & mask, spi->base + STM32H7_SPI_IFCR);

	spin_unlock_irqrestore(&spi->lock, flags);

	if (end) {
		stm32h7_spi_disable(spi);
		rtems_event_transient_send(spi->thread);
	}

}

static void stm32_spi_isr(void *arg) 
{
	struct stm32_spi *spi = arg;
	rtems_interrupt_server_request_submit(&spi->req);
}

/**
 * stm32_spi_prepare_msg - set up the controller to transfer a single message
 * @master: controller master interface
 * @msg: pointer to spi message
 */
static int stm32_spi_prepare_msg(struct spi_bus *spi_dev,
				 const struct spi_ioc_transfer *msg)
{
	struct stm32_spi *spi = RTEMS_CONTAINER_OF(spi_dev, struct stm32_spi, bus);
	// struct device_node *np = spi_dev->dev.of_node;
	unsigned long flags;
	uint32_t clrb = 0, setb = 0;
	(void)msg;

	/* SPI slave device may need time between data frames */
	spi->cur_midi = 0;
	// if (np && !of_property_read_u32(np, "st,spi-midi-ns", &spi->cur_midi))
	// 	dev_dbg(spi->dev, "%dns inter-data idleness\n", spi->cur_midi);

	if (spi_dev->mode & SPI_CPOL)
		setb |= spi->cfg->regs->cpol.mask;
	else
		clrb |= spi->cfg->regs->cpol.mask;

	if (spi_dev->mode & SPI_CPHA)
		setb |= spi->cfg->regs->cpha.mask;
	else
		clrb |= spi->cfg->regs->cpha.mask;

	if (spi_dev->mode & SPI_LSB_FIRST)
		setb |= spi->cfg->regs->lsb_first.mask;
	else
		clrb |= spi->cfg->regs->lsb_first.mask;

	dev_dbg(spi->dev, "cpol=%d cpha=%d lsb_first=%d cs_high=%d\n",
		spi_dev->mode & SPI_CPOL,
		spi_dev->mode & SPI_CPHA,
		spi_dev->mode & SPI_LSB_FIRST,
		spi_dev->mode & SPI_CS_HIGH);

	spin_lock_irqsave(&spi->lock, flags);

	/* CPOL, CPHA and LSB FIRST bits have common register */
	if (clrb || setb)
		writel_relaxed(
			(readl_relaxed(spi->base + spi->cfg->regs->cpol.reg) &
			 ~clrb) | setb,
			spi->base + spi->cfg->regs->cpol.reg);

	spin_unlock_irqrestore(&spi->lock, flags);

	return 0;
}

/**
 * stm32h7_spi_dma_cb - dma callback
 * @data: pointer to the spi controller data structure
 *
 * DMA callback is called when the transfer is complete or when an error
 * occurs. If the transfer is complete, EOT flag is raised.
 */
static void stm32h7_spi_dma_cb(struct drvmgr_dev *dev, void *user_data, 
	uint32_t channel, int status) {
	struct stm32_spi *spi = user_data;
	unsigned long flags;
	uint32_t sr;

	(void) channel;
	(void) status;
	(void) user_data;
	(void) dev;
	spin_lock_irqsave(&spi->lock, flags);

	sr = readl_relaxed(spi->base + STM32H7_SPI_SR);

	spin_unlock_irqrestore(&spi->lock, flags);

	if (!(sr & STM32H7_SPI_SR_EOT))
		dev_warn(spi->dev, "DMA error (sr=0x%08x)\n", sr);

	/* Now wait for EOT, or SUSP or OVR in case of error */
}

/**
 * stm32_spi_dma_config - configure dma slave channel depending on current
 *			  transfer bits_per_word.
 * @spi: pointer to the spi controller data structure
 * @dma_conf: pointer to the dma_slave_config structure
 * @dir: direction of the dma transfer
 */
static int stm32h7_spi_dma_configure(struct stm32_spi *spi, bool dma_tx) {
	static int dummy;
	uint32_t maxburst;
    int ret;
	
	if (spi->cfg->has_fifo) {
		/* Valid for DMA Half or Full Fifo threshold */
		if (spi->cur_fthlv == 2)
			maxburst = 1;
		else
			maxburst = spi->cur_fthlv;
	} else {
		maxburst = 1;
	}

    if (dma_tx && spi->dma_tx) {
        struct dma_config *config = &spi->dma_tx->config;
        struct dma_chan *dma_tx = spi->dma_tx;
        struct dma_block_config txblk = {0};

        config->channel_direction = MEMORY_TO_PERIPHERAL;
        config->dest_data_size = spi->cur_bpw >> 3;
        config->dest_burst_length = maxburst;
        config->source_data_size = spi->cur_bpw >> 3;
        config->source_burst_length = maxburst;
        config->channel_priority = 0;
        config->head_block = &txblk;
        config->block_count = 1;
		if (spi->cur_comm == SPI_SIMPLEX_TX ||
		    spi->cur_comm == SPI_3WIRE_TX) {
			config->dma_callback = stm32h7_spi_dma_cb;
			config->user_data = spi;
		}
        txblk.block_size = sizeof(dummy);
        txblk.source_address = (dma_addr_t)&dummy;
        txblk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
        txblk.dest_address = (dma_addr_t)&spi->base->TXDR;
        txblk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
        txblk.fifo_mode_control = 1;
        ret = dma_configure(dma_tx->dev, dma_tx->channel, &dma_tx->config);
        if (ret) {
            pr_err("Configure UART(%s) DMA-TX failed: %d\n", spi->dev->name, ret);
            goto _free_chan;
        }
		dev_dbg(spi->dev, "Tx DMA config buswidth=%d, maxburst=%d\n",
			spi->cur_bpw, maxburst);
    }
    if (!dma_tx && spi->dma_rx) {
        struct dma_config *config = &spi->dma_rx->config;
        struct dma_chan *dma_rx = spi->dma_rx;
        struct dma_block_config rxblk = {0};

        config->channel_direction = PERIPHERAL_TO_MEMORY;
        config->dest_data_size = spi->cur_bpw >> 3;
        config->dest_burst_length = 1;
        config->source_data_size = spi->cur_bpw >> 3;
        config->source_burst_length = 1;
        config->complete_callback_en = 1;
        config->channel_priority = 1;
        config->dma_callback = stm32h7_spi_dma_cb;
        config->user_data = spi;
        config->dma_callback = NULL;
        config->user_data = NULL;
        config->head_block = &rxblk;
        config->block_count = 1;

        rxblk.block_size = sizeof(dummy);
        rxblk.source_address = (dma_addr_t)&spi->base->RXDR;
        rxblk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
        rxblk.dest_address = (dma_addr_t)&dummy;
        rxblk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
        rxblk.fifo_mode_control = 1;
        ret = dma_configure(dma_rx->dev, dma_rx->channel, &dma_rx->config);
        if (ret) {
            pr_err("Configure UART(%s) DMA-RX failed: %d\n", spi->dev->name, ret);
            goto _free_chan;
        }
		dev_dbg(spi->dev, "Rx DMA config buswidth=%d, maxburst=%d\n",
			spi->cur_bpw, maxburst);
    }
    return 0;

_free_chan:
    if (spi->dma_tx) 
        dma_stop(spi->dma_rx->dev, spi->dma_rx->channel);
    if (spi->dma_tx) 
        dma_stop(spi->dma_tx->dev, spi->dma_tx->channel);

    return ret;
}

/**
 * stm32h7_spi_transfer_one_irq - transfer a single spi_transfer using
 *				  interrupts
 * @spi: pointer to the spi controller data structure
 *
 * It must returns 0 if the transfer is finished or 1 if the transfer is still
 * in progress.
 */
static int stm32h7_spi_transfer_one_irq(struct stm32_spi *spi)
{
	unsigned long flags;
	uint32_t ier = 0;

	/* Enable the interrupts relative to the current communication mode */
	if (spi->tx_buf && spi->rx_buf)	/* Full Duplex */
		ier |= STM32H7_SPI_IER_DXPIE;
	else if (spi->tx_buf)		/* Half-Duplex TX dir or Simplex TX */
		ier |= STM32H7_SPI_IER_TXPIE;
	else if (spi->rx_buf)		/* Half-Duplex RX dir or Simplex RX */
		ier |= STM32H7_SPI_IER_RXPIE;

	/* Enable the interrupts relative to the end of transfer */
	ier |= STM32H7_SPI_IER_EOTIE | STM32H7_SPI_IER_TXTFIE |
	       STM32H7_SPI_IER_OVRIE | STM32H7_SPI_IER_MODFIE;

	spin_lock_irqsave(&spi->lock, flags);

	stm32_spi_enable(spi);

	/* Be sure to have data in fifo before starting data transfer */
	if (spi->tx_buf)
		stm32h7_spi_write_txfifo(spi);

	stm32_spi_set_bits(spi, STM32H7_SPI_CR1, STM32H7_SPI_CR1_CSTART);

	writel_relaxed(ier, spi->base + STM32H7_SPI_IER);

	spin_unlock_irqrestore(&spi->lock, flags);

	return 1;
}

/**
 * stm32h7_spi_transfer_one_dma_start - Set SPI driver registers to start
 *					transfer using DMA
 * @spi: pointer to the spi controller data structure
 */
static void stm32h7_spi_transfer_one_dma_start(struct stm32_spi *spi)
{
	/* Enable the interrupts relative to the end of transfer */
	stm32_spi_set_bits(spi, STM32H7_SPI_IER, STM32H7_SPI_IER_EOTIE |
						 STM32H7_SPI_IER_TXTFIE |
						 STM32H7_SPI_IER_OVRIE |
						 STM32H7_SPI_IER_MODFIE);

	stm32_spi_enable(spi);

	stm32_spi_set_bits(spi, STM32H7_SPI_CR1, STM32H7_SPI_CR1_CSTART);
}

/**
 * stm32_spi_transfer_one_dma - transfer a single spi_transfer using DMA
 * @spi: pointer to the spi controller data structure
 * @xfer: pointer to the spi_transfer structure
 *
 * It must returns 0 if the transfer is finished or 1 if the transfer is still
 * in progress.
 */
static int stm32_spi_transfer_one_dma(struct stm32_spi *spi,
				      struct spi_ioc_transfer *xfer)
{
	unsigned long flags;
	int err;

	spin_lock_irqsave(&spi->lock, flags);

	if (spi->rx_buf && spi->dma_rx) {
		stm32_spi_set_bits(spi, spi->cfg->regs->dma_rx_en.reg,
				   spi->cfg->regs->dma_rx_en.mask);
		err = stm32h7_spi_dma_configure(spi, false);
		if (err)
			goto dma_desc_error;
		err = dma_chan_reload(spi->dma_rx, (dma_addr_t)&spi->base->RXDR, 
			(dma_addr_t)xfer->rx_buf, xfer->len);
		if (err)
			goto dma_desc_error;
	}

	if (spi->tx_buf && spi->dma_tx) {
		err = stm32h7_spi_dma_configure(spi, true);
		if (err)
			goto dma_desc_error;
		err = dma_chan_reload(spi->dma_tx, (dma_addr_t)xfer->tx_buf, 
			(dma_addr_t)&spi->base->TXDR, xfer->len);
		if (err)
			goto dma_desc_error;

		/* Enable Tx DMA request */
		stm32_spi_set_bits(spi, spi->cfg->regs->dma_tx_en.reg,
				   spi->cfg->regs->dma_tx_en.mask);
	}

	spi->cfg->transfer_one_dma_start(spi);
	spin_unlock_irqrestore(&spi->lock, flags);
	return 1;

dma_desc_error:
	stm32_spi_clr_bits(spi, spi->cfg->regs->dma_rx_en.reg,
			   spi->cfg->regs->dma_rx_en.mask);
	spin_unlock_irqrestore(&spi->lock, flags);
	dev_info(spi->dev, "DMA issue: fall back to irq transfer\n");

	spi->cur_usedma = false;
	return spi->cfg->transfer_one_irq(spi);
}

/**
 * stm32h7_spi_set_bpw - configure bits per word
 * @spi: pointer to the spi controller data structure
 */
static void stm32h7_spi_set_bpw(struct stm32_spi *spi)
{
	uint32_t bpw, fthlv;
	uint32_t cfg1_clrb = 0, cfg1_setb = 0;

	bpw = spi->cur_bpw - 1;

	cfg1_clrb |= STM32H7_SPI_CFG1_DSIZE;
	cfg1_setb |= (bpw << STM32H7_SPI_CFG1_DSIZE_SHIFT) &
		     STM32H7_SPI_CFG1_DSIZE;

	spi->cur_fthlv = stm32h7_spi_prepare_fthlv(spi, spi->cur_xferlen);
	fthlv = spi->cur_fthlv - 1;

	cfg1_clrb |= STM32H7_SPI_CFG1_FTHLV;
	cfg1_setb |= (fthlv << STM32H7_SPI_CFG1_FTHLV_SHIFT) &
		     STM32H7_SPI_CFG1_FTHLV;

	writel_relaxed(
		(readl_relaxed(spi->base + STM32H7_SPI_CFG1) &
		 ~cfg1_clrb) | cfg1_setb,
		spi->base + STM32H7_SPI_CFG1);
}

/**
 * stm32_spi_set_mbr - Configure baud rate divisor in master mode
 * @spi: pointer to the spi controller data structure
 * @mbrdiv: baud rate divisor value
 */
static void stm32_spi_set_mbr(struct stm32_spi *spi, uint32_t mbrdiv)
{
	uint32_t clrb = 0, setb = 0;

	clrb |= spi->cfg->regs->br.mask;
	setb |= ((uint32_t)mbrdiv << spi->cfg->regs->br.shift) &
		spi->cfg->regs->br.mask;

	writel_relaxed((readl_relaxed(spi->base + spi->cfg->regs->br.reg) &
			~clrb) | setb,
		       spi->base + spi->cfg->regs->br.reg);
}

/**
 * stm32_spi_communication_type - return transfer communication type
 * @mode: spi device work mode
 * @transfer: pointer to spi transfer
 */
static unsigned int stm32_spi_communication_type(uint32_t mode,
						 spi_ioc_transfer *transfer)
{
	unsigned int type = SPI_FULL_DUPLEX;

	if (mode & SPI_3WIRE) { /* MISO/MOSI signals shared */
		/*
		 * SPI_3WIRE and xfer->tx_buf != NULL and xfer->rx_buf != NULL
		 * is forbidden and unvalidated by SPI subsystem so depending
		 * on the valid buffer, we can determine the direction of the
		 * transfer.
		 */
		if (!transfer->tx_buf)
			type = SPI_3WIRE_RX;
		else
			type = SPI_3WIRE_TX;
	} else {
		if (!transfer->tx_buf)
			type = SPI_SIMPLEX_RX;
		else if (!transfer->rx_buf)
			type = SPI_SIMPLEX_TX;
	}

	return type;
}

/**
 * stm32h7_spi_set_mode - configure communication mode
 * @spi: pointer to the spi controller data structure
 * @comm_type: type of communication to configure
 */
static int stm32h7_spi_set_mode(struct stm32_spi *spi, unsigned int comm_type)
{
	uint32_t mode;
	uint32_t cfg2_clrb = 0, cfg2_setb = 0;

	if (comm_type == SPI_3WIRE_RX) {
		mode = STM32H7_SPI_HALF_DUPLEX;
		stm32_spi_clr_bits(spi, STM32H7_SPI_CR1, STM32H7_SPI_CR1_HDDIR);
	} else if (comm_type == SPI_3WIRE_TX) {
		mode = STM32H7_SPI_HALF_DUPLEX;
		stm32_spi_set_bits(spi, STM32H7_SPI_CR1, STM32H7_SPI_CR1_HDDIR);
	} else if (comm_type == SPI_SIMPLEX_RX) {
		mode = STM32H7_SPI_SIMPLEX_RX;
	} else if (comm_type == SPI_SIMPLEX_TX) {
		mode = STM32H7_SPI_SIMPLEX_TX;
	} else {
		mode = STM32H7_SPI_FULL_DUPLEX;
	}

	cfg2_clrb |= STM32H7_SPI_CFG2_COMM;
	cfg2_setb |= (mode << STM32H7_SPI_CFG2_COMM_SHIFT) &
		     STM32H7_SPI_CFG2_COMM;

	writel_relaxed(
		(readl_relaxed(spi->base + STM32H7_SPI_CFG2) &
		 ~cfg2_clrb) | cfg2_setb,
		spi->base + STM32H7_SPI_CFG2);

	return 0;
}

/**
 * stm32h7_spi_data_idleness - configure minimum time delay inserted between two
 *			       consecutive data frames in master mode
 * @spi: pointer to the spi controller data structure
 * @len: transfer len
 */
static void stm32h7_spi_data_idleness(struct stm32_spi *spi, uint32_t len)
{
	uint32_t cfg2_clrb = 0, cfg2_setb = 0;

	cfg2_clrb |= STM32H7_SPI_CFG2_MIDI;
	if ((len > 1) && (spi->cur_midi > 0)) {
		uint32_t sck_period_ns = DIV_ROUND_UP(SPI_1HZ_NS, spi->cur_speed);
		uint32_t midi = min((uint32_t)DIV_ROUND_UP(spi->cur_midi, sck_period_ns),
			       (uint32_t)STM32H7_SPI_CFG2_MIDI >>
			       STM32H7_SPI_CFG2_MIDI_SHIFT);

		dev_dbg(spi->dev, "period=%dns, midi=%d(=%dns)\n",
			sck_period_ns, midi, midi * sck_period_ns);
		cfg2_setb |= (midi << STM32H7_SPI_CFG2_MIDI_SHIFT) &
			     STM32H7_SPI_CFG2_MIDI;
	}

	writel_relaxed((readl_relaxed(spi->base + STM32H7_SPI_CFG2) &
			~cfg2_clrb) | cfg2_setb,
		       spi->base + STM32H7_SPI_CFG2);
}

/**
 * stm32h7_spi_number_of_data - configure number of data at current transfer
 * @spi: pointer to the spi controller data structure
 * @nb_words: transfer length (in words)
 */
static int stm32h7_spi_number_of_data(struct stm32_spi *spi, uint32_t nb_words)
{
	uint32_t cr2_clrb = 0, cr2_setb = 0;

	if (nb_words <= (STM32H7_SPI_CR2_TSIZE >>
			 STM32H7_SPI_CR2_TSIZE_SHIFT)) {
		cr2_clrb |= STM32H7_SPI_CR2_TSIZE;
		cr2_setb = nb_words << STM32H7_SPI_CR2_TSIZE_SHIFT;
		writel_relaxed((readl_relaxed(spi->base + STM32H7_SPI_CR2) &
				~cr2_clrb) | cr2_setb,
			       spi->base + STM32H7_SPI_CR2);
	} else {
		return -EMSGSIZE;
	}

	return 0;
}

/**
 * stm32_spi_transfer_one_setup - common setup to transfer a single
 *				  spi_transfer either using DMA or
 *				  interrupts.
 * @spi: pointer to the spi controller data structure
 * @spi_dev: pointer to the spi device
 * @transfer: pointer to spi transfer
 */
static int stm32_spi_transfer_one_setup(struct spi_bus *bus, struct stm32_spi *spi,
					struct spi_ioc_transfer *transfer)
{
	unsigned long flags;
	unsigned int comm_type;
	int nb_words, ret = 0;
	int mbr;

	(void)bus;
	spin_lock_irqsave(&spi->lock, flags);

	spi->cur_xferlen = transfer->len;

	spi->cur_bpw = transfer->bits_per_word;
	spi->cfg->set_bpw(spi);

	/* Update spi->cur_speed with real clock speed */
	mbr = stm32_spi_prepare_mbr(spi, transfer->speed_hz,
				    spi->cfg->baud_rate_div_min,
				    spi->cfg->baud_rate_div_max);
	if (mbr < 0) {
		ret = mbr;
		goto out;
	}

	transfer->speed_hz = spi->cur_speed;
	stm32_spi_set_mbr(spi, mbr);

	comm_type = stm32_spi_communication_type(transfer->mode, transfer);
	ret = spi->cfg->set_mode(spi, comm_type);
	if (ret < 0)
		goto out;

	spi->cur_comm = comm_type;

	if (spi->cfg->set_data_idleness)
		spi->cfg->set_data_idleness(spi, transfer->len);

	if (spi->cur_bpw <= 8)
		nb_words = transfer->len;
	else if (spi->cur_bpw <= 16)
		nb_words = DIV_ROUND_UP(transfer->len * 8, 16);
	else
		nb_words = DIV_ROUND_UP(transfer->len * 8, 32);

	if (spi->cfg->set_number_of_data) {
		ret = spi->cfg->set_number_of_data(spi, nb_words);
		if (ret < 0)
			goto out;
	}

	dev_dbg(spi->dev, "transfer communication mode set to %d\n",
		spi->cur_comm);
	dev_dbg(spi->dev,
		"data frame of %d-bit, data packet of %d data frames\n",
		spi->cur_bpw, spi->cur_fthlv);
	dev_dbg(spi->dev, "speed set to %dHz\n", spi->cur_speed);
	dev_dbg(spi->dev, "transfer of %d bytes (%d data frames)\n",
		spi->cur_xferlen, nb_words);
	dev_dbg(spi->dev, "dma %s\n",
		(spi->cur_usedma) ? "enabled" : "disabled");

out:
	spin_unlock_irqrestore(&spi->lock, flags);

	return ret;
}

/**
 * stm32_spi_transfer_one - transfer a single spi_transfer
 * @master: controller master interface
 * @spi_dev: pointer to the spi device
 * @transfer: pointer to spi transfer
 *
 * It must return 0 if the transfer is finished or 1 if the transfer is still
 * in progress.
 */
static int stm32_spi_transfer_one(struct spi_bus *bus, struct stm32_spi *spi,
				  struct spi_ioc_transfer *transfer)
{
	rtems_status_code sc;
	int ret;

	spi->tx_buf = transfer->tx_buf;
	spi->rx_buf = transfer->rx_buf;
	spi->tx_len = spi->tx_buf ? transfer->len : 0;
	spi->rx_len = spi->rx_buf ? transfer->len : 0;
	spi->cur_usedma = stm32_spi_can_dma(spi, transfer);
	ret = stm32_spi_transfer_one_setup(bus, spi, transfer);
	if (ret) {
		dev_err(spi->dev, "SPI transfer setup failed\n");
		return ret;
	}

	stm32_spi_set_cs(spi, transfer->cs);
	if (spi->cur_usedma)
		ret = stm32_spi_transfer_one_dma(spi, transfer);
	else
		ret = spi->cfg->transfer_one_irq(spi);
	if (ret == 0) { 
		/* Wait transfer complete */
		uint32_t timeout = (transfer->len * 8000) / spi->cur_speed + 1000;
		sc = rtems_event_transient_receive(RTEMS_WAIT|RTEMS_EVENT_ALL, 
			RTEMS_MILLISECONDS_TO_TICKS(timeout));
		if (sc != RTEMS_SUCCESSFUL) {
			pr_err("spi transfer timeout\n");
			stm32_spi_clr_cs(spi, transfer->cs);
			return  rtems_status_code_to_errno(sc);
		}
	}
	stm32_spi_clr_cs(spi, transfer->cs);
	return 0;
}

/**
 * stm32_spi_unprepare_msg - relax the hardware
 * @master: controller master interface
 * @msg: pointer to the spi message
 */
static int stm32_spi_unprepare_msg(struct stm32_spi *spi,
				   struct spi_ioc_transfer *msg)
{
	(void) msg;
	spi->cfg->disable(spi);
	return 0;
}

/**
 * stm32h7_spi_config - Configure SPI controller as SPI master
 * @spi: pointer to the spi controller data structure
 */
static int stm32h7_spi_config(struct stm32_spi *spi)
{
	unsigned long flags;

	spin_lock_irqsave(&spi->lock, flags);

	/* Ensure I2SMOD bit is kept cleared */
	stm32_spi_clr_bits(spi, STM32H7_SPI_I2SCFGR,
			   STM32H7_SPI_I2SCFGR_I2SMOD);

	/*
	 * - SS input value high
	 * - transmitter half duplex direction
	 * - automatic communication suspend when RX-Fifo is full
	 */
	stm32_spi_set_bits(spi, STM32H7_SPI_CR1, STM32H7_SPI_CR1_SSI |
						 STM32H7_SPI_CR1_HDDIR |
						 STM32H7_SPI_CR1_MASRX);

	/*
	 * - Set the master mode (default Motorola mode)
	 * - Consider 1 master/n slaves configuration and
	 *   SS input value is determined by the SSI bit
	 * - keep control of all associated GPIOs
	 */
	stm32_spi_set_bits(spi, STM32H7_SPI_CFG2, STM32H7_SPI_CFG2_MASTER |
						  STM32H7_SPI_CFG2_SSM |
						  STM32H7_SPI_CFG2_AFCNTR);

	spin_unlock_irqrestore(&spi->lock, flags);

	return 0;
}

static const struct stm32_spi_cfg stm32h7_spi_cfg = {
	.regs = &stm32h7_spi_regspec,
	.get_fifo_size = stm32h7_spi_get_fifo_size,
	.get_bpw_mask = stm32h7_spi_get_bpw_mask,
	.disable = stm32h7_spi_disable,
	.config = stm32h7_spi_config,
	.set_bpw = stm32h7_spi_set_bpw,
	.set_mode = stm32h7_spi_set_mode,
	.set_data_idleness = stm32h7_spi_data_idleness,
	.set_number_of_data = stm32h7_spi_number_of_data,
	.transfer_one_dma_start = stm32h7_spi_transfer_one_dma_start,
	.dma_rx_cb = stm32h7_spi_dma_cb,
	.dma_tx_cb = stm32h7_spi_dma_cb,
	.transfer_one_irq = stm32h7_spi_transfer_one_irq,
	.irq_handler_thread = stm32h7_spi_irq_thread,
	.baud_rate_div_min = STM32H7_SPI_MBR_DIV_MIN,
	.baud_rate_div_max = STM32H7_SPI_MBR_DIV_MAX,
	.has_fifo = true,
};

static int stm32h7_spi_get_clksrc(const char *devname, uint32_t *clksrc) {
    if (devname == NULL || clksrc == NULL)
        return -EINVAL;
    if (strncmp("/dev/spi", devname, 8))
        return -EINVAL;

    int id = devname[8] - '0';
    switch(id) {
    case 1 ... 3:
        *clksrc = LL_RCC_SPI123_CLKSOURCE;
        break;
    case 4 ... 5:
        *clksrc = LL_RCC_SPI45_CLKSOURCE;
        break;
    case 6:
        *clksrc = LL_RCC_SPI6_CLKSOURCE;
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

static int stm32_spi_transfer(spi_bus *bus, const struct spi_ioc_transfer *msgs, 
    uint32_t n) {
    struct stm32_spi *spi = (struct stm32_spi *)bus;
    const struct spi_ioc_transfer *curr_msg = msgs;
    int err = -EINVAL;

    spi->thread = rtems_task_self();
    while (n > 0) {
        _Assert(curr_msg->len < UINT16_MAX);
		//stm32_spi_prepare_msg(bus, curr_msg);
        err = stm32_spi_transfer_one(bus, spi, (void *)curr_msg);
        if (err)
            break;
        curr_msg++;
        n--;
    };
    return err;
}

static void stm32_spi_destroy(spi_bus *bus) {
    struct stm32_spi *spi = (struct stm32_spi *)bus;
    rtems_interrupt_level level;

    rtems_interrupt_local_disable(level);
    stm32h7_spi_disable(spi);
    rtems_interrupt_local_enable(level);
    drvmgr_interrupt_unregister(spi->dev, IRQF_HARD(spi->irq), 
        stm32_spi_isr, spi);
    clk_disable(spi->clk, &spi->clkid);
    if (spi->dma_tx)
        dma_chan_release(spi->dma_tx);
    if (spi->dma_rx)
        dma_chan_release(spi->dma_rx);
    spi_bus_destroy(bus);
    free(spi->cs_gpios);
    free(spi);
}

static int stm32_spi_setup(spi_bus *bus) {
	(void) bus;
	return 0;
}

static int stm32_spi_bus_unite(struct drvmgr_drv *drv, struct drvmgr_dev *dev) {
	return ofw_platform_bus_match(drv, dev, DRVMGR_BUS_TYPE_SPI);
}

static struct drvmgr_bus_ops stm32h7_spi_bus = {
	.init = {
		ofw_platform_bus_populate_device,
	},
	.unite = stm32_spi_bus_unite
};

static int stm32_spi_preprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    rtems_ofw_memory_area reg;
    rtems_vector_number irq;
    struct stm32_spi *spi;
    int ret;

    ret = rtems_ofw_get_reg(devp->np, &reg, sizeof(reg));
    if (ret < 0) 
        return -ENOSTR;
    ret = rtems_ofw_get_interrupts(devp->np, &irq, sizeof(irq));
    if (ret < 0) 
        return -ENOSTR;
    spi = rtems_calloc(1, sizeof(struct stm32_spi));
	if (spi == NULL) 
		return -ENOMEM;
	spi->cfg = device_match_data(dev);
    spi->base = (void *)reg.start;
    spi->irq = (int)irq;
    spi->dev = dev;
    devp->devops = &spi->bus;
    dev->priv = spi;
    pr_dbg("%s: %s reg<0x%x> irq<%d>\n", __func__, dev->name, reg.start, irq);
    return ofw_platform_bus_device_register(dev, &stm32h7_spi_bus, 
    DRVMGR_BUS_TYPE_SPI);
}

static int stm32_spi_probe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct stm32_spi *spi = dev->priv;
    uint32_t clksrc = 0;
    int ret = -EINVAL;

    if (stm32h7_spi_get_clksrc(dev->name, &clksrc)) {
        pr_err("%s: Invalid device name (%s)\n", __func__, dev->name);
        return -EINVAL;
    }
    //TODO: Allocate muli-gpios
    spi->cs_gpios = ofw_cs_gpios_request(devp->np, 0, &spi->cs_num);
    if (!spi->cs_gpios) {
        pr_err("%s: reqeust gpio_cs failed!\n", __func__);
        return -ENOSTR;
    }
    _Assert(spi->cs_num < 32);

    spi->clk = ofw_clock_request(devp->np, NULL, (pcell_t *)&spi->clkid, 
        sizeof(spi->clkid));
    if (!spi->clk) {
        pr_err("%s: reqeust clock failed!\n", __func__);
        ret = -ENODEV;
        goto _free_cs;
    }
    ret = drvmgr_interrupt_register(dev, IRQF_HARD(spi->irq), dev->name, 
		stm32_spi_isr, spi);
    if (ret) {
        pr_err("%s register IRQ(%d) failed\n", dev->name, spi->irq);
        goto _free;
    }

	rtems_interrupt_server_request_initialize(0, &spi->req, 
		stm32h7_spi_irq_thread, spi);
	rtems_interrupt_server_request_set_vector(&spi->req, spi->irq);
    spi_bus_init(&spi->bus);
	spi->bus.transfer = stm32_spi_transfer;
    spi->bus.setup = stm32_spi_setup;
    spi->bus.destroy = stm32_spi_destroy;
    spi->bus.lsb_first = false;
    spi->bus.bits_per_word = 8;
    spi->bus.mode = SPI_MODE_0;
    spi->bus.max_speed_hz = LL_RCC_GetSPIClockFreq(clksrc) / 2;
    spi->bus.speed_hz = 8000000;
    spi->bus.cs = UINT8_MAX;
	ret = spi_bus_register(&spi->bus, dev->name);
	if (ret) {
		drvmgr_interrupt_unregister(dev, IRQF_HARD(spi->irq), 
            stm32_spi_isr, spi);
        pr_err("%s: install interrupt failed!\n", __func__);
        goto _free_cs;
    }
    /* Enable clock */
    clk_enable(spi->clk, &spi->clkid);
    pr_dbg("%s: spi bus max-frequency (%u)\n", __func__, spi->bus.max_speed_hz);
	return 0;

_free_cs:
    free(spi->cs_gpios);
_free:
    free(spi);
    return ret;
}

static int stm32h7_spi_extprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct stm32_spi *spi = dev->priv;
    pcell_t specs[3];
  
    if (pinctrl_simple_set("/dev/pinctrl", dev)) 
        rtems_panic("%s: %s configure pins failed\n", __func__, 
            dev->name);
 
    spi->dma_tx = ofw_dma_chan_request(devp->np, "dma_tx", 
        specs, sizeof(specs));
    if (spi->dma_tx) 
        spi->dma_tx->config.dma_slot = specs[0];

    spi->dma_rx = ofw_dma_chan_request(devp->np, "dma_rx", 
        specs, sizeof(specs));
    if (spi->dma_rx) 
        spi->dma_rx->config.dma_slot = specs[0];

    return 0;
}

static struct drvmgr_drv_ops stm32h7_spi_driver = {
	.init = {
        stm32_spi_preprobe,
		stm32_spi_probe,
        stm32h7_spi_extprobe
	},
};

static const struct dev_id id_table[] = {
    {.compatible = "st,stm32h7-spi", (void *)&stm32h7_spi_cfg},
    {NULL, NULL}
};

OFW_PLATFORM_DRIVER(stm32_spi) = {
	.drv = {
		.drv_id   = DRIVER_SPI_ID,
		.name     = "spi",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &stm32h7_spi_driver
	},
    .ids = id_table
};

//TODO: just only for debug
#include <unistd.h>
#include <fcntl.h>
#include "shell/shell_utils.h"

static int spi_writeread(const void *tx_buf, 
    size_t tx_len, void *rx_buf, size_t rx_len) {
    struct drvmgr_dev *master;
    spi_ioc_transfer msgs[] = {
        {
            .len = (uint16_t)tx_len,
            .tx_buf = tx_buf,
            .rx_buf = NULL,
            .cs_change = false,
            .cs = 0,
            .bits_per_word = 8,
            .mode = SPI_MODE_0,
            .speed_hz = 4000000,
            .delay_usecs = 0
        },{
            .len = (uint16_t)rx_len,
            .tx_buf = NULL,
            .rx_buf = rx_buf,
            .cs_change = true,
            .cs = 0,
            .bits_per_word = 8,
            .mode = SPI_MODE_0,
            .speed_hz = 4000000,
            .delay_usecs = 0
        }
    };
    master = drvmgr_dev_by_name("/dev/spi1");
    if (!master)
        return -ENODEV;

    return spi_master_transfer(master, msgs, 2);
}

static int shell_cmd_flashid(int argc, char **argv) {
    (void) argv;
    if (argc > 1)
        return -EINVAL;
    
    uint8_t cmd[] = {0x90, 0, 0, 0};
    uint32_t id = 0;

    if (spi_writeread(cmd, sizeof(cmd), &id, 2) > 0) 
        printf("<flash ID>: 0x%x\n", id);
    
    return 0;
}

static int shell_cmd_spiwrite(int argc, char **argv) {
    struct drvmgr_dev *master;
    char *tx_buf;
    size_t tx_len;

    if (argc < 2) 
        return -EINVAL;

    master = drvmgr_dev_by_name("/dev/spi1");
    if (!master)
        return -ENODEV;
    tx_buf = argv[1];
    tx_len = strlen(tx_buf);
    if (!tx_len) {
        npr_err(ulog, "(%s)Invalid length\n", tx_buf);
        return -EINVAL;
    }
    spi_ioc_transfer msgs[] = {
        {
            .len = (uint16_t)tx_len,
            .tx_buf = tx_buf,
            .rx_buf = NULL,
            .cs_change = true,
            .cs = 0,
            .bits_per_word = 8,
            .mode = SPI_MODE_0,
            .speed_hz = 100000,
            .delay_usecs = 0
        }
    };
    return spi_master_transfer(master, msgs, 1);
}

SHELL_CMDS_DEFINE(spidev_cmds,
	{
		.name = "spiwr",
		.usage = " SPI bus device test",
		.topic = "misc",
		.command = shell_cmd_spiwrite
	},
    {
		.name = "flashid",
		.usage = " Read flash id",
		.topic = "misc",
		.command = shell_cmd_flashid
    }
);
