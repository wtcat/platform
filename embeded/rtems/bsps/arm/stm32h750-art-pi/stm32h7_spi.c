/*
 * Copyright 2022 wtcat
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include <rtems.h>
#include <rtems/malloc.h>

#include "drivers/clock.h"
#include "drivers/dma.h"
#include "drivers/spi.h"
#include "drivers/gpio.h"
#include "drivers/ofw_platform_bus.h"

#include "rtems/rtems/event.h"
#include "stm32/stm32_com.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_rcc.h"

#define SPI_DEBUG

struct stm32h7_spi {
    spi_bus bus;
    rtems_id thread;
    const spi_ioc_transfer *msg;
    struct drvmgr_dev *dev;
    uint8_t *rx_buf;
    const uint8_t *tx_buf;
    size_t tx_len;
    size_t rx_len;
    size_t cur_xferlen;
    SPI_TypeDef *reg;
    struct gpio_pin *cs_gpios;
    struct dma_chan *tx;
    struct dma_chan *rx;
    struct drvmgr_dev *clk;
    int clkid;
    int irq;
    int error;
    int cs_num;
    bool cur_usedma;
};

#define STM32_SPI_TX_COMPLETE RTEMS_EVENT_25
#define STM32_SPI_RX_COMPLETE RTEMS_EVENT_26 

#define STM32H7_FIFO_SIZE 16
#define DIV_ROUND_UP(x, y) howmany(x, y)

#ifdef SPI_DEBUG
#define devdbg(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define devdbg(...)
#endif

static const uint32_t div_table[] = {
    0xFFFFFFFF,
    LL_SPI_BAUDRATEPRESCALER_DIV2,
    LL_SPI_BAUDRATEPRESCALER_DIV4,
    LL_SPI_BAUDRATEPRESCALER_DIV8,
    LL_SPI_BAUDRATEPRESCALER_DIV16,
    LL_SPI_BAUDRATEPRESCALER_DIV32,
    LL_SPI_BAUDRATEPRESCALER_DIV64,
    LL_SPI_BAUDRATEPRESCALER_DIV128,
    LL_SPI_BAUDRATEPRESCALER_DIV256
};

static int stm32h7_spi_get_clksrc(const char *devname, uint32_t *clksrc) {
    if (devname == NULL || clksrc == NULL)
        return -EINVAL;
    if (strncmp("/dev/spi", devname, 8))
        return -EINVAL;
    int id = devname[8] - '0';
    switch(id) {
    case 1 ... 3:
        return LL_RCC_SPI123_CLKSOURCE;
    case 4 ... 5:
        return LL_RCC_SPI45_CLKSOURCE;
    case 6:
        return LL_RCC_SPI6_CLKSOURCE;
    default:
        return -EINVAL;
    }
}

static inline void stm32h7_spi_set_cs(struct gpio_pin *cs_gpios, int cs) {
    gpiod_pin_assert(&cs_gpios[cs]);
}

static inline void stm32h7_spi_clr_cs(struct gpio_pin *cs_gpios, int cs) {
    gpiod_pin_deassert(&cs_gpios[cs]);
}

static inline bool stm32h7_spi_can_dma(struct stm32h7_spi *spi,
    const spi_ioc_transfer *msg) {
    if (msg->len > STM32H7_FIFO_SIZE && (spi->tx || spi->rx))
        return true;
    return false;
}

static void stm32h7_spi_write_txfifo(struct stm32h7_spi *spi, SPI_TypeDef *reg) {
	while (spi->tx_len > 0 && (reg->SR & SPI_SR_TXP)) {
		uint32_t offs = spi->cur_xferlen - spi->tx_len;

		if (spi->tx_len >= sizeof(uint32_t)) {
            reg->TXDR = *(const uint32_t *)(spi->tx_buf + offs);
			spi->tx_len -= sizeof(uint32_t);
		} else if (spi->tx_len >= sizeof(uint16_t)) {
            reg->TXDR = *(const uint16_t *)(spi->tx_buf + offs);
			spi->tx_len -= sizeof(uint16_t);
		} else {
            reg->TXDR = *(const uint8_t *)(spi->tx_buf + offs);
			spi->tx_len -= sizeof(uint8_t);
		}
	}
	devdbg("%s: %d bytes left\n", __func__, spi->tx_len);
}

static void stm32h7_spi_read_rxfifo(struct stm32h7_spi *spi, SPI_TypeDef *reg, bool flush) {
    uint32_t sr = reg->SR;
    uint32_t rxplvl = (sr & SPI_SR_RXPLVL_Msk) >> SPI_SR_RXPLVL_Pos;
    int bpw = spi->bus.bits_per_word;
    
	while (spi->rx_len > 0 && 
        ((sr & SPI_SR_RXP) || (flush && ((sr & SPI_SR_RXWNE) || rxplvl > 0)))) {
		uint32_t offs = spi->cur_xferlen - spi->rx_len;

		if ((spi->rx_len >= sizeof(uint32_t)) ||
            (flush && (sr & SPI_SR_RXWNE))) {
			uint32_t *rx_buf32 = (uint32_t *)(spi->rx_buf + offs);
			*rx_buf32 = reg->RXDR;
			spi->rx_len -= sizeof(uint32_t);

		} else if (spi->rx_len >= sizeof(uint16_t) ||
            (flush && (rxplvl >= 2 || bpw > 8))) {
			uint16_t *rx_buf16 = (uint16_t *)(spi->rx_buf + offs);
			*rx_buf16 = (uint16_t)reg->RXDR;
			spi->rx_len -= sizeof(uint16_t);

		} else {
			uint8_t *rx_buf8 = (uint8_t *)(spi->rx_buf + offs);
			*rx_buf8 = (uint8_t)reg->RXDR;
			spi->rx_len -= sizeof(uint8_t);
		}
        sr = reg->SR;
        rxplvl = (sr & SPI_SR_RXPLVL_Msk) >> SPI_SR_RXPLVL_Pos;
	}

	devdbg("%s: %d bytes left\n", __func__, spi->rx_len);
}

static void stm32h7_spi_disable(struct stm32h7_spi *spi, SPI_TypeDef *reg) {
    if (!spi->cur_usedma) {
        if (spi->rx_buf && spi->rx_len > 0)
            stm32h7_spi_read_rxfifo(spi, reg, true);
    } else {
        if (spi->tx)
            dma_chan_stop(spi->tx);
        if (spi->rx)
            dma_chan_stop(spi->rx);
    }
    reg->CR1 &= ~SPI_CR1_SPE;
    reg->CFG1 &= ~(SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN);

    /* Disable interrupts and clear status flags */
    reg->IER = 0;
    reg->IFCR |= SPI_IER_RXPIE | SPI_IER_TXPIE | SPI_IER_DXPIE | 
                SPI_IER_EOTIE | SPI_IER_TXTFIE | SPI_IER_UDRIE |
                SPI_IER_OVRIE | SPI_IER_CRCEIE | SPI_IER_TIFREIE |
                SPI_IER_MODFIE | SPI_IER_TSERFIE;
    (void) spi;
}

static void __isr stm32h7_spi_isr(void *arg) {
    struct stm32h7_spi *spi = (struct stm32h7_spi *)arg;
    SPI_TypeDef *reg = spi->reg;
    uint32_t ier = reg->IER;
    uint32_t sr = reg->SR;

    sr &= ier;
    /* SPI bus error */
    if (sr & (SPI_SR_MODF | SPI_SR_OVR | SPI_SR_TIFRE | SPI_SR_UDR)) {
        printk("%s: SPI bus error (%d)\n", __func__, sr);
        goto _end;
    }
    if (sr & SPI_SR_RXP) {
		if (!spi->cur_usedma && (spi->rx_buf && spi->rx_len > 0))
			stm32h7_spi_read_rxfifo(spi, reg, false);
    }
    if (sr & SPI_SR_TXP) {
		if (!spi->cur_usedma && (spi->tx_buf && spi->tx_len > 0))
			stm32h7_spi_write_txfifo(spi, reg);
    }
    if (sr & SPI_SR_EOT) {
		if (!spi->cur_usedma && (spi->rx_buf && spi->rx_len > 0))
			stm32h7_spi_read_rxfifo(spi, reg, true);
        goto _end;
    }
    return;

_end:
    stm32h7_spi_disable(spi, reg);
    rtems_event_transient_send(spi->thread);
}

static int stm32h7_spi_dma_configure(struct stm32h7_spi *spi, int bits_per_word) {
    int ret;
    if (spi->tx) {
        struct dma_config *config = &spi->tx->config;
        struct dma_chan *tx = spi->tx;
        struct dma_block_config txblk = {0};

        config->channel_direction = MEMORY_TO_PERIPHERAL;
        config->dest_data_size = bits_per_word >> 3;
        config->dest_burst_length = 4;
        config->source_data_size = bits_per_word >> 3;
        config->source_burst_length = 4;
        config->channel_priority = 0;
        config->dma_callback = NULL;
        config->user_data = NULL;
        config->head_block = &txblk;
        config->block_count = 1;

        txblk.block_size = 1;
        txblk.source_address = (dma_addr_t)0;
        txblk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
        txblk.dest_address = (dma_addr_t)&spi->reg->TXDR;
        txblk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
        txblk.fifo_mode_control = 1;
        ret = dma_configure(tx->dev, tx->channel, &tx->config);
        if (ret) {
            printk("Configure UART(%s) DMA-TX failed: %d\n", spi->dev->name, ret);
            goto _free_chan;
        }
    }
    if (spi->rx) {
        struct dma_config *config = &spi->rx->config;
        struct dma_chan *rx = spi->rx;
        struct dma_block_config rxblk = {0};

        config->channel_direction = PERIPHERAL_TO_MEMORY;
        config->dest_data_size = bits_per_word >> 3;
        config->dest_burst_length = 4;
        config->source_data_size = bits_per_word >> 3;
        config->source_burst_length = 4;
        config->complete_callback_en = 1;
        config->channel_priority = 1;
        config->dma_callback = NULL;
        config->user_data = NULL;
        config->head_block = &rxblk;
        config->block_count = 1;

        rxblk.block_size = 0;
        rxblk.source_address = (dma_addr_t)&spi->reg->RXDR;
        rxblk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
        rxblk.dest_address = (dma_addr_t)0;
        rxblk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
        rxblk.fifo_mode_control = 1;
        ret = dma_configure(rx->dev, rx->channel, &rx->config);
        if (ret) {
            printk("Configure UART(%s) DMA-RX failed: %d\n", spi->dev->name, ret);
            goto _free_chan;
        }
    }
    return 0;

_free_chan:
    if (spi->rx) {
        dma_release_channel(spi->rx->dev, spi->rx->channel);
        spi->rx = NULL;
    }
    if (spi->tx) {
        dma_release_channel(spi->tx->dev, spi->tx->channel);
        spi->tx = NULL;
    }
    return ret;
}

static void stm32h7_spi_set_size(SPI_TypeDef *reg, int cur_bpw, size_t len) {
    size_t nb_words;
	if (cur_bpw <= 8)
		nb_words = len;
	else if (cur_bpw <= 16)
		nb_words = DIV_ROUND_UP(len * 8, 16);
	else
		nb_words = DIV_ROUND_UP(len * 8, 32);
    LL_SPI_SetTransferSize(reg, nb_words);
}

static int stm32h7_spi_configure(struct stm32h7_spi *spi,
    uint32_t speed_hz, uint32_t mode, uint8_t wordbits) {
    LL_SPI_InitTypeDef ll_struct;
    uint32_t div;

    LL_SPI_StructInit(&ll_struct);
    if (mode & SPI_3WIRE) {
        if (!spi->tx_buf)
            ll_struct.TransferDirection = LL_SPI_HALF_DUPLEX_RX;
        else
            ll_struct.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
    } else {
        if (!spi->tx_buf)
            ll_struct.TransferDirection = LL_SPI_SIMPLEX_RX;
        else if (!spi->rx_buf)
            ll_struct.TransferDirection = LL_SPI_SIMPLEX_TX;
    }
    switch (mode & SPI_MODE_3) {
    case SPI_MODE_0:
        ll_struct.ClockPolarity = LL_SPI_POLARITY_LOW;
        ll_struct.ClockPhase = LL_SPI_PHASE_1EDGE;
        break;
    case SPI_MODE_1:
        ll_struct.ClockPolarity = LL_SPI_POLARITY_LOW;
        ll_struct.ClockPhase = LL_SPI_PHASE_2EDGE;
        break;
    case SPI_MODE_2:
        ll_struct.ClockPolarity = LL_SPI_POLARITY_HIGH;
        ll_struct.ClockPhase = LL_SPI_PHASE_1EDGE;
        break;
    case SPI_MODE_3:
        ll_struct.ClockPolarity = LL_SPI_POLARITY_HIGH;
        ll_struct.ClockPhase = LL_SPI_PHASE_2EDGE;
        break;
    default:
        printk("%s: Invalid clock phase\n", __func__);
        return -EINVAL;
    }
    switch (wordbits) {
    case 8:
        ll_struct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
        break;
    case 16:
        ll_struct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
        break;
    case 32:
        ll_struct.DataWidth = LL_SPI_DATAWIDTH_32BIT;
        break;
    default:
        printk("%s: Invalid databit width (%d)\n", __func__, wordbits);
        return -EINVAL;
    }

    ll_struct.Mode = LL_SPI_MODE_MASTER;
    ll_struct.NSS = LL_SPI_NSS_SOFT;
    if (mode & SPI_LSB_FIRST)
        ll_struct.BitOrder = LL_SPI_LSB_FIRST;
    div = spi->bus.max_speed_hz / speed_hz;
    if (!div || (div & (div - 1))) {
        ll_struct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
        printk("%s: Invalid SPI bus frequency(%d) and use default\n",
            __func__, speed_hz);
    } else {
        ll_struct.BaudRate = div_table[div];
    }
    LL_SPI_Init(spi->reg, &ll_struct);
    return 0;
}

static int stm32h7_spi_transmit_one_irq(struct stm32h7_spi *spi) { 
    SPI_TypeDef *reg = spi->reg;
	uint32_t ier = 0;

	/* Enable the interrupts relative to the current communication mode */
	if (spi->tx_buf && spi->rx_buf)	/* Full Duplex */
		ier |= SPI_IER_DXPIE;
	else if (spi->tx_buf)		/* Half-Duplex TX dir or Simplex TX */
		ier |= SPI_IER_TXPIE;
	else if (spi->rx_buf)		/* Half-Duplex RX dir or Simplex RX */
		ier |= SPI_IER_RXPIE;

	/* Enable the interrupts relative to the end of transfer */
	ier |= SPI_IER_EOTIE | SPI_IER_TXTFIE | SPI_IER_OVRIE | SPI_IER_MODFIE;
    
    /* Enable SPI */
    LL_SPI_Enable(reg);

	/* Be sure to have data in fifo before starting data transfer */
	if (spi->tx_buf)
		stm32h7_spi_write_txfifo(spi, reg);
	LL_SPI_StartMasterTransfer(reg);
	reg->IER = ier;
    return 0;
}

static int stm32h7_spi_transmit_one_dma(struct stm32h7_spi *spi) {
    SPI_TypeDef *reg = spi->reg;
    int ret = -EINVAL;

    if (spi->rx_buf && spi->rx) {
        ret = dma_chan_reload(spi->rx, (dma_addr_t)&reg->RXDR, 
            (dma_addr_t)spi->rx_buf, spi->cur_xferlen);
        if (ret) {
            printk("%s: start rx-dma failed(%d)\n", __func__, ret);
            goto _irq_xmit;
        }
        LL_SPI_EnableDMAReq_RX(reg);
    }
    if (spi->tx_buf && spi->tx) {
       ret = dma_chan_reload(spi->rx, (dma_addr_t)spi->tx_buf, 
            (dma_addr_t)&reg->TXDR, spi->cur_xferlen);
        if (ret) {
            printk("%s: start rx-dma failed(%d)\n", __func__, ret);
            goto _irq_xmit;
        }
        LL_SPI_EnableDMAReq_TX(reg);
    }
    if (ret)
        goto _irq_xmit;

    /* Enable spi error interrupt */
    reg->IER |= SPI_IER_OVRIE | SPI_IT_UDR | SPI_IER_TIFREIE | SPI_IER_MODFIE;

    /* Enable SPI */
    LL_SPI_Enable(reg);
    LL_SPI_StartMasterTransfer(reg);
    return ret;

_irq_xmit:
    reg->CFG1 = ~(SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN);
    spi->cur_usedma = false;
    return stm32h7_spi_transmit_one_irq(spi);
}

static int stm32h7_spi_transmit_one(struct stm32h7_spi *spi, spi_bus *bus, 
    const spi_ioc_transfer *msg) {
    int err;

    /* Copy paramters */
    spi->cur_xferlen = msg->len;
	spi->tx_buf = msg->tx_buf;
	spi->rx_buf = msg->rx_buf;
	spi->tx_len = spi->tx_buf ? msg->len : 0;
	spi->rx_len = spi->rx_buf ? msg->len : 0;
    spi->cur_usedma = stm32h7_spi_can_dma(spi, msg);

    /* Update SPI-BUS configuration */
    if (msg->speed_hz != bus->speed_hz || 
        msg->mode != bus->mode || 
        msg->bits_per_word != bus->bits_per_word) {
        err = stm32h7_spi_configure(spi, msg->speed_hz, msg->mode, 
            msg->bits_per_word);
        if (err) {
            printk("%s: configure %s failed(%d)\n", __func__, spi->dev->name, err);
            return err;
        }

        stm32h7_spi_dma_configure(spi, msg->bits_per_word);
        bus->speed_hz = msg->speed_hz;
        bus->mode = msg->mode;
        bus->bits_per_word = msg->bits_per_word;
    }

    stm32h7_spi_set_size(spi->reg, bus->bits_per_word, spi->cur_xferlen);
    if (spi->cur_usedma)
        return stm32h7_spi_transmit_one_dma(spi);
    else
        return stm32h7_spi_transmit_one_irq(spi);
}

static int stm32h7_spi_setup(spi_bus *bus) {
    struct stm32h7_spi *spi = (struct stm32h7_spi *)bus;
    if (bus->max_speed_hz < bus->speed_hz)
        return -EINVAL;
    if (bus->bits_per_word % 8)
        return -EINVAL;

    return stm32h7_spi_configure(spi, bus->max_speed_hz, 
        bus->mode, bus->bits_per_word);
}

static int stm32h7_spi_transfer(spi_bus *bus, const spi_ioc_transfer *msgs, 
    uint32_t n) {
    struct stm32h7_spi *spi = (struct stm32h7_spi *)bus;
    int err = -EINVAL;

    spi->msg = &msgs[0];
    spi->thread = rtems_task_self();
    while (n > 0) {
        err = stm32h7_spi_transmit_one(spi, bus, spi->msg);
        if (err)
            break;
        rtems_event_transient_receive(RTEMS_WAIT|RTEMS_EVENT_ALL, 
            RTEMS_NO_TIMEOUT);
        spi->msg++;
        n--;
    };
    return err;
}

static void stm32h7_spi_destroy(spi_bus *bus) {
	spi_bus_destroy_and_free(bus);
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
    struct stm32h7_spi *spi;
    int ret;

    ret = rtems_ofw_get_reg(devp->np, &reg, sizeof(reg));
    if (ret < 0) 
        return -ENOSTR;
    ret = rtems_ofw_get_interrupts(devp->np, &irq, sizeof(irq));
    if (ret < 0) 
        return -ENOSTR;
    spi = rtems_calloc(1, sizeof(struct stm32h7_spi));
    if (spi == NULL) 
        return -ENOMEM;
    spi->reg = (void *)reg.start;
    spi->irq = (int)irq;
    spi->dev = dev;
    devp->devops = &spi->bus;
    dev->priv = spi;
    return ofw_platform_bus_device_register(dev, &stm32h7_spi_bus, 
    DRVMGR_BUS_TYPE_SPI);
}

static int stm32_spi_probe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct stm32h7_spi *spi = dev->priv;
    uint32_t clksrc;
    int ret = -EINVAL;

    if (stm32h7_spi_get_clksrc(dev->name, &clksrc)) {
        printk("%s: Invalid device name (%s)\n", __func__, dev->name);
        return -EINVAL;
    }
    //TODO: Allocate muli-gpios
    spi->cs_gpios = ofw_cs_gpios_request(devp->np, 0, &spi->cs_num);
    if (!spi->cs_gpios) 
        return -ENOSTR;
    spi->clk = ofw_clock_request(devp->np, NULL, (pcell_t *)&spi->clkid, 
        sizeof(spi->clkid));
    if (!spi->clk) {
        ret = -ENODEV;
        goto _free_cs;
    }
    ret = drvmgr_interrupt_register(dev, IRQF_HARD(spi->irq), dev->name, 
		stm32h7_spi_isr, spi);
    if (ret) {
        printk("%s register IRQ(%d) failed\n", dev->name, spi->irq);
        goto _free;
    }
    ret = stm32_pinctrl_set(dev);
    if (ret) {
        printk("%s configure pins failed: %d\n", dev->name, ret);
        goto _free_cs;
    }

    spi_bus_init(&spi->bus);
	spi->bus.transfer = stm32h7_spi_transfer;
    spi->bus.setup = stm32h7_spi_setup;
    spi->bus.destroy = stm32h7_spi_destroy;
    spi->bus.lsb_first = false;
    spi->bus.bits_per_word = 8;
    spi->bus.mode = SPI_MODE_0;
    spi->bus.max_speed_hz = LL_RCC_GetSPIClockFreq(clksrc) / 2;
    spi->bus.speed_hz = 1000000;
    spi->bus.cs = UINT8_MAX;
	ret = spi_bus_register(&spi->bus, dev->name);
	if (ret) {
		drvmgr_interrupt_unregister(dev, IRQF_HARD(spi->irq), 
            stm32h7_spi_isr, spi);
        goto _free_cs;
    }

    /* Enable clock */
    clk_enable(spi->clk, &spi->clkid);

    /* Reset SPI */
    LL_SPI_DeInit(spi->reg);

    /* */
	return 0;

_free_cs:
    free(spi->cs_gpios);
_free:
    free(spi);
    return ret;
}

static int stm32h7_spi_extprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct stm32h7_spi *spi = dev->priv;
    pcell_t specs[3];

    spi->tx = ofw_dma_chan_request(devp->np, "tx", 
        specs, sizeof(specs));
    if (spi->tx) 
        spi->tx->config.dma_slot = specs[0];

    spi->rx = ofw_dma_chan_request(devp->np, "rx", 
        specs, sizeof(specs));
    if (spi->rx) 
        spi->rx->config.dma_slot = specs[0];

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
    {.compatible = "st,stm32h7-spi", NULL},
    {NULL, NULL}
};

OFW_PLATFORM_DRIVER(stm32h7_spi) = {
	.drv = {
		.drv_id   = DRIVER_SPI_ID,
		.name     = "spi",
		.bus_type = DRVMGR_BUS_TYPE_PLATFORM,
		.ops      = &stm32h7_spi_driver
	},
    .ids = id_table
};
