/*
 * Copyright 2022 wtcat
 */
#include <stdlib.h>
#include <rtems/malloc.h>
#include <sys/errno.h>

#include "drivers/clock.h"
#include "drivers/dma.h"
#include "drivers/spi.h"
#include "drivers/gpio.h"
#include "drivers/ofw_platform_bus.h"

#include "stm32/stm32_com.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_rcc.h"

struct cs_gpio {
    struct gpio_pin *pin;
    int polarity;
};

struct stm32h7_spi {
    spi_bus bus;
    rtems_id thread;
    const spi_ioc_transfer *msg;
    struct drvmgr_dev *dev;
    size_t msg_todo;
    size_t todo;
    int in_transfer;
    uint8_t *rxbuf;
    const uint8_t *txbuf;
    SPI_TypeDef *reg;
    struct cs_gpio cs_gpios;
    struct dma_chan *tx;
    struct dma_chan *rx;
    struct drvmgr_dev *clk;
    int clkid;
    int irq;
    int clksrc;
};


static const uint32_t spi_clksrc[] = {
    0,
    LL_RCC_SPI123_CLKSOURCE,
    LL_RCC_SPI123_CLKSOURCE,
    LL_RCC_SPI123_CLKSOURCE,
    LL_RCC_SPI45_CLKSOURCE,
    LL_RCC_SPI45_CLKSOURCE,
    LL_RCC_SPI6_CLKSOURCE
}

static inline void stm32h7_spi_set_cs(struct cs_gpio *cs) {
    gpiod_set_pin(cs->pin, cs->polarity);
}

static inline void stm32h7_spi_clr_cs(struct cs_gpio *cs) {
    gpiod_set_pin(cs->pin, !cs->polarity);
}

static uint32_t stm32h7_spi_calc_clkdiv(struct stm32h7_spi *spi, uint32_t speed_hz) {
    uint32_t clkfreq = LL_RCC_GetSPIClockFreq(spi->clksrc);
    
}

static void stm32h7_spi_transfer_complete(struct stm32h7_spi *priv) {
    LL_SPI_DisableIT_EOT(priv->reg);
    rtems_event_transient_send(priv->thread);
}

static int stm32h7_spi_configure(struct stm32h7_spi *spi,
    uint32_t speed_hz, uint32_t mode, uint8_t cs, uint8_t wordbits) {
    LL_SPI_InitTypeDef ll_struct;
    uint32_t div;

    LL_SPI_StructInit(&ll_struct);
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
        return -EINVAL;
    }
    if (wordbits == 16)
        ll_struct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
    else if (wordbits == 32)
         ll_struct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
    ll_struct.Mode = LL_SPI_MODE_MASTER;
    ll_struct.NSS = LL_SPI_NSS_SOFT;
    if (mode & SPI_LSB_FIRST)
        ll_struct.BitOrder = LL_SPI_LSB_FIRST;
    LL_SPI_Init(spi->reg, &ll_struct);

    div = spi->bus.max_speed_hz / speed_hz;
    if (div && (div & (div - 1)) == 0) {


        return 0;
    }
    return -EINVAL;
}

static int stm32h7_spi_setup(spi_bus *bus) {
    struct stm32h7_spi *spi = (struct stm32h7_spi *)bus;

    if (bus->max_speed_hz < bus->speed_hz)
        return -EINVAL;
    if (bus->bits_per_word < 4 || bus->bits_per_word > 32)
        return -EINVAL;

    
    return stm32h7_spi_configure(spi, bus->max_speed_hz, 
    bus->mode, bus->cs);
}

static void stm32h7_spi_next_message(struct stm32h7_spi *spi) {
    if (spi->msg_todo > 0) {
        const spi_ioc_transfer *msg = spi->msg;
        spi_bus *bus = &spi->bus;
        if (msg->speed_hz != bus->speed_hz || 
            msg->mode != bus->mode || 
            msg->cs != bus->cs ||
            msg->bits_per_word != bus->bits_per_word) {
            stm32h7_spi_configure(spi, msg->speed_hz, msg->mode, msg->cs);
        }
        spi->todo = msg->len;
        spi->txbuf = msg->tx_buf;
        spi->rxbuf = msg->rx_buf;

        /* xxxxxxxxxxxx */
    } else {
        stm32h7_spi_transfer_complete(spi);
    }
}

static void stm32h7_spi_isr(void *arg) {
    struct stm32h7_spi *spi = (struct stm32h7_spi *)arg;

    if (spi->todo > 0) {

    } else if (spi->in_transfer > 0) {

    } else {
        spi->msg_todo--;
        spi->msg++;
        stm32h7_spi_next_message(spi);
    }
}

static int stm32h7_spi_transfer(spi_bus *bus, const spi_ioc_transfer *msgs, 
    uint32_t n) {
    struct stm32h7_spi *spi = (struct stm32h7_spi *)bus;
    spi->msg_todo = n;
    spi->msg = &msgs[0];
    spi->thread = rtems_task_self();
    stm32h7_spi_next_message(spi);
    rtems_event_transient_receive(RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    return 0;
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
    int ret = -EINVAL;

    //TODO: Allocate muli-gpios
    spi->cs_gpios.pin = ofw_cs_gpio_pin_request(devp->np, 0, &spi->cs_gpios.polarity);
    if (!spi->cs_gpios.pin) 
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
    spi->bus.max_speed_hz = LL_RCC_GetSPIClockFreq(spi->clksrc) / 2;
    spi->bus.speed_hz = 1000000;
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
    free(spi->cs_gpios.pin);
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
