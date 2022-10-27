/*
 * Copyright 2022 wtcat
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <rtems/console.h>
#include <rtems/termiostypes.h>
#include <rtems/rtems/cache.h> 
#include <rtems/bspIo.h>
#include <rtems/malloc.h>

#include <bsp/irq-generic.h>

#include "drivers/clock.h"
#include "drivers/dma.h"
#include "drivers/ofw_platform_bus.h"
#include "stm32/stm32_queue.h"

#undef B0
#include "stm32h7xx_ll_usart.h"

#define UART_FIFO_SIZE 8
#define UART_INPUT_BUFSZ 1200
#define UART_OUTPUT_BUSSZ 256

struct stm32h7_uart {
    rtems_termios_device_context base;
    struct drvmgr_dev *dev;
    rtems_termios_tty *tty;
    struct drvmgr_dev *clk;
    USART_TypeDef *reg;
    uint32_t bdr;
    uint32_t cflag;
    uint32_t intmsk;
    int clkid;
    int irqno;
    int transmit_err;
    const char *buf;
    size_t transmited;
    size_t length;
    size_t txbuf_size;
    char *txbuf;
    struct dma_chan *tx;
    struct dma_chan *rx;
    struct dma_circle_queue *rxq;
};

#define stm32h7_uart_context(_base) \
    RTEMS_CONTAINER_OF((_base), struct stm32h7_uart, base);

static struct drvmgr_dev *stdout_device;
static int stdout_baudrate = 115200;

static bool stm32h7_uart_set_termios(rtems_termios_device_context *base, 
    const struct termios *t);

static size_t stm32h7_uart_fifo_write(USART_TypeDef *reg, const char *buf, 
    size_t size) {
    size_t idx = 0;
    size_t bytes = min_t(size_t, size, UART_FIFO_SIZE);
    while (idx < bytes) {
        reg->TDR = buf[idx];
        idx++;
    }
    return idx;
}

static int stm32h7_uart_dma_transmit(struct stm32h7_uart *uart, 
    const char *buffer, size_t size) {
    _Assert(size < 65536);
    const char *sndbuf;
    int err;

    /* Stop transmit */
    if (size == 0) {
        LL_USART_DisableDMAReq_TX(uart->reg);
        return 0;
    }
    /* DMA address aligned */
    if ((uintptr_t)buffer & 0x3)
        sndbuf = memcpy(uart->txbuf, buffer, size);
    else
        sndbuf = buffer;
    uart->length = size;
    rtems_cache_invalidate_multiple_data_lines(sndbuf, size);
    err = dma_reload(uart->tx->dev, uart->tx->channel, (dma_addr_t)sndbuf, 
    (dma_addr_t)&uart->reg->TDR, size);
    _Assert(err == 0);
    LL_USART_EnableDMAReq_TX(uart->reg);
    return err;
}

static void stm32h7_uart_fifo_transmit(struct stm32h7_uart *uart, const char *buf, 
    size_t len) {
    if (len > 0) {
        uart->buf = buf;
        uart->length = len;
        uart->transmited = stm32h7_uart_fifo_write(uart->reg, buf, len);
        LL_USART_EnableIT_TXFE(uart->reg);
    } else {
        LL_USART_DisableIT_TXFE(uart->reg);
    }
}

static void __isr stm32h7_txdma_isr(struct drvmgr_dev *dev, void *arg, 
	uint32_t channel, int status) {
    struct stm32h7_uart *uart = (struct stm32h7_uart *)arg;
    if (!status) {
        rtems_termios_dequeue_characters(uart->tty, uart->length);
        uart->length = 0;
    } else {
        uart->transmit_err++;
    }
    (void) dev;
    (void) channel;
}

static void stm32h7_uart_dma_open(struct stm32h7_uart *uart) {
    int ret;
    if (uart->tx) {
        struct dma_config *config = &uart->tx->config;
        struct dma_chan *tx = uart->tx;
        struct dma_block_config txblk = {0};

        config->channel_direction = MEMORY_TO_PERIPHERAL;
        config->dest_data_size = 1;
        config->dest_burst_length = 1;
        config->source_burst_length = 1;
        config->source_data_size = 1;
        config->channel_priority = 0;
        config->dma_callback = stm32h7_txdma_isr;
        config->user_data = uart;
        config->head_block = &txblk;
        config->block_count = 1;
        txblk.block_size = 1;
        txblk.source_address = (dma_addr_t)0;
        txblk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
        txblk.dest_address = (dma_addr_t)&uart->reg->TDR;
        txblk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
        ret = dma_configure(tx->dev, tx->channel, &tx->config);
        if (ret) {
            printk("Configure UART(%s) DMA-TX failed: %d\n", uart->dev->name, ret);
            dma_release_channel(tx->dev, tx->channel);
            uart->tx = NULL;
        }
    }
    if (uart->rx) {
        struct dma_config *config = &uart->rx->config;
        struct dma_chan *rx = uart->rx;
        struct dma_block_config rxblk = {0};

        config->channel_direction = PERIPHERAL_TO_MEMORY;
        config->dest_data_size = 1;
        config->dest_burst_length = 1;
        config->source_burst_length = 1;
        config->source_data_size = 1;
        config->complete_callback_en = 1;
        config->channel_priority = 1;
        config->head_block = &rxblk;
        config->block_count = 1;
        rxblk.block_size = uart->rxq->size;
        rxblk.source_address = (dma_addr_t)&uart->reg->RDR;
        rxblk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
        rxblk.dest_address = (dma_addr_t)uart->rxq->buffer;
        rxblk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
        rxblk.source_reload_en = 1;
        rxblk.dest_reload_en = 1;
        ret = dma_configure(rx->dev, rx->channel, &rx->config);
        ret |= dma_start(rx->dev, rx->channel);
        if (ret) {
            printk("Configure UART(%s) DMA-RX failed: %d\n", uart->dev->name, ret);
            dma_stop(rx->dev, rx->channel);
            dma_release_channel(rx->dev, rx->channel);
            dma_circle_queue_delete(uart->rxq);
            uart->rx = NULL;
            return;
        }
    }
}

static void stm32h7_uart_dma_close(struct stm32h7_uart *uart) {
    if (uart->rx)
        dma_stop(uart->rx->dev, uart->rx->channel);
    if (uart->tx)
        dma_stop(uart->tx->dev, uart->tx->channel);
}

static bool uart_options_parse(const char *s, int *baudrate, 
    uint32_t *cflags) {
    uint32_t cc = 0;
    int bdr = 0;

    while (*s != '\0') {
        if (!isdigit((int)*s)) 
            break;
        bdr = bdr * 10 + (*s - '0');
        s++;
    }
    if (*s == 'N' || *s == 'n') {
        s++;
    } else if (*s == 'O' || *s == 'o') {
        cc |= PARENB | PARODD;
        s++;
    } else if (*s == 'E' || *s == 'e') {
        cc |= PARENB;
        s++;
    } else {
        return false;
    }
    if (!isdigit((int)*s) || s[1] != '\0')
        return false;
    switch (*s - '0') {
    case 7:
        cc |= CS7;
        break;
    case 8:
        cc |= CS8;
        break;
    default:
        return false;
    }
    if (baudrate)
        *baudrate = bdr;
    if (cflags)
        *cflags = cc;
    return true;
}

static __always_inline void stm32h7_uart_rx_isr_process(struct stm32h7_uart *uart, 
    USART_TypeDef *reg) {
    if (!uart->rx) {
        char rxfifo[32];
        int i = 0;
        while (reg->ISR & USART_ISR_RXNE_RXFNE) {
            rxfifo[i] = (char)reg->RDR;
            i++;
        }
        rtems_termios_enqueue_raw_characters(uart->tty, rxfifo, i);
        return;
    }
    struct dma_circle_queue *rxq = uart->rxq;
    if (!dma_circle_queue_update(rxq, uart->rx)) {
        rtems_cache_invalidate_multiple_data_lines(rxq->buffer, rxq->size);
        uint16_t newout = rxq->out + rxq->count;
        if (newout > rxq->size) {
            size_t bytes = rxq->size - rxq->out;
            rtems_termios_enqueue_raw_characters(uart->tty, &rxq->buffer[rxq->out], 
            bytes);
            rtems_termios_enqueue_raw_characters(uart->tty, &rxq->buffer[0], 
            rxq->count - bytes);
        } else {
            rtems_termios_enqueue_raw_characters(uart->tty, &rxq->buffer[rxq->out], 
            rxq->count);
        }
        rxq->count = 0;
        rxq->out = newout % rxq->size;
    } else {
        dma_circle_queue_reset(rxq);
    }
}

static __always_inline void stm32h7_uart_tx_isr_process(struct stm32h7_uart *uart,
    USART_TypeDef *reg) {
    if (uart->length > 0) {
        size_t transmited = uart->transmited;
        size_t remain = uart->length - transmited;
        if (remain > 0) {
            transmited = stm32h7_uart_fifo_write(reg, &uart->buf[transmited], remain);
            uart->transmited += transmited;
        } else {
            uart->length = 0;
            uart->transmited = 0;
            rtems_termios_dequeue_characters(uart->tty, transmited);
        }
    }
}

static void __isr stm32h7_uart_isr(void *arg) {
    struct stm32h7_uart *uart = (struct stm32h7_uart *)arg;
    USART_TypeDef *reg = uart->reg;
    uint32_t status = reg->ISR;

    reg->ICR = status;
    status &= uart->intmsk;
    if (status & (USART_ISR_IDLE | USART_ISR_RXFF)) 
        stm32h7_uart_rx_isr_process(uart, reg);
    if (status & USART_ISR_TXFE) 
        stm32h7_uart_tx_isr_process(uart, reg);
}

static void stm32h7_uart_intr_enable(struct stm32h7_uart *uart) {
    uint32_t mask = 0, cr1 = 0;
    if (!uart->tx)
        mask |= USART_ISR_TXFE;
    if (uart->rx) {
        LL_USART_EnableDMAReq_RX(uart->reg);
    } else {
        cr1 |= USART_CR1_RXFFIE;
        mask |= USART_ISR_RXFF;
    }
    uart->intmsk = mask | USART_ISR_IDLE;
    uart->reg->CR1 |= cr1 | USART_CR1_IDLEIE;
}

static bool stm32h7_uart_open(struct rtems_termios_tty *tty,
    rtems_termios_device_context *base,
    struct termios *term,
    rtems_libio_open_close_args_t *args) {
    (void) args;
    struct stm32h7_uart *uart = stm32h7_uart_context(base);

    if (clk_enable(uart->clk, &uart->clkid))
        return false;
    uart->tty = tty;

    // rtems_termios_set_initial_baud(tty, stdout_baudrate);
    tty->termios.c_ispeed = stdout_baudrate;
    tty->termios.c_ospeed = stdout_baudrate;
    stm32h7_uart_dma_open(uart);
    return stm32h7_uart_set_termios(base, term);
}

static void stm32h7_uart_close(struct rtems_termios_tty *tty,
    rtems_termios_device_context *base,
    rtems_libio_open_close_args_t *args) {
    (void) args;
    (void) tty;
    struct stm32h7_uart *uart = stm32h7_uart_context(base);
    uart->reg->CR1 = 0;
    uart->reg->CR2 = 0;
    uart->reg->CR3 = 0;
    uart->tty = NULL;
    clk_disable(uart->clk, &uart->clkid);
    stm32h7_uart_dma_close(uart);
}

static bool stm32h7_uart_set_termios(rtems_termios_device_context *base, 
    const struct termios *t) {
    struct stm32h7_uart *uart = stm32h7_uart_context(base);
    rtems_interrupt_lock_context ctx;
    LL_USART_InitTypeDef ll_struct;
    uint32_t bdr;

    /*
    *  Calculate the baud rate divisor
    *  Assert ensures there is no division by 0.
    */
    LL_USART_StructInit(&ll_struct);
    bdr = t->c_ospeed;
    ll_struct.BaudRate = bdr;

    /* Parity */
    if (t->c_cflag & PARENB) {
        if (!(t->c_cflag & PARODD))
            ll_struct.Parity = LL_USART_PARITY_ODD;
        else
            ll_struct.Parity = LL_USART_PARITY_EVEN;
    }
    /*  Character Size */
    if (t->c_cflag & CSIZE) {
        switch (t->c_cflag & CSIZE) {
        case CS7:
            if (t->c_cflag & PARENB)
                ll_struct.DataWidth = LL_USART_DATAWIDTH_8B;
            else
                ll_struct.DataWidth = LL_USART_DATAWIDTH_7B;
            break;
        case CS8:
            if (t->c_cflag & PARENB)
                ll_struct.DataWidth = LL_USART_DATAWIDTH_9B;
            else
                ll_struct.DataWidth = LL_USART_DATAWIDTH_8B;
            break;
        default:
            return false;
        }
    } else {
        ll_struct.DataWidth = LL_USART_DATAWIDTH_8B;
    }

    /* Stop Bits */
    if (t->c_cflag & CSTOPB)
        ll_struct.StopBits = LL_USART_STOPBITS_2; /* 2 stop bits */
    else
        ll_struct.StopBits = LL_USART_STOPBITS_1;

    /* Now actually set the chip */
    rtems_termios_device_lock_acquire(base, &ctx);
    if (bdr != uart->bdr || t->c_cflag != uart->cflag) {
        uart->cflag = t->c_cflag;
        uart->bdr = bdr;
        uart->reg->CR1 = 0;
        uart->reg->CR2 = 0;
        uart->reg->CR3 = 0;
        LL_USART_Init(uart->reg, &ll_struct);
        LL_USART_ConfigFIFOsThreshold(uart->reg, LL_USART_FIFOTHRESHOLD_1_8, 
            LL_USART_FIFOTHRESHOLD_8_8);
        stm32h7_uart_intr_enable(uart);
        uart->reg->CR1 |= USART_CR1_FIFOEN | USART_CR1_UE;
    }
    rtems_termios_device_lock_release(base, &ctx);
    return true;
}

static void stm32h7_uart_write(rtems_termios_device_context *base,
    const char *buf, size_t len) {
    struct stm32h7_uart *uart = stm32h7_uart_context(base);
    if (uart->tx) 
        stm32h7_uart_dma_transmit(uart, buf, len);
    else
        stm32h7_uart_fifo_transmit(uart, buf, len);
}

static void stm32h7_uart_polled_putchar(USART_TypeDef *reg, char ch) {
    while (!(reg->ISR & USART_ISR_TXE_TXFNF));
    reg->TDR = ch;
}

static int stm32h7_uart_polled_getchar(USART_TypeDef *reg) {
    (void) reg;
    return -1;
}

static const rtems_termios_device_handler stm32h7_uart_ops = {
    .first_open     = stm32h7_uart_open,
    .last_close     = stm32h7_uart_close,
    .set_attributes = stm32h7_uart_set_termios,
    .write          = stm32h7_uart_write,
    .poll_read      = NULL,
    .mode           = TERMIOS_IRQ_DRIVEN
};

static int stm32h7_uart_preprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    rtems_ofw_memory_area reg;
    rtems_vector_number irq;
    struct stm32h7_uart *uart;
    rtems_status_code sc;
    int ret;

    uart = rtems_calloc(1, sizeof(struct stm32h7_uart));
    if (uart == NULL) 
        return -ENOMEM;
    ret = rtems_ofw_get_reg(devp->np, &reg, sizeof(reg));
    if (ret < 0) {
        ret = -ENOSTR;
        goto _freem;
    }
    ret = rtems_ofw_get_interrupts(devp->np, &irq, sizeof(irq));
    if (ret < 0) {
        ret = -ENOSTR;
        goto _freem;
    }
    uart->reg = (void *)reg.start;
    uart->irqno = (int)irq;
    uart->dev = dev;
    dev->priv = uart;
    rtems_termios_device_context_initialize(&uart->base, dev->name);
    sc = rtems_termios_device_install(dev->name, &stm32h7_uart_ops, 
        NULL, &uart->base);
    if (sc != RTEMS_SUCCESSFUL) {
        printk("UART(%s) register failed: %s\n", dev->name, 
            rtems_status_text(sc));
        ret = -rtems_status_code_to_errno(sc);
        goto _freem;
    }

    return 0;
_freem:
    free(uart);
    return ret;
}

static int stm32h7_uart_probe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct stm32h7_uart *uart = dev->priv;
    pcell_t clks[2];
    int ret;

    if (rtems_ofw_get_enc_prop(devp->np, "clocks", clks, sizeof(clks)) < 0)
        return -ENODATA;
    uart->clk = ofw_device_get_by_phandle(clks[0]);
    if (!uart->clk) {
        printk("Not found clock device for %s\n", dev->name);
        return -ENODATA;
    }
    uart->clkid = clks[1];
    ret = drvmgr_interrupt_register(dev, IRQF_HARD(uart->irqno), dev->name, 
        stm32h7_uart_isr, uart);
    return ret;
}

static void stm32h7_uart_putc(char c) {
    struct drvmgr_dev *stddev = stdout_device;
    if (likely(stddev)) {
        struct stm32h7_uart *uart = stddev->priv;
        stm32h7_uart_polled_putchar(uart->reg, c);
    }
}

static int stm32h7_uart_extprobe(struct drvmgr_dev *dev) {
    struct dev_private *devp = device_get_private(dev);
    struct stm32h7_uart *uart = dev->priv;
    pcell_t specs[3];

    uart->tx = ofw_dma_chan_request(devp->np, "tx", 
        specs, sizeof(specs));
    if (uart->tx) {
        uart->txbuf_size = UART_OUTPUT_BUSSZ;
        uart->txbuf = rtems_cache_coherent_allocate(uart->txbuf_size, 
        sizeof(void *), 0);
        if (!uart->txbuf) {
            dma_release_channel(uart->tx->dev, uart->tx->channel);
            uart->tx = NULL;
        } else {
            uart->tx->config.dma_slot = specs[0];
        }
    }

    uart->rx = ofw_dma_chan_request(devp->np, "rx", 
        specs, sizeof(specs));
    if (uart->rx) {
        uart->rx->config.dma_slot = specs[0];
        uart->rxq = dma_circle_queue_create(UART_INPUT_BUFSZ);
        if (!uart->rxq) {
            dma_release_channel(uart->rx->dev, uart->rx->channel);
            uart->rx = NULL;
        }
    }
    rtems_termios_bufsize(UART_INPUT_BUFSZ, UART_INPUT_BUFSZ, 
    UART_OUTPUT_BUSSZ);
    return 0;
}

static int stm32h7_uart_postprobe(struct drvmgr_dev *dev) {
    (void) dev;
    phandle_t chosen, aliase;
    char *path = NULL;
    char *prop = NULL;
    char *next;

    if (stdout_device)
        return 0;
    chosen = rtems_ofw_find_device("/chosen");
    if ((int)chosen < 0)
        goto _out;
    aliase = rtems_ofw_find_device("/aliases"); 
    if ((int)aliase < 0)
        goto _out;
    if (rtems_ofw_get_prop_alloc(chosen, "stdout-path", (void **)&prop) < 0)
        goto _out;
    for (next = prop; *next != '\0'; next++) {
        if (*next == ':') {
            *next++ = '\0';
            break;
        }
    }
    if (rtems_ofw_get_prop_alloc(aliase, prop, (void **)&path) < 0)
        goto _out;  
    uart_options_parse(next, &stdout_baudrate, NULL);   
    stdout_device = ofw_device_get_by_path(path);
    if (!stdout_device) 
        goto _out;
    link(stdout_device->name, CONSOLE_DEVICE_NAME);
    BSP_output_char = stm32h7_uart_putc;
_out:
    if (prop)
        free(prop);
    if (path)
        free(path);
    return 0;
}

static struct drvmgr_drv_ops uart_driver_ops = {
	.init = {
		stm32h7_uart_preprobe,
        stm32h7_uart_probe,
        stm32h7_uart_extprobe,
        stm32h7_uart_postprobe
	}
};

static const struct dev_id id_table[] = {
    {.compatible = "st,stm32h7-uart", NULL},
    {NULL, NULL}
};

OFW_PLATFORM_DRIVER(stm32h7_uart) = {
    .drv = {
        .drv_id   = DRIVER_PLATFORM_ID,
        .name     = "uart",
        .bus_type = DRVMGR_BUS_TYPE_PLATFORM,
        .ops      = &uart_driver_ops
    },
    .ids = id_table
};
