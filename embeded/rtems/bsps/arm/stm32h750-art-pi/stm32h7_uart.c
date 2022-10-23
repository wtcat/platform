/*
 * Copyright 2022 wtcat
 */
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#include <rtems/console.h>
#include <rtems/termiostypes.h>
#include <rtems/bspIo.h>
#include <rtems/malloc.h>

#include <bsp/irq-generic.h>

#include "drivers/clock.h"
#include "drivers/dma.h"
#include "drivers/ofw_platform_bus.h"

#undef B0
#include "stm32h7xx_ll_usart.h"

#define UART_FIFO_SIZE 8

struct stm32h7_uart {
    rtems_termios_device_context base;
    rtems_termios_tty *tty;
    struct drvmgr_dev *clk;
    USART_TypeDef *reg;
    uint32_t bdr;
    uint32_t cflag;
    int clkid;
    int irqno;
    const char *buf;
    size_t transmited;
    size_t length;
    struct ofw_dmachan *tx;
    struct ofw_dmachan *rx;
};

#define stm32h7_uart_context(_base) \
    RTEMS_CONTAINER_OF((_base), struct stm32h7_uart, base);

//void LL_USART_EnableDMAReq_TX(USART_TypeDef *USARTx)
//void LL_USART_DisableDMAReq_RX(USART_TypeDef *USARTx)

static struct drvmgr_dev *stdout_device;
static int stdout_baudrate = 115200;

static bool stm32h7_uart_set_termios(rtems_termios_device_context *base, 
    const struct termios *t);

static size_t stm32h7_uart_fifo_write(USART_TypeDef *reg, const char *buf, 
    size_t size) {
    size_t idx = 0;
    size_t bytes = min_t(size_t, size, UART_FIFO_SIZE);
    while (idx < bytes) {
        //if (reg->ISR & USART_ISR_TXE_TXFNF)
        reg->TDR = buf[idx];
        idx++;
    }
    return idx;
}

static bool uart_options_parse(const char *s, int *baudrate, uint32_t *cflags) {
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

static void stm32h7_uart_isr(void *arg) {
    struct stm32h7_uart *uart = (struct stm32h7_uart *)arg;
    USART_TypeDef *reg = uart->reg;
    uint32_t status = reg->ISR;

    reg->ICR = USART_ICR_IDLECF | USART_ICR_TXFECF;
    if (status & (USART_ISR_IDLE | USART_ISR_RXFF)) {
        char rxfifo[32];
        int i = 0;
        while (reg->ISR & USART_ISR_RXNE_RXFNE) {
            rxfifo[i] = (char)reg->RDR;
            i++;
        }
        rtems_termios_enqueue_raw_characters(uart->tty, rxfifo, i);
    }
    if ((status & USART_ISR_TXFE) && uart->length > 0) {
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

static bool stm32h7_uart_open(struct rtems_termios_tty *tty,
    rtems_termios_device_context *base,
    struct termios *term,
    rtems_libio_open_close_args_t *args) {
    (void) args;
    struct stm32h7_uart *uart = stm32h7_uart_context(base);

    if (clk_enable(uart->clk, &uart->clkid))
        return false;
    uart->tty = tty;
    tty->termios.c_ispeed = stdout_baudrate;
    tty->termios.c_ospeed = stdout_baudrate;

    // rtems_termios_set_initial_baud(tty, stdout_baudrate);
    return stm32h7_uart_set_termios(base, term);
}

static void stm32h7_uart_close(struct rtems_termios_tty *tty,
    rtems_termios_device_context *base,
    rtems_libio_open_close_args_t *args) {
    (void) args;
    struct stm32h7_uart *uart = stm32h7_uart_context(base);
    uart->reg->CR1 = 0;
    uart->reg->CR2 = 0;
    uart->reg->CR3 = 0;
    uart->tty = NULL;
    clk_disable(uart->clk, &uart->clkid);
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
        if (uart->tx)
            LL_USART_EnableDMAReq_TX(uart->reg);
        if (uart->rx)
            LL_USART_EnableDMAReq_RX(uart->reg);
        else
            uart->reg->CR1 |= USART_CR1_RXFFIE;
        uart->reg->CR1 |=  USART_CR1_IDLEIE | USART_CR1_FIFOEN | USART_CR1_UE;
    }
    rtems_termios_device_lock_release(base, &ctx);
    return true;
}

static void stm32h7_uart_write(rtems_termios_device_context *base,
    const char *buf, size_t len) {
    struct stm32h7_uart *uart = stm32h7_uart_context(base);
    if (len > 0) {
        uart->buf = buf;
        uart->length = len;
        uart->transmited = stm32h7_uart_fifo_write(uart->reg, buf, len);
        LL_USART_EnableIT_TXFE(uart->reg);
    } else {
        LL_USART_DisableIT_TXFE(uart->reg);
    }
}

static void stm32h7_uart_polled_putchar(USART_TypeDef *reg, char ch) {
    while (!(reg->ISR & USART_ISR_TXE_TXFNF));
    reg->TDR = ch;
}

static int stm32h7_uart_polled_getchar(USART_TypeDef *reg) {
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
    pcell_t pecs[3];

    uart->tx = ofw_dma_chan_request(devp->np, "tx", pecs, sizeof(pecs));
    if (uart->tx) {
        
    }

    return 0;
}

static int stm32h7_uart_postprobe(struct drvmgr_dev *dev) {
    phandle_t chosen, aliase;
    char *path = NULL;
    char *prop = NULL;
    char *next;

    if (stdout_device)
        return 0;
    chosen = rtems_ofw_find_device("/chosen");
    if (chosen < 0)
        goto _out;
    aliase = rtems_ofw_find_device("/aliases"); 
    if (aliase < 0)
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
	},
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
