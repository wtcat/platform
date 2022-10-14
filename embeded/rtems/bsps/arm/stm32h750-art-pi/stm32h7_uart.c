/*
 * Copyright 2022 wtcat
 */
#include <stdlib.h>
#include <stdio.h>

#include <rtems/console.h>
#include <rtems/termiostypes.h>
#include <rtems/bspIo.h>
#include <rtems/malloc.h>

#include <bsp/irq-generic.h>

#include "drivers/clock.h"
#include "drivers/ofw_platform_bus.h"
#include "drivers/mio.h"
#include "rtems/rtems/intr.h"
#include "stm32h7xx_ll_usart.h"

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
};

//void LL_USART_EnableDMAReq_TX(USART_TypeDef *USARTx)
//void LL_USART_DisableDMAReq_RX(USART_TypeDef *USARTx)

static bool stm32h7_uart_set_termios(rtems_termios_device_context *base, 
    const struct termios *t);

static inline struct stm32h7_uart *stm32h7_uart_context(rtems_termios_device_context *base) {
    return RTEMS_CONTAINER_OF(base, struct stm32h7_uart, base);
}

static void stm32h7_uart_isr(void *arg) {
    struct stm32h7_uart *uart = (struct stm32h7_uart *)arg;

    rtems_termios_enqueue_raw_characters(uart->tty, buf, i);
    rtems_termios_dequeue_characters(uart->tty, platdata->total);
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
    rtems_termios_set_initial_baud(tty, xxx);
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

static void ns16550_putc_poll(rtems_termios_device_context *base, 
    char ch) {
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
    bdr = rtems_termios_baud_to_number(t->c_ospeed);
    _Assert(bdr != 0);
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
        LL_USART_Disable(uart->reg);
        LL_USART_Init(uart->reg, &ll_struct);
        LL_USART_EnableFIFO(uart->reg);
    }
    rtems_termios_device_lock_release(base, &ctx);
    return true;
}

static void stm32h7_uart_write(rtems_termios_device_context *base,
    const char *buf, size_t len) {
    struct stm32h7_uart *uart = stm32h7_uart_context(base);
    if (len > 0) {
        LL_USART_EnableIT_TXFE(uart->reg);
    } else {
        LL_USART_DisableIT_TXFE(uart->reg);
    }
}

static int stm32h7_uart_polled_getchar(rtems_termios_device_context *base) {
    return -1;
}

static const rtems_termios_device_handler stm32h7_uart_ops = {
    .first_open     = stm32h7_uart_open,
    .last_close     = stm32h7_uart_close,
    .set_attributes = stm32h7_uart_set_termios,
 #ifdef USE_INTR_MODE   
    .write          = stm32h7_uart_write,
    .poll_read      = NULL,
    .mode           = TERMIOS_IRQ_DRIVEN
#else
    .write          = stm32h7_uart_polled_write,
    .poll_read      = stm32h7_uart_polled_getchar,
    .mode           = TERMIOS_POLLED
#endif
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
    struct stm32h7_uart *uart = dev->priv;
    int ret;
   
    
    ret = drvmgr_interrupt_register(dev, IRQF_HARD(uart->irqno), dev->name, 
        stm32h7_uart_isr, uart);
    return ret;
}

static void stm32h7_uart_putc(char c) {
    if (stdout_path) {
        struct ns16550_priv *platdata = stdout_path->priv;
        ns16550_putc_poll(&platdata->base, c);
    }
}

static int stm32h7_uart_post(struct drvmgr_dev *dev) {
    union drvmgr_key_value *prop = devcie_get_property(dev, "stdout");
    if (prop) {
        link(dev->name, CONSOLE_DEVICE_NAME);
        stdout_path = dev;
        BSP_output_char = stm32h7_uart_putc;
    }
    return 0;
}

static struct drvmgr_drv_ops uart_driver_ops = {
	.init = {
		stm32h7_uart_preprobe,
        stm32h7_uart_probe,
        stm32h7_uart_post
	},
};

static const struct dev_id id_table[] = {
    {.compatible = "st,stm32h7-uart", NULL},
    {NULL, NULL}
};

PLATFORM_DRIVER(stm32h7_uart) = {
    .drv = {
        .drv_id   = DRIVER_PLATFORM_ID,
        .name     = "uart",
        .bus_type = DRVMGR_BUS_TYPE_PLATFORM,
        .ops      = &uart_driver_ops
    },
    .ids = id_table
};
