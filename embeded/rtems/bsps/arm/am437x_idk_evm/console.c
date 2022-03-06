#include <rtems/console.h>
#include <rtems/termiostypes.h>

#include "bsp/platform_bus.h"

struct ns16550_info {
    uint32_t fifo_size;
    uint32_t reg_shift;
    uint32_t clock;
};

struct ns16550_priv {
   rtems_termios_device_context base;
    rtems_termios_tty *tty;
    struct drvmgr_dev *dev;
    const char *buf;
    size_t total;
    size_t remaining;
    size_t current;
    bool txintr;

    uintptr_t port;
    int irq;
    uint32_t clock;
    bool has_fractional_divider_register;
    bool has_precision_clock_synthesizer;
    uint8_t modem_control;
    uint8_t line_control;
    uint32_t baud_divisor;
    const struct ns16550_info *info;
};

#define NS16550_DEFAULT_BDR 115200
#define SP_FIFO_SIZE 16

/*
 * Register list
 */
#define _REGOFS(N) ((N) << _reg_shift)

#define NS16550_RECEIVE_BUFFER   _REGOFS(0)
#define NS16550_TRANSMIT_BUFFER  _REGOFS(0)
#define NS16550_DIVISOR_LATCH_L  _REGOFS(0)
#define NS16550_INTERRUPT_ENABLE _REGOFS(1)
#define NS16550_DIVISOR_LATCH_M  _REGOFS(1)
#define NS16550_INTERRUPT_ID     _REGOFS(2)
#define NS16550_FIFO_CONTROL     _REGOFS(2)
#define NS16550_LINE_CONTROL     _REGOFS(3)
#define NS16550_MODEM_CONTROL    _REGOFS(4)
#define NS16550_LINE_STATUS      _REGOFS(5)
#define NS16550_MODEM_STATUS     _REGOFS(6)
#define NS16550_SCRATCH_PAD      _REGOFS(7)
#define NS16550_FRACTIONAL_DIVIDER _REGOFS(10)

/*
 * Define serial port interrupt enable register structure.
 */
#define SP_INT_RX_ENABLE  0x01
#define SP_INT_TX_ENABLE  0x02
#define SP_INT_LS_ENABLE  0x04
#define SP_INT_MS_ENABLE  0x08

#define NS16550_ENABLE_ALL_INTR           (SP_INT_RX_ENABLE | SP_INT_TX_ENABLE)
#define NS16550_DISABLE_ALL_INTR          0x00
#define NS16550_ENABLE_ALL_INTR_EXCEPT_TX (SP_INT_RX_ENABLE)

/*
 * Define serial port interrupt ID register structure.
 */
#define SP_IID_0 0x01
#define SP_IID_1 0x02
#define SP_IID_2 0x04
#define SP_IID_3 0x08

/*
 * Define serial port fifo control register structure.
 */
#define SP_FIFO_ENABLE  0x01
#define SP_FIFO_RXRST 0x02
#define SP_FIFO_TXRST 0x04
#define SP_FIFO_DMA   0x08
#define SP_FIFO_RXLEVEL 0xc0
//#define SP_FIFO_SIZE(platdata) (platdata->info->fifo_size)


/*
 * Define serial port line control register structure.
 */
#define SP_LINE_SIZE  0x03
#define SP_LINE_STOP  0x04
#define SP_LINE_PAR   0x08
#define SP_LINE_ODD   0x10
#define SP_LINE_STICK 0x20
#define SP_LINE_BREAK 0x40
#define SP_LINE_DLAB  0x80

/*
 * Line status register character size definitions.
 */
#define FIVE_BITS 0x0                   /* five bits per character */
#define SIX_BITS 0x1                    /* six bits per character */
#define SEVEN_BITS 0x2                  /* seven bits per character */
#define EIGHT_BITS 0x3                  /* eight bits per character */

/*
 * Define serial port modem control register structure.
 */
#define SP_MODEM_DTR  0x01
#define SP_MODEM_RTS  0x02
#define SP_MODEM_IRQ  0x08
#define SP_MODEM_LOOP 0x10
#define SP_MODEM_DIV4 0x80

/*
 * Define serial port line status register structure.
 */
#define SP_LSR_RDY    0x01
#define SP_LSR_EOVRUN 0x02
#define SP_LSR_EPAR   0x04
#define SP_LSR_EFRAME 0x08
#define SP_LSR_BREAK  0x10
#define SP_LSR_THOLD  0x20
#define SP_LSR_TX   0x40
#define SP_LSR_EFIFO  0x80

static int _reg_shift;


static inline int ns16550_tx_empty(struct ns16550_priv *platdata) {
    int status = readb(platdata->port + NS16550_LINE_STATUS);
    return status & SP_LSR_THOLD;
}

static ssize_t ns16550_fifo_write(struct ns16550_priv *platdata, 
    const char *buf, size_t size) {
    size_t out = size > SP_FIFO_SIZE? SP_FIFO_SIZE: size;
    for (size_t i = 0; i < out; ++i)
        writeb(buf[i], platdata->port + NS16550_TRANSMIT_BUFFER);
    return out;
}

static int ns16550_set_termios(struct udevice *dev, const struct termios *t)
{
    struct ns16550_platdata *platdata = dev_get_plat(dev);
    uint32_t ulBaudDivisor;
    uint8_t ucLineControl;
    uint32_t baud_requested;

    /*
    *  Calculate the baud rate divisor
    *  Assert ensures there is no division by 0.
    */
    baud_requested = rtems_termios_baud_to_number(t->c_ospeed);
    _Assert( baud_requested != 0 );
    ulBaudDivisor = ns16550_baud_divisor_get(platdata, baud_requested);
    ucLineControl = 0;

    /* Parity */
    if (t->c_cflag & PARENB) {
        ucLineControl |= SP_LINE_PAR;
        if (!(t->c_cflag & PARODD))
            ucLineControl |= SP_LINE_ODD;
    }
    /*  Character Size */
    if (t->c_cflag & CSIZE) {
        switch (t->c_cflag & CSIZE) {
        case CS5:
            ucLineControl |= FIVE_BITS;
            break;
        case CS6:
            ucLineControl |= SIX_BITS;
            break;
        case CS7:
            ucLineControl |= SEVEN_BITS;
            break;
        case CS8:
            ucLineControl |= EIGHT_BITS;
            break;
        }
    } else {
        ucLineControl |= EIGHT_BITS;
    }

    /* Stop Bits */
    if (t->c_cflag & CSTOPB) 
        ucLineControl |= SP_LINE_STOP; /* 2 stop bits */

    /* Now actually set the chip */
    if (ulBaudDivisor != platdata->baud_divisor || ucLineControl != platdata->line_control) {
        platdata->baud_divisor = ulBaudDivisor;
        platdata->line_control = ucLineControl;

        /*
        *  Set the baud rate
        *
        *  NOTE: When the Divisor Latch Access Bit (DLAB) is set to 1,
        *        the transmit buffer and interrupt enable registers
        *        turn into the LSB and MSB divisor latch registers.
        */
        writeb(SP_LINE_DLAB, platdata->port + NS16550_LINE_CONTROL);
        writeb(ulBaudDivisor & 0xff, platdata->port + NS16550_DIVISOR_LATCH_L);
        writeb((ulBaudDivisor >> 8) & 0xff, platdata->port + NS16550_DIVISOR_LATCH_M);

        /* Now write the line control */
        if (platdata->has_precision_clock_synthesizer) {
            writeb((uint8_t)(ulBaudDivisor >> 24), platdata->port + NS16550_SCRATCH_PAD);
            writeb(ucLineControl, platdata->port + NS16550_LINE_CONTROL);
            writeb((uint8_t)(ulBaudDivisor >> 16), platdata->port + NS16550_SCRATCH_PAD);
        } else {
            writeb(ucLineControl, platdata->port + NS16550_LINE_CONTROL);
        }
    }
    return 0;
}

static int ns16550_putc_poll(struct udevice *dev, const char ch)
{
    struct ns16550_platdata *platdata = dev_get_plat(dev);
    uint32_t status;

    do {
        status = readb(platdata->port + NS16550_LINE_STATUS);
    } while (!(status & SP_LSR_THOLD));
    writeb(ch, platdata->port + NS16550_TRANSMIT_BUFFER);
    return 0;
}




static uint32_t ns16550_baud_divisor_get(struct ns16550_priv *platdata, 
    uint32_t baud) {
    uint32_t clock;
    uint32_t baudDivisor;
    uint32_t err;
    uint32_t actual;
    uint32_t newErr;

    if (platdata->clock) 
        clock = platdata->clock;
    else
        clock = 115200;
    baudDivisor = clock / (baud * 16);
    if (platdata->has_precision_clock_synthesizer) {
        uint32_t i;
        err = baud;
        baudDivisor = 0x0001ffff;
        for (i = 2; i <= 0x10000; i *= 2) {
            uint32_t fout;
            uint32_t fin;

            fin = i - 1;
            fout = (baud * fin * 16) / clock;
            actual = (clock * fout) / (16 * fin);
            newErr = actual > baud ? actual - baud : baud - actual;
            if (newErr < err) {
                err = newErr;
                baudDivisor = fin | (fout << 16);
            }
        }
    } else if (platdata->has_fractional_divider_register) {
        uint32_t fractionalDivider = 0x10;
        uint32_t mulVal;
        uint32_t divAddVal;

        err = baud;
        clock /= 16 * baudDivisor;
        for (mulVal = 1; mulVal < 16; ++mulVal) {
            for (divAddVal = 0; divAddVal < mulVal; ++divAddVal) {
                actual = (mulVal * clock) / (mulVal + divAddVal);
                newErr = actual > baud ? actual - baud : baud - actual;
                if (newErr < err) {
                    err = newErr;
                    fractionalDivider = (mulVal << 4) | divAddVal;
                }
            }
        }
        writeb(fractionalDivider, platdata->port + NS16550_FRACTIONAL_DIVIDER);
    } 
    return baudDivisor;
}

static void ns16550_isr(void *arg) {
    struct ns16550_priv *platdata = arg;
    char buf [SP_FIFO_SIZE];
    int i;

    do {
        for (i = 0; i < SP_FIFO_SIZE; ++i) {
            if (readb(platdata->port + NS16550_LINE_STATUS) & SP_LSR_RDY)
                buf[i] = readb(platdata->port + NS16550_RECEIVE_BUFFER);
            else
                break;
        }
        rtems_termios_enqueue_raw_characters(platdata->tty, buf, i);
        if (platdata->total && ns16550_tx_empty(platdata)) {
            size_t current = platdata->current;
            platdata->buf += current;
            platdata->remaining -= current;
            if (platdata->remaining > 0) {
                platdata->current = ns16550_fifo_write(platdata, platdata->buf, 
                    platdata->remaining);
            } else {
                ret = rtems_termios_dequeue_characters(platdata->tty, 
                    platdata->total);
                platdata->total = 0;
            }
        }
    } while (!(readb(platdata->port + NS16550_INTERRUPT_ID) & SP_IID_0));
}

static void serial_class_txintr_enable(struct ns16550_priv *platdata) {
    rtems_interrupt_lock_context ctx;
    rtems_termios_device_lock_acquire(&platdata->base, &ctx);
    writeb(SP_INT_TX_ENABLE, platdata->port + NS16550_INTERRUPT_ENABLE);
    platdata->txintr = true;
    rtems_termios_device_lock_release(&platdata->base, &ctx);
}

static void serial_class_txintr_disable(struct ns16550_priv *platdata) {
    rtems_interrupt_lock_context ctx;
    rtems_termios_device_lock_acquire(&platdata->base, &ctx);
    writeb(NS16550_ENABLE_ALL_INTR_EXCEPT_TX, 
        platdata->port + NS16550_INTERRUPT_ENABLE);
    platdata->txintr = false;
    rtems_termios_device_lock_release(&platdata->base, &ctx);
}

static bool serial_class_open(struct rtems_termios_tty *tty,
    rtems_termios_device_context *base,
    struct termios *term,
    rtems_libio_open_close_args_t *args) {
    struct ns16550_priv *platdata = RTEMS_CONTAINER_OF(base, 
        struct ns16550_priv, base);
    platdata->tty = tty;
    rtems_termios_set_initial_baud(tty, SERIAL_DEFAULT_BDR); // class default baudrate
   writeb(NS16550_ENABLE_ALL_INTR_EXCEPT_TX, 
        platdata->port + NS16550_INTERRUPT_ENABLE);
    return 0;
}

static void serial_class_close(struct rtems_termios_tty *tty,
                            rtems_termios_device_context *base,
                            rtems_libio_open_close_args_t *args
)
{
    struct serial_class *platdata = container_of(base, 
        struct serial_class, base);
    const struct dm_serial_ops *ops = serial_get_ops(platdata->dev);

    ops->release(platdata->dev);
}

static void serial_class_putc_poll(rtems_termios_device_context *base, 
    char out)
{
    struct serial_class *platdata = container_of(base, 
        struct serial_class, base);
    const struct dm_serial_ops *ops = serial_get_ops(platdata->dev);
    bool enabled;

    enabled = serial_class_txintr_disable(platdata, ops);
    ops->putc_poll(platdata->dev, out);
     if (enabled)
        serial_class_txintr_enable(platdata, ops);
}

static void serial_class_flowctrl_starttx(rtems_termios_device_context *base)
{
    struct serial_class *platdata = container_of(base, struct serial_class, base);
    const struct dm_serial_ops *ops = serial_get_ops(platdata->dev);
    rtems_interrupt_lock_context ctx;

    rtems_termios_device_lock_acquire(base, &ctx);
    ops->set_mctrl(platdata->dev, TIOCM_RTS);
    rtems_termios_device_lock_release(base, &ctx);
}

static void serial_class_flowctrl_stoptx(rtems_termios_device_context *base)
{
    struct serial_class *platdata = container_of(base, struct serial_class, base);
    const struct dm_serial_ops *ops = serial_get_ops(platdata->dev);
    rtems_interrupt_lock_context ctx;

    rtems_termios_device_lock_acquire(base, &ctx);
    ops->set_mctrl(platdata->dev, 0);
    rtems_termios_device_lock_release(base, &ctx);
}

static bool serial_class_termios_set(rtems_termios_device_context *base,
    const struct termios *t)
{
    struct serial_class *platdata = container_of(base, struct serial_class, base);
    struct dm_serial_ops *ops = serial_get_ops(platdata->dev);
    rtems_interrupt_lock_context ctx;
    int ret;

    rtems_termios_device_lock_acquire(base, &ctx);
    ret = ops->set_termios(platdata->dev, t);
    rtems_termios_device_lock_release(base, &ctx);
    return !ret;
}

static void serial_class_xmit_start(rtems_termios_device_context *base,
    const char *buf, size_t len)
{
    struct serial_class *platdata = container_of(base, struct serial_class, base);
    const struct dm_serial_ops *ops = serial_get_ops(platdata->dev);

    platdata->total = len;
    if (len > 0) {
        platdata->remaining = len;
        platdata->buf = buf;
        platdata->current = ops->fill_fifo(platdata->dev, buf, len);
        serial_class_txintr_enable(platdata, ops);
    } else {
        serial_class_txintr_disable(platdata, ops);
    }
}

static const rtems_termios_device_flow serial_flowctrl_ops = {
    .stop_remote_tx  = serial_class_flowctrl_stoptx,
    .start_remote_tx = serial_class_flowctrl_starttx
};

static const rtems_termios_device_handler serial_termios_ops = {
    .first_open     = serial_class_open,
    .last_close     = serial_class_close,
    .write          = serial_class_xmit_start,
    .set_attributes = serial_class_termios_set,
    .mode           = TERMIOS_IRQ_DRIVEN
};

static int ns16550_serial_probe(struct drvmgr_dev *dev) {
    struct ns16550_priv *priv;
    struct dev_private *devp;
    rtems_status_code sc;
    int ret = ;

    priv = rtems_calloc(1, sizeof(struct ns16550_priv));
    if (priv == NULL) 
        return -DRVMGR_NOMEM;
    devp = device_get_private(dev);
    priv->port = devp->base;
    dev->priv = priv;
    rtems_termios_device_context_initialize(&platdata->base, "UART");
    sc = rtems_termios_device_install(dev->name, &serial_termios_ops, 
        fops, &priv->base);
    if (sc != RTEMS_SUCCESSFUL) {
        printk("UART(%s) register failed: %s\n", name, 
            rtems_status_text(sc));
        ret = rtems_status_code_to_errno(sc);
        goto _free;
    }

    return 0;

}

static const struct dev_id id_table[] = {
    {.compatible = "", NULL},
    {NULL, NULL}
}

static struct drvmgr_drv_ops serial_driver_ops = {
	.init = {
		am43xx_serial_probe,
	},
	.remove = NULL,
};
		
PLATFORM_DRIVER(serial) = {
    .drv = {
        .obj_type = DRVMGR_OBJ_DRV,
        .drv_id   = DRIVER_PLATFORM_ID,
        .name     = "serial",
        .bus_type = DRVMGR_BUS_TYPE_ROOT,
        .ops      = &serial_driver_ops
    },
    .ids = id_table
};
