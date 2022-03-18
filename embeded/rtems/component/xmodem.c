#include "bsp/sysconf.h"
#ifdef CONFIGURE_SHELL_COMMAND_XMODEM
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <rtems.h>
#include <rtems/malloc.h>
#include <rtems/sysinit.h>
#include <rtems/termiostypes.h>
#include <rtems/shell.h>

#include <sys/param.h>

#define __need_getopt_newlib
#include <getopt.h>

#define SOH  0x01
#define STX  0x02
#define EOT  0x04
#define ACK  0x06
#define NAK  0x15
#define CAN  0x18
#define CTRLZ 0x1A

#define MAXRETRANS 25
#define XMODEM_FILE_SIZE (10 * 1024 * 1024)
#define XMODEM_EXIT_EVENT RTEMS_EVENT_0
#define XMODE_FCACHE_SIZE 4096
#define XMODE_TIMEOUT 15 /* 1.5s */

struct param_struct {
    struct termios t;
    struct termios t_new;
    union {
        off_t offset;
        size_t size;
    };
    int file;
    int fdev;
};

static const char *err_code_text[] = {
    [0] = "Completed!",
    [1] = "Canceled by remote",
    [2] = "Sync error",
    [3] = "Too many retry error",
    [4] = "Abort",
    [5] = "File I/O error",
    [6] = "File is too large",
    [7] = "Transmit error",
};

static const unsigned short crc16tab[256]= {
	0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
	0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
	0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
	0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
	0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
	0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
	0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
	0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
	0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
	0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
	0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
	0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
	0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
	0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
	0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
	0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
	0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
	0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
	0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
	0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
	0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
	0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
	0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
	0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
	0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
	0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
	0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
	0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
	0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
	0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
	0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
	0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};
  
static uint16_t crc16_ccitt(const void *buf, int len) {
	uint16_t crc = 0;
	for(int counter = 0; counter < len; counter++)
		crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *(char *)buf++)&0x00FF];
	return crc;
}

static int xm_check(const uint8_t *buf, int size, int is_crc) {
	if (!!is_crc) {
		uint16_t crc = crc16_ccitt(buf, size);
		uint16_t tcrc = (buf[size] << 8) + buf[size + 1];
		if (crc == tcrc)
			return 1;
	} else {
		uint8_t cks = 0;
		for (int i = 0; i < size; ++i)
			cks += buf[i];
		if (cks == buf[size])
		    return 1;
	}
	return 0;
}

static inline void xm_input_flush(int fd) {
    tcflush(fd, TCIFLUSH);
}

static int xm_getc(int fd) {
    char ch;
    if (read(fd, &ch, 1) == 1)
        return ch;
    return -1;
}

static int xm_putc(int fd, int c) {
    uint8_t ch = c & 0xFF;
    return write(fd, &ch, 1);
}

static ssize_t xm_read(int fd, void *buffer, size_t size, 
    struct termios *t) {
    void *ptr = buffer;
    int len = 0;
    t->c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, t);
    while (true) {
        int ret = read(fd, ptr, 1029);
        if (ret <= 0)
            break;
        len += ret;
        ptr += ret;
    }
    t->c_cc[VTIME] = 15;
    tcsetattr(fd, TCSANOW, t);
    return len;
}

static ssize_t xm_write(int fd, const void *buffer, size_t size, 
    struct termios *t) {
    const void *ptr = buffer;
    int len = 0;
    t->c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, t);
    while (size > 0) {
        int bytes = MIN(size, 1029);
        if (write(fd, ptr, bytes) <= 0)
            break;
        size -= bytes;
        ptr += bytes;
        len += bytes;
    }
    t->c_cc[VTIME] = 15;
    tcsetattr(fd, TCSANOW, t);
    return len;
}

static int xm_receive(struct param_struct *param) {
    uint8_t xbuff[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
    char *fcache;
    int bufsz, crc = 0;
    uint8_t trychar = 'C';
    uint8_t packetno = 1;
    int c, len = 0;
    int retry = 64;
    int retrans = MAXRETRANS;
    struct termios t_old, t_new;
    size_t packet_size;
    int fd;
    int ret = 0;
    int fp = 0;
    int file;
    
    fcache = rtems_malloc(XMODE_FCACHE_SIZE);
    if (fcache == NULL)
        return -ENOMEM;
    fd = param->fdev;
    file = param->file;
    xm_input_flush(fd);
    for ( ; ; ) {
        while (retry > 0) {
            if (trychar)
                xm_putc(fd, trychar);
            c = xm_getc(fd);
            if (c >= 0) {
                switch (c) {
                case SOH:
                    bufsz = 128;
                    goto start_recv;
                case STX:
                    bufsz = 1024;
                    goto start_recv;
                case EOT:
                    xm_input_flush(fd);
                    if (fp > 0)
                        ret = write(file, fcache, fp);
                    if (ret == fp) {
                        xm_putc(fd, ACK);
                        ret = 0;
                    } else {
                        xm_putc(fd, NAK);
                        ret = 5;
                    }
                    goto _exit; /* normal end */
                case CAN:
                    c = xm_getc(fd);
                    if (c == CAN) {
                        xm_input_flush(fd);
                        xm_putc(fd, ACK);
                        ret = 1;
                        goto _exit; /* canceled by remote */
                    }
                    break;
                case 'Q':
                case 'q':
                    ret = 4;
                    goto _exit;
                default:
                    break;
                }
            }
            retry--;
        }
        if (trychar == 'C') { 
            trychar = NAK; 
            continue; 
        }
        xm_input_flush(fd);
        xm_putc(fd, CAN);
        xm_putc(fd, CAN);
        xm_putc(fd, CAN);
        ret = 2; /* sync error */
        goto _exit;

    start_recv:
        if (trychar == 'C') 
            crc = 1;

        trychar = 0;
        packet_size = bufsz + (crc? 1: 0) + 3;
        xbuff[0] = c;
        if (xm_read(fd, xbuff+1, packet_size, &param->t_new) != packet_size)
            goto reject;
        if (xbuff[1] == (uint8_t)(~xbuff[2]) && 
           (xbuff[1] == packetno || xbuff[1] == (uint8_t)packetno-1) &&
            xm_check(&xbuff[3], bufsz, crc)) {
            if (xbuff[1] == packetno) {
                int count = XMODEM_FILE_SIZE - len;
                if (count > bufsz) 
                    count = bufsz;
                if (count > 0) {
                    memcpy(fcache+fp, &xbuff[3], count);
                    fp += count;
                    if (fp == XMODE_FCACHE_SIZE) {
                        ret = write(file, fcache, fp);
                        if (ret != fp) {
                            ret = 5;
                            goto _exit;
                        }
                        
                        fp = 0;
                        ret = 0;
                    }
                    len += count;
                } else {
                    ret = 6;
                    goto _exit;
                }

                ++packetno;
                retrans = MAXRETRANS + 1;
            }
            if (--retrans <= 0) {
                xm_input_flush(fd);
                xm_putc(fd, CAN);
                xm_putc(fd, CAN);
                xm_putc(fd, CAN);
                ret = 3;
                goto _exit; /* too many retry error */
            }
            xm_putc(fd, ACK);
            continue;
        }
    reject:
        xm_input_flush(fd);
        xm_putc(fd, NAK);
    }    
_exit:
    free(fcache);
    xm_input_flush(fd);
_close:
    printf("\n%s\n", err_code_text[ret]);
    return ret;
}

static int xm_send(struct param_struct *param) {
    uint8_t xbuff[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
	uint8_t packetno = 1;
    int bufsz, crc = -1;
	int i, c, len = 0;
	int retry = 16;
    size_t packet_size;
    uint16_t ccrc;
    uint8_t ccks;
    int fd;
    int ret = 0;
    int file;

    fd = param->fdev;
    file = param->file;
    xm_input_flush(fd);
    for ( ; ; ) {
        while (retry) {
            c = xm_getc(fd);
            if (c >= 0) {
                switch (c) {
                case 'C':
                    crc = 1;
                    goto start_trans;
                case NAK:
                    crc = 0;
                    goto start_trans;
                case CAN:
                    c = xm_getc(fd);
                    if (c == CAN) {
                        xm_putc(fd, ACK);
                        xm_input_flush(fd);
                        ret = 1;
                        goto _exit; /* canceled by remote */
                    }
                    break;
                case 'Q':
                case 'q':
                    ret = 4;
                    goto _exit;
                default:
                    break;
                }
            }
        }

        xm_putc(fd, CAN);
        xm_putc(fd, CAN);
        xm_putc(fd, CAN);
        xm_input_flush(fd);
        ret = 2;
        goto _exit; /* no sync */

		for ( ; ; ) {
start_trans:
			xbuff[1] = packetno;
			xbuff[2] = ~packetno;
			c = param->size - len;
            if (c >= 1024) {
                bufsz = 1024;
                xbuff[0] = STX; 
            } else {
                bufsz = 128;
    			xbuff[0] = SOH; 
            }
			if (c > bufsz) 
                c = bufsz;
			if (c >= 0) {
				memset(&xbuff[3], 0, bufsz);
                if (read(file, &xbuff[3], c) != c) {
                    ret = 5;
                    goto _exit;
                }
				if (c < bufsz)
					xbuff[3] = CTRLZ;
				if (crc) {
					ccrc = crc16_ccitt(&xbuff[3], bufsz);
					xbuff[bufsz + 3] = (ccrc >> 8) & 0xFF;
					xbuff[bufsz + 4] = ccrc & 0xFF;
				} else {
					ccks = 0;
					for (i = 3; i < bufsz + 3; ++i) 
						ccks += xbuff[i];
					xbuff[bufsz + 3] = ccks;
				}

                packet_size = bufsz + 4 + (crc? 1: 0);
				for (retry = 0; retry < MAXRETRANS; retry++) {
                    if (xm_write(fd, xbuff, packet_size, &param->t_new) 
                        != packet_size) {
                        continue;
                    }
                    c = xm_getc(fd);
					if (c >= 0) {
						switch (c) {
						case ACK:
							++packetno;
							len += bufsz;
							goto start_trans;
						case CAN:
                            c = xm_getc(fd);
							if (c == CAN) {
								xm_putc(fd, ACK);
								xm_input_flush(fd);
								ret = 1; /* canceled by remote */
                                goto _exit;
							}
							break;
						case NAK:
						default:
							break;
						}
					}
				}
                xm_putc(fd, CAN);
                xm_putc(fd, CAN);
                xm_putc(fd, CAN);
				xm_input_flush(fd);
                ret = 7;
				goto _exit; /* xmit error */
			} else {
				for (retry = 0; retry < 10; retry++) {
					xm_putc(fd, EOT);
                    c = xm_getc(fd);
					if (c == ACK) {
                        ret = 0;
                        goto _exit;
                    }
				}
				xm_input_flush(fd);
				ret = 7;
                goto _exit;
			}
		}
	} 
_exit:
    xm_input_flush(fd);
_close:
    printf("%s\n", err_code_text[ret]);
    return ret;
}

static int xm_open_console(struct param_struct *param) {
    int fd = open("/dev/console", O_RDWR);
    if (fd < 0) {
        printf("%s open console failed\n", __func__);
        return -1;
    }
    tcgetattr(fd, &param->t_new);
    param->t = param->t_new;
    param->fdev = fd;
    param->t_new.c_iflag = BRKINT;
    param->t_new.c_oflag = 0;
    param->t_new.c_cflag = CS8 | CREAD | CLOCAL;
    param->t_new.c_lflag = 0;
    param->t_new.c_cc[VMIN] = 0;
    param->t_new.c_cc[VTIME] = XMODE_TIMEOUT;
    tcsetattr(fd, TCSANOW, &param->t_new);
    return 0;
}

static void xm_close_console(struct param_struct *param) {
    tcsetattr(param->fdev, TCSADRAIN, &param->t);
    close(param->fdev);
}

static const char help_usage[] = {
    "Usage:\n"
    "xm -f file_name [-t] [-o offset]\n"
    "   -t send file"
};

static int shell_main_xm(int argc, char *argv[]) {
    int (*fn_exec)(struct param_struct *param) = xm_receive;
    struct param_struct param;
    struct getopt_data getopt_reent;
    const char *fname = NULL;
    off_t offset = 0;
    int ch;
    int permission;
    int ret;

    if (argc < 2) {
        printf(help_usage);
        return 0;
    }
    memset(&getopt_reent, 0, sizeof(getopt_data));
    memset(&param, 0, sizeof(param));
    while ((ch = getopt_r(argc, argv, "f:o:s:ht", &getopt_reent)) != -1) {
        switch(ch) {
        case 'f':
            fname = getopt_reent.optarg;
            break;
        case 'o':
            param.offset = (size_t)strtoul(getopt_reent.optarg, NULL, 0);
            break;
        case 't':
            fn_exec = xm_send;
            break;
        case 'h':
            printf(help_usage);
            return 0;
        default:
            break;
        }
    }

    if (!fname)
        return -EINVAL;
    if (!access(fname, F_OK)) {
        permission = O_RDWR;
    } else {
        if (fn_exec == xm_send) {
            printf("Not found file(%s)\n", fname);
            return -EINVAL;
        }
        permission = O_CREAT|O_RDWR;
    }
    if (fn_exec == xm_send) {
        struct stat statbuf;
        if (stat(fname, &statbuf) < 0)
            return -EIO;
        param.size = statbuf.st_size;
    }
    printf("xmodem 33333...\n");
    param.file = open(fname, permission);
    if (param.file < 0)
        return -EIO;
     printf("xmodem 4444...\n");
    if (!xm_open_console(&param)) {
         printf("xmodem 5555...\n");
        ret = fn_exec(&param);
        xm_close_console(&param);
    }
_close_f:
    close(param.file);
    return ret;
}

rtems_shell_cmd_t shell_xmodem_command = {
    "xm",                                       /* name */
    help_usage,  /* usage */
    "rtems",                                    /* topic */
    shell_main_xm,                          /* command */
    NULL,                                       /* alias */
    NULL                                        /* next */
};

static void termios_resize(void) {
    rtems_termios_bufsize(2048, 2048, 2048);
}

RTEMS_SYSINIT_ITEM(termios_resize,
    RTEMS_SYSINIT_BSP_START, 
    RTEMS_SYSINIT_ORDER_MIDDLE);
#endif /* CONFIGURE_SHELL_COMMAND_XMODEM */
