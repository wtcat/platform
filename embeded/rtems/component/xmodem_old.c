#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <rtems.h>
#include <rtems/sysinit.h>
#include <rtems/termiostypes.h>
#include <rtems/shell.h>

#include <sys/param.h>


#define SOH  0x01
#define STX  0x02
#define EOT  0x04
#define ACK  0x06
#define NAK  0x15
#define CAN  0x18
#define CTRLZ 0x1A

#define MAXRETRANS 25

#define XMODE_THREAD_STACK_SIZE (8 * 1024)
#define XMODEM_FILE_SIZE (10 * 1024 * 1024)
#define XMODEM_EXIT_EVENT RTEMS_EVENT_0
#define XMODE_FCACHE_SIZE 4096
#define XMODE_TIMEOUT 15 /* 1.5s */


struct param_struct {
    rtems_id parent;
    union {
        off_t offset;
        size_t size;
    };
    int file;
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
  
static uint16_t crc16_ccitt(const void *buf, int len)
{
	uint16_t crc = 0;
    int counter;
    
	for( counter = 0; counter < len; counter++)
		crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *(char *)buf++)&0x00FF];
	return crc;
}

static int check(const uint8_t *buf, int size, int is_crc)
{
    uint16_t crc, tcrc;
    uint8_t cks;
    
	if (!!is_crc) {
		crc = crc16_ccitt(buf, size);
		tcrc = (buf[size] << 8) + buf[size + 1];
		if (crc == tcrc)
			return 1;
	} else {
		cks = 0;
		for (int i = 0; i < size; ++i)
			cks += buf[i];
		if (cks == buf[size])
		    return 1;
	}
	return 0;
}

static void xflush_input(int fd)
{
    tcflush(fd, TCIFLUSH);
}

static int xgetc(int fd)
{
    char ch;
    if (read(fd, &ch, 1) == 1)
        return ch;
    return -1;
}

static int xputc(int fd, int c)
{
    uint8_t ch = c & 0xFF;
    return write(fd, &ch, 1);
}

static ssize_t xread(int fd, void *buffer, size_t size, 
    struct termios *t)
{
    void *ptr = buffer;
    int len = 0;
    int ret;

    t->c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, t);
    while (true) {
        ret = read(fd, ptr, 1029);
        if (ret <= 0)
            break;
        len += ret;
        ptr += ret;
    }

    t->c_cc[VTIME] = 15;
    tcsetattr(fd, TCSANOW, t);
    return len;
}

static ssize_t xwrite(int fd, const void *buffer, size_t size, 
    struct termios *t)
{
    const void *ptr = buffer;
    int len = 0;
    int bytes;
    int ret;

    t->c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, t);
    while (size > 0) {
        bytes = MIN(size, 1029);
        ret = write(fd, ptr, bytes);
        if (ret <= 0)
            break;
        size -= bytes;
        ptr += bytes;
        len += bytes;
    }

    t->c_cc[VTIME] = 15;
    tcsetattr(fd, TCSANOW, t);
    return len;
}

static void xmodem_rx_daemon_thread(rtems_task_argument arg)
{
    struct param_struct *param = (struct param_struct *)arg;
    uint8_t xbuff[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
    uint8_t fcache[XMODE_FCACHE_SIZE];
    int bufsz, crc = 0;
    uint8_t trychar = 'C';
    uint8_t packetno = 1;
    int c, len = 0;
    int retry = 64;
    int retrans = MAXRETRANS;
    struct termios t_old, t_new;
    size_t packet_size;
    int fd = -1;
    int ret = 0;
    int fp = 0;
    int file;

    fd = open("/dev/console", O_RDWR);
    if (fd < 0) {
        printf("%s open console failed\n", __func__);
        goto _close;
    }

    file = param->file;
    lseek(file, SEEK_SET, param->offset);

    tcgetattr(fd, &t_old);
    t_new = t_old;

    /* 115200N8, Timeout: 15 * 0.1s*/
    t_new.c_iflag = BRKINT;
    t_new.c_oflag = 0;
    t_new.c_cflag = CS8 | CREAD | CLOCAL;
    t_new.c_lflag = 0;
    t_new.c_ispeed = B115200;
    t_new.c_ospeed = B115200;
    t_new.c_cc[VMIN] = 0;
    t_new.c_cc[VTIME] = XMODE_TIMEOUT;
    tcsetattr(fd, TCSANOW, &t_new);
    xflush_input(fd);

    for ( ; ; ) {

        while (retry > 0) {
            if (trychar)
                xputc(fd, trychar);

            c = xgetc(fd);
            if (c >= 0) {
                switch (c) {
                case SOH:
                    bufsz = 128;
                    goto start_recv;
                case STX:
                    bufsz = 1024;
                    goto start_recv;
                case EOT:
                    xflush_input(fd);
                    if (fp > 0)
                        ret = write(file, fcache, fp);
                    if (ret == fp) {
                        xputc(fd, ACK);
                        ret = 0;
                    } else {
                        xputc(fd, NAK);
                        ret = 5;
                    }
                    goto _exit; /* normal end */
                case CAN:
                    c = xgetc(fd);
                    if (c == CAN) {
                        xflush_input(fd);
                        xputc(fd, ACK);
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

        xflush_input(fd);
        xputc(fd, CAN);
        xputc(fd, CAN);
        xputc(fd, CAN);
        ret = 2; /* sync error */
        goto _exit;

    start_recv:
        if (trychar == 'C') 
            crc = 1;

        trychar = 0;
        packet_size = bufsz + (crc? 1: 0) + 3;
        xbuff[0] = c;
        if (xread(fd, xbuff+1, packet_size, &t_new) != packet_size)
            goto reject;
  
        if (xbuff[1] == (uint8_t)(~xbuff[2]) && 
           (xbuff[1] == packetno || xbuff[1] == (uint8_t)packetno-1) &&
            check(&xbuff[3], bufsz, crc)) {

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
                xflush_input(fd);
                xputc(fd, CAN);
                xputc(fd, CAN);
                xputc(fd, CAN);
                ret = 3;
                goto _exit; /* too many retry error */
            }
            xputc(fd, ACK);
            continue;
        }

    reject:
        xflush_input(fd);
        xputc(fd, NAK);
    }    

_exit:
    tcsetattr(fd, TCSADRAIN, &t_old);
    xflush_input(fd);
    close(fd);
    
_close:
    printf("\n%s\n", err_code_text[ret]);
    rtems_event_send(param->parent, XMODEM_EXIT_EVENT);
    rtems_task_exit();
}

static void xmodem_tx_daemon_thread(rtems_task_argument arg)
{
    struct param_struct *param = (struct param_struct *)arg;
    struct termios t_old, t_new;
    uint8_t xbuff[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
	uint8_t packetno = 1;
    int bufsz, crc = -1;
	int i, c, len = 0;
	int retry = 16;
    size_t packet_size;
    uint16_t ccrc;
    uint8_t ccks;
    int fd = -1;
    int ret = 0;
    int file;

    fd = open("/dev/console", O_RDWR);
    if (fd < 0) {
        printf("%s open console failed\n", __func__);
        goto _close;
    }

    file = param->file;

    tcgetattr(fd, &t_old);
    t_new = t_old;

    /* 115200N8, Timeout: 15 * 0.1s*/
    t_new.c_iflag = BRKINT;
    t_new.c_oflag = 0;
    t_new.c_cflag = CS8 | CREAD | CLOCAL;
    t_new.c_lflag = 0;
    t_new.c_ispeed = B115200;
    t_new.c_ospeed = B115200;
    t_new.c_cc[VMIN] = 0;
    t_new.c_cc[VTIME] = XMODE_TIMEOUT;
    tcsetattr(fd, TCSANOW, &t_new);
    xflush_input(fd);

    for ( ; ; ) {

        while (retry) {
            c = xgetc(fd);
            if (c >= 0) {
                switch (c) {
                case 'C':
                    crc = 1;
                    goto start_trans;
                case NAK:
                    crc = 0;
                    goto start_trans;
                case CAN:
                    c = xgetc(fd);
                    if (c == CAN) {
                        xputc(fd, ACK);
                        xflush_input(fd);
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

        xputc(fd, CAN);
        xputc(fd, CAN);
        xputc(fd, CAN);
        xflush_input(fd);
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
                    if (xwrite(fd, xbuff, packet_size, &t_new) 
                        != packet_size) {
                        continue;
                    }
                        
                    c = xgetc(fd);
					if (c >= 0) {
						switch (c) {
						case ACK:
							++packetno;
							len += bufsz;
							goto start_trans;
						case CAN:
                            c = xgetc(fd);
							if (c == CAN) {
								xputc(fd, ACK);
								xflush_input(fd);
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

                xputc(fd, CAN);
                xputc(fd, CAN);
                xputc(fd, CAN);
				xflush_input(fd);
                ret = 7;
				goto _exit; /* xmit error */
                
			}else {
				for (retry = 0; retry < 10; retry++) {
					xputc(fd, EOT);
                    c = xgetc(fd);
					if (c == ACK) {
                        ret = 0;
                        goto _exit;
                    }
				}
                
				xflush_input(fd);
				ret = 7;
                goto _exit;
			}
		}
	} 

_exit:
    tcsetattr(fd, TCSADRAIN, &t_old);
    xflush_input(fd);
    close(fd);
    
_close:
    printf("%s\n", err_code_text[ret]);
    rtems_event_send(param->parent, XMODEM_EXIT_EVENT);
    rtems_task_exit();
}

static int shell_main_xmodem(int argc, char *argv[])
{
    void (*task_fn)(rtems_task_argument);
    struct param_struct param;
    rtems_task_priority prio;
    rtems_event_set event;
    rtems_status_code sc;
    rtems_name name;
    rtems_id id;
    int ret = -1;

    if (!strcmp(argv[0], "rx")) {
        if (argc == 2 || argc == 3) {
            param.parent = rtems_task_self();
            ret = open(argv[1], O_RDWR);
            if (ret < 0) {
                if (!strncmp("/dev", argv[1], 4)) {
                    printf("\"%s\" is not exist.\n", argv[1]);
                    goto out;
                }
                    
                ret = open(argv[1], O_CREAT|O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);
                if (ret < 0) { 
                    printf("%s open %s failed\n", __func__, argv[1]);
                    goto out;
                }
            }

            param.file = ret;
            if (argc == 2)
                param.offset = 0;
            else
                param.offset = strtoul(argv[2], NULL, 0);
            task_fn = xmodem_rx_daemon_thread;
        } else {
            printf("Invalid command format. rx [filepath] [offset]\n");
            ret = -EINVAL;
            goto out;
        }
    } else {
        if (argc == 2) {
            struct stat statbuf;
            
            if (!strncmp("/dev", argv[1], 4)) {
                printf("Can't send device file: \"%s\"\n", argv[1]);
                goto out;
            }

            param.parent = rtems_task_self();
            ret = open(argv[1], O_RDONLY);
            if (ret < 0) {
                printf("%s open %s failed\n", __func__, argv[1]);
                goto out; 
            }

            param.file = ret;
            ret = stat(argv[1], &statbuf);
            if (ret < 0) {
                printf("%s get file(%s) size failed\n", __func__, argv[1]);
                goto out;
            }

            param.size = statbuf.st_size;
            task_fn = xmodem_tx_daemon_thread;
        } else {
            printf("Invalid command format. rx [filepath] [offset]\n");
            ret = -EINVAL;
            goto out; 
        }
    }

    sc = rtems_task_set_priority(RTEMS_SELF, RTEMS_CURRENT_PRIORITY, 
        &prio);
    if (sc != RTEMS_SUCCESSFUL) {
        printf("error: cannot obtain the current priority: %s\n", 
            rtems_status_text (sc));         
        ret = rtems_status_code_to_errno(sc);
        goto free;
    }

    name = rtems_build_name('X', 'm', 'd', 'm');
    sc = rtems_task_create (name, prio, XMODE_THREAD_STACK_SIZE,
        RTEMS_PREEMPT | RTEMS_TIMESLICE | RTEMS_NO_ASR,
        RTEMS_LOCAL, &id);
    if (sc != RTEMS_SUCCESSFUL) {
        printf("error: cannot create helper thread: %s\n", 
            rtems_status_text (sc));     
        ret = rtems_status_code_to_errno(sc);
        goto restore_prio;
    }

    sc = rtems_task_start (id, task_fn, (rtems_task_argument)&param);
    if (sc != RTEMS_SUCCESSFUL) {
        printf("error: cannot start helper thread: %s\n", 
            rtems_status_text (sc));   
        rtems_task_delete (id);
        ret = rtems_status_code_to_errno(sc);
        goto restore_prio;
    }

    sc = rtems_event_receive(XMODEM_EXIT_EVENT,
        RTEMS_EVENT_ALL | RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event);
    if (sc != RTEMS_SUCCESSFUL) {
        ret = rtems_status_code_to_errno(sc);
        goto restore_prio;
    }

restore_prio:
    sc = rtems_task_set_priority(RTEMS_SELF, prio, &prio);
    ret = rtems_status_code_to_errno(sc);
free:
    close(param.file);
out:
    return ret;
}


rtems_shell_cmd_t shell_xmodem_command = {
    "rx",                                       /* name */
    "rx [filepath] [offset], XModem-115200N8",  /* usage */
    "rtems",                                    /* topic */
    shell_main_xmodem,                          /* command */
    NULL,                                       /* alias */
    NULL                                        /* next */
};

static void termios_resize(void) {
    rtems_termios_bufsize(2048, 2048, 2048);
}

RTEMS_SYSINIT_ITEM(termios_resize,
    RTEMS_SYSINIT_BSP_START, 
    RTEMS_SYSINIT_ORDER_MIDDLE);