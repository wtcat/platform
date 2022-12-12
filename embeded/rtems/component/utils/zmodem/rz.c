/*
 * File      : rz.c
 * the implemention of receiving files from the remote computers
 * through the zmodem protocol.
 * Change Logs:
 * Date           Author       Notes
 * 2011-03-29     itspy
 * 2011-12-12     aozima       fixed syntax error.
 */
/*
 * Copyright 
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "zdef.h"


void zr_start(char *path);
static int zrec_init(uint8_t *rxbuf, struct zfile *zf);
static int zrec_files(struct zfile *zf);
static int zwrite_file(uint8_t *buf, uint16_t size, struct zfile *zf);
static int zrec_file_data(uint8_t *buf, struct zfile *zf);;
static int zrec_file(uint8_t *rxbuf, struct zfile *zf);
static int zget_file_info(char *name, struct zfile *zf);
static int zwrite_file(uint8_t *buf, uint16_t size, struct zfile *zf);
static void zrec_ack_bibi(void);


/* start zmodem receive proccess */
void zr_start(char *path)
{
    struct zfile *zf;
    uint8_t n;
    char ch,*p,*q;
    int res = -RT_ERROR;

    zf = malloc(sizeof(struct zfile));
    if (zf == NULL)
    {
        printf("zf: out of memory\r\n");
        return;
    }
    memset(zf, 0, sizeof(struct zfile));
    zf->fname = path;
    zf->fd = -1;
    res = zrec_files(zf);
    p = zf->fname;
    for (;;)
    {
        q = strstr(p,"/");
        if (q == NULL)  break;
        p = q+1;
    }
    if (res == 0)
    {
        printf("\b\b\bfile: %s                           \r\n",p);
        printf("size: %ld bytes\r\n",zf->bytes_received);
        printf("receive completed.\r\n");
        close(zf->fd);
        free(zf->fname);
    }
    else
    {
        printf("\b\b\bfile: %s                           \r\n",p);
        printf("size: 0 bytes\r\n");
        printf("receive failed.\r\n");
        if (zf->fd >= 0)
        {
            close(zf->fd);
            unlink(zf->fname);    /* remove this file */
            free(zf->fname);
        }
    }
    free(zf);
    /* waiting,clear console buffer */
    rt_thread_delay(RT_TICK_PER_SECOND/2);
    while(1)
    {
       n=rt_device_read(shell->device, 0, &ch, 1);
       if (n == 0) break;
    }

    return ;
}

/* receiver init, wait for ack */
static int zrec_init(uint8_t *rxbuf, struct zfile *zf)
{
    uint8_t err_cnt = 0;
    int res = -RT_ERROR;

    for (;;)
    {
        zput_pos(0L);
        tx_header[ZF0] = ZF0_CMD;
        tx_header[ZF1] = ZF1_CMD;
        tx_header[ZF2] = ZF2_CMD;
        zsend_hex_header(ZRINIT, tx_header);
again:
        res = zget_header(rx_header);
        switch(res)
        {
        case ZFILE:
             ZF0_CMD  = rx_header[ZF0];
             ZF1_CMD  = rx_header[ZF1];
             ZF2_CMD  = rx_header[ZF2];
             ZF3_CMD  = rx_header[ZF3];
             res = zget_data(rxbuf, RX_BUFFER_SIZE);
             if (res == GOTCRCW)
             {
                 if ((res =zget_file_info((char*)rxbuf,zf))!= 0)
                 {
                     zsend_hex_header(ZSKIP, tx_header);
                     return (res);
                 }
                 return 0;;
             }
             zsend_hex_header(ZNAK, tx_header);
             goto again;
        case ZSINIT:
             if (zget_data((uint8_t*)Attn, ZATTNLEN) == GOTCRCW)       /* send zack */
             {
                zsend_hex_header(ZACK, tx_header);
                goto again;
             }
             zsend_hex_header(ZNAK, tx_header);          /* send znak */
             goto again;
        case ZRQINIT:
             continue;
        case ZEOF:
             continue;
        case ZCOMPL:
             goto again;
        case ZFIN:               /* end file session */
             zrec_ack_bibi();
             return res;
         default:
              if (++err_cnt >1000) return -RT_ERROR;
              continue;
        }
    }
}

/* receive files */
static int zrec_files(struct zfile *zf)
{
    uint8_t *rxbuf;
    int res = -RT_ERROR;

    zinit_parameter();
    rxbuf = malloc(RX_BUFFER_SIZE*sizeof(uint8_t));
    if (rxbuf == NULL)
    {
         printf("rxbuf: out of memory\r\n");
         return -RT_ERROR;
    }
    printf("\r\nrz: ready...\r\n");    /* here ready to receive things */
    if ((res = zrec_init(rxbuf,zf))!= 0)
    {
         printf("\b\b\breceive init failed\r\n");
         free(rxbuf);
         return -RT_ERROR;
    }
    res = zrec_file(rxbuf,zf);
    if (res == ZFIN)
    {
        free(rxbuf);
        return 0;       /* if finish session */
    }
    else if (res == ZCAN)
    {
        free(rxbuf);
        return ZCAN;        /* cancel by sender */
    }
    else
    {
       zsend_can();
       free(rxbuf);
       return res;
    }
}
/* receive file */
static int zrec_file(uint8_t *rxbuf, struct zfile *zf)
{
    int res = - RT_ERROR;
    uint16_t err_cnt = 0;

    do
    {
        zput_pos(zf->bytes_received);
        zsend_hex_header(ZRPOS, tx_header);
again:
        res = zget_header(rx_header);
        switch (res)
        {
        case ZDATA:
             zget_pos(Rxpos);
             if (Rxpos != zf->bytes_received)
             {
                 zsend_break(Attn);
                 continue;
             }
             err_cnt = 0;
             res = zrec_file_data(rxbuf,zf);
             if (res == -RT_ERROR)
             {
                 zsend_break(Attn);
                 continue;
             }
             else if (res == GOTCAN) return res;
             else goto again;
        case ZRPOS:
             zget_pos(Rxpos);
             continue;
        case ZEOF:
             err_cnt = 0;
             zget_pos(Rxpos);
             if (Rxpos != zf->bytes_received  || Rxpos != zf->bytes_total)
             {
                 continue;
             }
             return (zrec_init(rxbuf,zf));    /* resend ZRINIT packet,ready to receive next file */
        case ZFIN:
             zrec_ack_bibi();
             return ZCOMPL;
        case ZCAN:
#ifdef ZDEBUG
             printf("error code: sender cancelled \r\n");
#endif
             zf->bytes_received = 0L;        /* throw the received data */
             return res;
        case ZSKIP:
             return res;
        case -RT_ERROR:
             zsend_break(Attn);
             continue;
        case ZNAK:
        case TIMEOUT:
        default:
            continue;
        }
    } while(++err_cnt < 100);

    return res;
}

/* proccess file infomation */
static int zget_file_info(char *name, struct zfile *zf)
{
    char *p;
    char *full_path,*ptr;
    uint16_t i,len;
    int res  = -RT_ERROR;
    struct statfs buf;
    struct stat finfo;

    if (zf->fname == NULL)              /* extract file path  */
    {
        len = strlen(name)+2;
    }
    else
        len = strlen(zf->fname)+strlen(name)+2;
    full_path = malloc(len);
    if (full_path == NULL)
    {
        zsend_can();
        printf("\b\b\bfull_path: out of memory\n");
        free(full_path);
        return -RT_ERROR;
    }
    memset(full_path,0,len);

    for (i=0,ptr=zf->fname;i<len-strlen(name)-2;i++)
         full_path[i] = *ptr++;
    full_path[len-strlen(name)-2] = '/';
    /* check if is a directory */
    if ((zf->fd=open(full_path, DFS_O_DIRECTORY,0)) < 0)
    {
        zsend_can();
        printf("\b\b\bcan not open file:%s\r\n",zf->fname+1);
        close(zf->fd);
        zf->fd = -1;
        free(full_path);
        return res;
    }
    fstat(zf->fd, &finfo);
    if ((finfo.st_mode&S_IFDIR) != S_IFDIR)
    {
        close(zf->fd);
        zf->fd = -1;
        return res;
    }
    close(zf->fd);
    /* get fullpath && file attributes */
    strcat(full_path,name);
    zf->fname = full_path;
    p = strlen(name)+name+1;
    sscanf((const char *)p, "%ld%lo%o", &zf->bytes_total,&zf->ctime,&zf->mode);
#if defined(RT_USING_DFS) && defined(DFS_USING_WORKDIR)
    dfs_statfs(working_directory,&buf);
    if (zf->bytes_total > (buf.f_blocks * buf.f_bfree))
    {
        zsend_can();
        printf("\b\b\bnot enough disk space\r\n");
        zf->fd = -1;
        free(full_path);
        return -RT_ERROR;
    }
#else
    buf = buf;
#endif
    zf->bytes_received   = 0L;
    if ((zf->fd = open(zf->fname,DFS_O_CREAT|DFS_O_WRONLY,0)) < 0)   /* create or replace exist file */
    {
        zsend_can();
        printf("\b\b\bcan not create file:%s \r\n",zf->fname);
        return -RT_ERROR;
    }

    return 0;
}

/* receive file data,continously, no ack */
static int zrec_file_data(uint8_t *buf, struct zfile *zf)
{
    int res = -RT_ERROR;

more_data:
    res = zget_data(buf,RX_BUFFER_SIZE);
    switch(res)
    {
    case GOTCRCW:                          /* zack received */
         zwrite_file(buf,Rxcount,zf);
         zf->bytes_received += Rxcount;
         zput_pos(zf->bytes_received);
         zsend_line(XON);
         zsend_hex_header(ZACK, tx_header);
         return 0;
    case GOTCRCQ:
         zwrite_file(buf,Rxcount,zf);
         zf->bytes_received += Rxcount;
         zput_pos(zf->bytes_received);
         zsend_hex_header(ZACK, tx_header);
         goto more_data;
    case GOTCRCG:
         zwrite_file(buf,Rxcount,zf);
         zf->bytes_received += Rxcount;
         goto more_data;
    case GOTCRCE:
         zwrite_file(buf,Rxcount,zf);
         zf->bytes_received += Rxcount;
         return 0;
    case GOTCAN:
#ifdef ZDEBUG
         printf("error code : ZCAN \r\n");
#endif
         return res;
    case TIMEOUT:
         return res;
    case -RT_ERROR:
         zsend_break(Attn);
         return res;
    default:
         return res;
    }
}

/* write file */
static int zwrite_file(uint8_t *buf,uint16_t size, struct zfile *zf)
{
    return (write(zf->fd,buf,size));
}

/* ack bibi */
static void zrec_ack_bibi(void)
{
    uint8_t i;

    zput_pos(0L);
    for (i=0;i<3;i++)
    {
        zsend_hex_header(ZFIN, tx_header);
        switch (zread_line(100))
        {
        case 'O':
             zread_line(1);
             return;
        case RCDO:
             return;
        case TIMEOUT:
        default:
             break;
        }
    }
}

/* end of rz.c */
