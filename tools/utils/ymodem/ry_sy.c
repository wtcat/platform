/*
 * Copyright (c) 2019 Fuzhou Rockchip Electronics Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-12-09     Steven Liu   the first version
 * 2021-04-14     Meco Man     Check the file path's legitimacy of 'sy' command
 */

 /*
  * Copyright 2022 wtcat
  */
#include <errno.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <ymodem.h>


struct custom_ctx
{
    struct rym_ctx parent;
    int fd;
    int flen;
    char fpath[];
};

static enum rym_code _rym_recv_begin(
    struct rym_ctx *ctx,
    uint8_t *buf,
    size_t len)
{
    struct custom_ctx *cctx = (struct custom_ctx *)ctx;

    cctx->fpath[0] = '/';
    strncpy(&(cctx->fpath[1]), (const char *)buf, len - 1);
    cctx->fd = open(cctx->fpath, O_CREAT | O_WRONLY | O_TRUNC, 0);
    if (cctx->fd < 0)
    {
        printf("error creating file: %d\n", errno);
        return RYM_CODE_CAN;
    }
    cctx->flen = atoi(1 + (const char *)buf + strnlen((const char *)buf, len - 1));
    if (cctx->flen == 0)
        cctx->flen = -1;

    return RYM_CODE_ACK;
}

static enum rym_code _rym_recv_data(
    struct rym_ctx *ctx,
    uint8_t *buf,
    size_t len)
{
    struct custom_ctx *cctx = (struct custom_ctx *)ctx;

    assert(cctx->fd >= 0);
    if (cctx->flen == -1)
    {
        write(cctx->fd, buf, len);
    }
    else
    {
        int wlen = len > cctx->flen ? cctx->flen : len;
        write(cctx->fd, buf, wlen);
        cctx->flen -= wlen;
    }

    return RYM_CODE_ACK;
}

static enum rym_code _rym_recv_end(
    struct rym_ctx *ctx,
    uint8_t *buf,
    size_t len)
{
    struct custom_ctx *cctx = (struct custom_ctx *)ctx;

    assert(cctx->fd >= 0);
    close(cctx->fd);
    cctx->fd = -1;

    return RYM_CODE_ACK;
}

static enum rym_code _rym_send_begin(
    struct rym_ctx *ctx,
    uint8_t *buf,
    size_t len)
{
    struct custom_ctx *cctx = (struct custom_ctx *)ctx;
    struct stat file_buf;
    char insert_0 = '\0';
    int err;

    cctx->fd = open(cctx->fpath, O_RDONLY);
    if (cctx->fd < 0)
    {
        err = rt_get_errno();
        printf("error open file: %d\n", err);
        return RYM_ERR_FILE;
    }
    rt_memset(buf, 0, len);
    err = stat(cctx->fpath, &file_buf);
    if (err != RT_EOK)
    {
        printf("error open file.\n");
        return RYM_ERR_FILE;
    }
    rt_sprintf((char *)buf, "%s%c%d", (char *) & (cctx->fpath[1]), insert_0, file_buf.st_size);

    return RYM_CODE_SOH;
}

static enum rym_code _rym_send_data(
    struct rym_ctx *ctx,
    uint8_t *buf,
    size_t len)
{
    struct custom_ctx *cctx = (struct custom_ctx *)ctx;
    size_t read_size;
    int retry_read;

    read_size = 0;
    for (retry_read = 0; retry_read < 10; retry_read++)
    {
        read_size += read(cctx->fd, buf + read_size, len - read_size);
        if (read_size == len)
            break;
    }

    if (read_size < len)
    {
        rt_memset(buf + read_size, 0x1A, len - read_size);
        ctx->stage = RYM_STAGE_FINISHING;
    }

    return RYM_CODE_SOH;
}

static enum rym_code _rym_send_end(
    struct rym_ctx *ctx,
    uint8_t *buf,
    size_t len)
{
    rt_memset(buf, 0, len);

    return RYM_CODE_SOH;
}

int rym_download_file(rt_device_t idev)
{
    struct custom_ctx *ctx = calloc(1, sizeof(*ctx));
    int err;

    if (!ctx)
    {
        printf("rt_malloc failed\n");
        return RT_ENOMEM;
    }
    ctx->fd = -1;
    assert(idev);
    err = rym_recv_on_device(&ctx->parent, idev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                             _rym_recv_begin, _rym_recv_data, _rym_recv_end, 1000);
    free(ctx);
    return err;
}

int rym_upload_file(rt_device_t idev, const char *file_path)
{
    int err = 0;

    struct custom_ctx *ctx = calloc(1, sizeof(*ctx));
    if (!ctx)
    {
        printf("rt_malloc failed\n");
        return RT_ENOMEM;
    }
    ctx->fd = -1;
    strncpy(ctx->fpath, file_path, DFS_PATH_MAX);
    assert(idev);
    err = rym_send_on_device(&ctx->parent, idev,
                             RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                             _rym_send_begin, _rym_send_data, _rym_send_end, 1000);
    free(ctx);

    return err;
}
