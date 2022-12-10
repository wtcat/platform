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
#define pr_fmt(fmt) "ry_sy: "fmt

#include <errno.h>
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include "base/ymodem.h"
#include "base/log.h"


struct custom_ctx
{
#define RYM_MAX_PATH 256
    struct rym_ctx parent;
    int fd;
    int flen;
    off_t offset;
    char *root;
    char fpath[RYM_MAX_PATH];
};

static enum rym_code _rym_recv_begin(
    struct rym_ctx *ctx,
    uint8_t *buf,
    size_t len)
{
    struct custom_ctx *cctx = (struct custom_ctx *)ctx;
    size_t plen = strlen(cctx->root);

    if (plen + len + 1 >= RYM_MAX_PATH)
    {
        pr_err("file path is too long!\n");
        return RYM_CODE_CAN;
    }

    if (!strncmp(cctx->root, "/dev/", 4))
    {
        cctx->fd = open(cctx->fpath, O_WRONLY);
        if (cctx->fd < 0)
        {
            pr_err("Open file error(%s)\n", cctx->fpath);
            return RYM_CODE_CAN;
        }

        if (cctx->offset)
            lseek(cctx->fd, cctx->offset, SEEK_SET);
    }
    else 
    {
        const char *ss = cctx->root + plen - 1;
        bool is_file = false;
        bool append_split = false;

        if (*ss != '/') 
        {
            ss--;
            while (ss >= cctx->root)
            {
                if (*ss == '.')
                {
                    is_file = true;
                    break;
                }
                if (*ss == '/')
                {
                    break;
                }
                ss--;
            }
            append_split = !is_file;
        }
        if (!is_file)
        {
            strcpy(cctx->fpath, cctx->root);
            if (append_split)
                cctx->fpath[plen++] = '/';
            memcpy(&cctx->fpath[plen], (const char *)buf, len - 1);
        }

        cctx->fd = open(cctx->fpath, O_CREAT | O_WRONLY | O_TRUNC, 
            S_IRWXU|S_IRWXG|S_IRWXO);
        if (cctx->fd < 0)
        {
            pr_err("error creating file(%s)\n", cctx->fpath);
            return RYM_CODE_CAN;
        }
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
    if (cctx->flen < 0)
    {
        write(cctx->fd, buf, len);
    }
    else
    {
        size_t wlen = len > (size_t)cctx->flen ? (size_t)cctx->flen : len;
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
    (void) buf;
    (void) len;
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
        pr_err("error open file: %d\n", errno);
        return RYM_ERR_FILE;
    }
    memset(buf, 0, len);
    err = stat(cctx->fpath, &file_buf);
    if (err)
    {
        pr_err("error open file.\n");
        return RYM_ERR_FILE;
    }
    sprintf((char *)buf, "%s%c%d", (char *) & (cctx->fpath[1]), insert_0, (int)file_buf.st_size);

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
        memset(buf + read_size, 0x1A, len - read_size);
        ctx->stage = RYM_STAGE_FINISHING;
    }

    return RYM_CODE_SOH;
}

static enum rym_code _rym_send_end(
    struct rym_ctx *ctx,
    uint8_t *buf,
    size_t len)
{
    (void) ctx;
    memset(buf, 0, len);

    return RYM_CODE_SOH;
}

int rym_download_file(const char *idev, const char *path, off_t offset)
{
    struct custom_ctx *ctx;
    int err;

    ctx = calloc(1, sizeof(*ctx));
    if (!ctx)
    {
        pr_err("No more memory\n");
        return -ENOMEM;
    }

    if (!path)
    {
        ctx->root = "/";
    }

    ctx->fd = -1;
    ctx->offset = offset;
    assert(idev);
    err = rym_recv_on_device(&ctx->parent, idev, 
                             _rym_recv_begin, _rym_recv_data, _rym_recv_end, 1000);
    free(ctx);
    return err;
}

int rym_upload_file(const char *idev, const char *file_path)
{
    int err = 0;

    struct custom_ctx *ctx = calloc(1, sizeof(*ctx));
    if (!ctx)
    {
        pr_err("No more memory\n");
        return -ENOMEM;
    }
    ctx->fd = -1;
    strncpy(ctx->fpath, file_path, RYM_MAX_PATH - 1);
    assert(idev);
    err = rym_send_on_device(&ctx->parent, idev,
                             _rym_send_begin, _rym_send_data, _rym_send_end, 1000);
    free(ctx);

    return err;
}
