/*  $NetBSD: echo.c,v 1.12 2005/02/06 04:43:43 perry Exp $  */

/*-
 * Copyright (c) 1991, 1993
 *  The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Kenneth Almquist.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *  @(#)echo.c  8.1 (Berkeley) 5/31/93
 */

/*
 * Echo command.
 *
 * echo is steeped in tradition - several of them!
 * netbsd has supported 'echo [-n | -e] args' in spite of -e not being
 * documented anywhere.
 * Posix requires that -n be supported, output from strings containing
 * \ is implementation defined
 * The Single Unix Spec requires that \ escapes be treated as if -e
 * were set, but that -n not be treated as an option.
 * (ksh supports 'echo [-eEn] args', but not -- so that it is actually
 * impossible to actually output '-n')
 *
 * It is suggested that 'printf "%b" "string"' be used to get \ sequences
 * expanded.  printf is now a builtin of netbsd's sh and csh.
 */

/*
 * Copyright 2022 wtcat
 */
#include "configs/rtems_confdefs.h"

#ifdef CONFIGURE_SHELL_COMMAND_XECHO
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

#include <rtems.h>
#include <rtems/shell.h>

#include "shell/shell_utils.h"

static FILE *xecho_redirect_stdout(const char *path) {
    FILE *fp = NULL;
    int fd, flags;

    if (strncmp("/dev/", path, 5))
        flags = O_RDWR | O_CREAT;
    else
        flags = O_WRONLY;
    fd = open(path, flags, S_IRWXU|S_IRWXG|S_IRWXO);
    if (fd < 0) {
        errno = -EFAULT;
        goto _end;
    }
    fp = fdopen(fd, "a");
_end:
    return fp;
}

static int shell_cmd_xecho(int argc, char *argv[]) {
    FILE *fp = stdout;
    char **ap;
    char *p;
    char c;
    int count;
    int nflag = 0;
    int eflag = 0;

    ap = argv;
    if (argc)
        ap++;

    if ((p = *ap) != NULL) {
        if (!strcmp(p, "-n")) {
            nflag = 1;
            ap++;
        } else if (!strcmp(p, "-e")) {
            eflag = 1;
            ap++;
        }
    }
    if (argc >= 4) {
        if (!strcmp(argv[argc-2], ">>") ||
            !strcmp(argv[argc-2], ">")) {
            fp = xecho_redirect_stdout(argv[argc - 1]);
            if (fp == NULL)
                return -EIO;
        }
    }

    while ((p = *ap++) != NULL) {
        while ((c = *p++) != '\0') {
            if (c == '\\' && eflag) {
                switch (*p++) {
                case 'a':  c = '\a';  break;  /* bell */
                case 'b':  c = '\b';  break;
                case 'c':  return 0;    /* exit */
                case 'e':  c =  033;  break;  /* escape */
                case 'f':  c = '\f';  break;
                case 'n':  c = '\n';  break;
                case 'r':  c = '\r';  break;
                case 't':  c = '\t';  break;
                case 'v':  c = '\v';  break;
                case '\\':  break;    /* c = '\\' */
                case '0':
                    c = 0;
                    count = 3;
                    while (--count >= 0 && (unsigned)(*p - '0') < 8)
                        c = (c << 3) + (*p++ - '0');
                    break;
                default:
                    /* Output the '/' and char following */
                    p--;
                    break;
                }
            }
            fputc(c, fp);
        }
        if (*ap)
            fputc(' ', fp);
    }
    if (!nflag)
        fputc('\n', fp);
    if (fp != stdout)
        fclose(fp);
    return 0;
}

SHELL_CMDS_DEFINE(echo_cmds,
	{
		.name = "xecho",
		.usage = "xecho [args]",
		.topic = "misc",
		.command = shell_cmd_xecho
	}
);
#endif /* CONFIGURE_SHELL_COMMAND_XECHO */
