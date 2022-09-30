/*
 * Copyright 2022 wtcat
 */
#include "shell/shell_utils.h"

static int shell_cmd_gtest(int argc, char **argv) {
	extern int gtest_main(int argc, char **argv);
    return gtest_main(argc, argv);
}

SHELL_CMDS_DEFINE(gtest_cmds,
	{
		.name = "gtest_run",
		.usage = " Run gtest test framework",
		.topic = "misc",
		.command = shell_cmd_gtest
	}
);