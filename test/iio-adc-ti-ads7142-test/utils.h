// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Jozsef Horvath <info@ministro.hu>
 *
 */
#ifndef _UTILS_H_
#define _UTILS_H_

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

int file_cat(const char *file, int start, int length, void **buff);
int file_scanf(const char *file, const char *fmt, ...)
	__attribute__ ((format (scanf, 2, 3)));
int file_printf(const char *file, const char *fmt, ...)
	__attribute__ ((format (printf, 2, 3)));
int event_create_timer(int delay, int period);
int event_set_timer(int fd, int delay, int period);

static inline int check_prefix(const char *str, const char *prefix)
{
	return strlen(str) > strlen(prefix) &&
	       strncmp(str, prefix, strlen(prefix)) == 0;
}

static inline int check_postfix(const char *str, const char *postfix)
{
	return strlen(str) > strlen(postfix) &&
	       strcmp(str + strlen(str) - strlen(postfix), postfix) == 0;
}

#endif
