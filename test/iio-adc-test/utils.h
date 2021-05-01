// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Jozsef Horvath <info@ministro.hu>
 *
 */
#ifndef _UTILS_H_
#define _UTILS_H_

int file_scanf(const char *file, const char *fmt, ...)
	__attribute__ ((format (scanf, 2, 3)));
int file_printf(const char *file, const char *fmt, ...)
	__attribute__ ((format (printf, 2, 3)));
int event_create_timer(int delay, int period);
int event_set_timer(int fd, int delay, int period);

#endif
