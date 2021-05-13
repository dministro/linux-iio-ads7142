// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Jozsef Horvath <info@ministro.hu>
 *
 */
#define _GNU_SOURCE
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <limits.h>
#include <sys/timerfd.h>

#include "utils.h"

int file_cat(const char *file, int start, int length, void **buff)
{
	int ret = 0;
	int fd;
	unsigned char tmp_buff[1024];
	int size;
	int remaining;
	
	fd = open(file, O_RDONLY);
	if (fd == -1)
		return -errno;

	if (start) {
		ret = lseek(fd, start, SEEK_SET);
		if (ret == -1) {
			ret = -errno;
			goto out;
		}
	}

	*buff = NULL;
	size = 0;
	do {
		if (!length)
			remaining = sizeof(tmp_buff);
		else
			remaining = length - size;

		if (remaining > sizeof(tmp_buff))
			remaining = sizeof(tmp_buff);

		ret = read(fd, tmp_buff, remaining);
		if (ret > 0) {
			*buff = realloc(*buff, size + ret);
			if (*buff == NULL) {
				ret = -ENOMEM;
				goto out;
			}
			memcpy(&((unsigned char*)*buff)[size], tmp_buff, ret);
			size += ret;
		}

	} while(((!length) || (length && size < length)) && ret > 0);

	if (ret == -1)
		ret = -errno;
	else
		ret = size;
out:
	close(fd);

	return ret;
}

int file_scanf(const char *file, const char *fmt, ...)
{
	int ret = 0;
	va_list arg_list;
	FILE *fp;
	
	fp = fopen(file, "r");
	if (fp == NULL)
		return -errno;

	va_start(arg_list, fmt);
	ret = vfscanf(fp, fmt, arg_list);
	if (ret < 0)
		ret = -errno;
	fclose(fp);
	va_end(arg_list);

	return ret;
}

int file_printf(const char *file, const char *fmt, ...)
{
	int ret = 0;
	va_list arg_list;
	FILE *fp;
	
	fp = fopen(file, "w");
	if (fp == NULL)
		return -errno;

	va_start(arg_list, fmt);
	ret = vfprintf(fp, fmt, arg_list);
	if (ret >= 0)
		fflush(fp);
	va_end(arg_list);
	return ret;
}

int event_create_timer(int delay, int period)
{
	int fd = -1;
	int ret = 0;

	fd = timerfd_create(CLOCK_REALTIME, TFD_NONBLOCK);
	if (fd < 0)
		return -errno;

	ret = event_set_timer(fd, delay, period);
	if (ret) {
		close(fd);
		fd = ret;
	}
	return fd;
}

int event_set_timer(int fd, int delay, int period)
{
	int ret;
	struct timespec tm_start;
	struct itimerspec tm_set;
	long period_ns = 0;
	long period_s = 0;
	unsigned int period_ms = 100;

	if (fd < 0)
		return -EINVAL;
	
	ret = clock_gettime(CLOCK_REALTIME, &tm_start);
	if (ret)
		return ret;

	if (delay < 0) {
		memset(&tm_set, 0x00, sizeof(struct itimerspec));
	} else {
		period_ms = delay;
		if (period_ms >= 1000) {
			period_s = period_ms / 1000;
		}
		period_ns = (period_ms - (period_s * 1000)) * 1000000;
		if (period_ns < 0) {
			period_s -= 1;
			period_ns += 1000000000;
		}
		tm_set.it_value.tv_sec = tm_start.tv_sec + period_s;
		tm_set.it_value.tv_nsec = tm_start.tv_nsec + period_ns;
		if (tm_set.it_value.tv_nsec >= 1000000000) {
			tm_set.it_value.tv_sec += 1;
			tm_set.it_value.tv_nsec -= 999999999;
		}

		if (period <= 0) {
			tm_set.it_interval.tv_sec = 0;
			tm_set.it_interval.tv_nsec = 0;
		} else {
			period_s = 0;
			period_ns = 0;
			period_ms = period;
			if(period_ms >= 1000) {
				period_s = period_ms / 1000;
			}
			period_ns = (period_ms - (period_s * 1000)) * 1000000;
			if(period_ns < 0) {
				period_s -= 1;
				period_ns += 1000000000;
			}
			tm_set.it_interval.tv_sec = period_s;
			tm_set.it_interval.tv_nsec = period_ns;
		}
	}

	return timerfd_settime(fd, TFD_TIMER_ABSTIME, &tm_set, NULL);
}

