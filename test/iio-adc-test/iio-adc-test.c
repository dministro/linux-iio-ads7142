// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Jozsef Horvath <info@ministro.hu>
 *
 */
#define _GNU_SOURCE
#include <stdlib.h>
#include <getopt.h>
#include <signal.h>
#include <unistd.h>
#include <dirent.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/iio/events.h>
#include <linux/iio/types.h>
#include <sys/eventfd.h>
#include <time.h>

#include "utils.h"

const char* g_short_options = "d:c:mp:r:f:h";
const struct option g_long_options[] = {
	{ "iio-device", required_argument, NULL, 'd' },
	{ "adc-channel", required_argument, NULL, 'c' },
	{ "monitor", no_argument, NULL, 'm' },
	{ "threshold-low", required_argument, NULL, 'f' },
	{ "threshold-high", required_argument, NULL, 'r' },
	{ "hysteresis", required_argument, NULL, 257 },
	{ "poll-period", required_argument, NULL, 'p'},
	{ "help", no_argument, NULL, 'h' },
	{ NULL, 0, NULL, 0 },
};

static struct {
	int iio_device;
	int adc_channel;
	bool monitor;
	int threshold_low;
	bool threshold_low_en;
	int threshold_high;
	bool threshold_high_en;
	int hysteresis;
	bool hysteresis_set;

	int poll_period_ms;

	int stop_fd;
	int event_fd;

	float channel_offset;
	float channel_scale;
} g_app_ctx = {
	.poll_period_ms = 500,
	.channel_offset = 0,
	.channel_scale = 1,
};

static void print_help(FILE* output)
{
	fprintf(output,
		"\niio-adc-test - ADC command line test utility\n"
		"usage: iio-adc-test [options]\n"
		"\n"
		"  -d, --iio-device=DEVICE_INDEX\n"
		"  -c, --adc-channel=CHANNEL_INDEX\n"
		"  -m, --monitor\n"
		"  -f, --threshold-low=LOWER_THRESHOLD\n"
		"  -r, --threshold-high=HIGHER_THRESHOLD\n"
		"      --hysteresis=HYSTERESIS\n"
		"  -p, --poll-period=POLL_PERIOD\n"
		"  -h, --help\n"
	);
}

static void print_help_exit(FILE* output, int exit_code)
{
	print_help(output);
	exit(exit_code);
}

static int get_device_event_fd(int device, int *event_fd)
{
	int ret;
	char* chrdev_path;
	int chrdev_fd;

	ret = asprintf(&chrdev_path, "/dev/iio:device%d", device);
	if (ret < 0)
		return -ENOMEM;

	chrdev_fd = open(chrdev_path, 0);
	if (chrdev_fd == -1) {
		ret = -errno;
		fprintf(stderr, "device(%s) open error(%d)\n", chrdev_path, errno);
	} else {
		ret = ioctl(chrdev_fd, IIO_GET_EVENT_FD_IOCTL, event_fd);
		if (ret == -1 || *event_fd == -1) {
			ret = -errno;
			if (ret == -ENODEV)
				fprintf(stderr,
					"this device does not support events\n");
			else
				fprintf(stderr, "failed to retrive event fd\n");
		}

		close(chrdev_fd);
	}

	free(chrdev_path);
	return ret;
}

static int get_device_channel_attr_float(int device, int channel, const char* attr_name, float* value)
{
	int ret;
	char* attr_path;

	ret = asprintf(&attr_path, "/sys/bus/iio/devices/iio:device%d/%s%d_%s",
		       device, "in_voltage", channel, attr_name);
	if (ret < 0)
		return -ENOMEM;
	
	ret = file_scanf(attr_path, "%f", value);
	free(attr_path);
	return ret;
}

static int set_device_channel_event_attr_int(int device, int channel, const char* attr_name, int value)
{
	int ret;
	char* attr_path;

	ret = asprintf(&attr_path, "/sys/bus/iio/devices/iio:device%d/events/%s%d_%s",
		       device, "in_voltage", channel, attr_name);
	if (ret < 0)
		return -ENOMEM;
	
	ret = file_printf(attr_path, "%d\n", value);
	free(attr_path);
	return ret > 0 ? 0 : -1;
}

static int print_conversion_result(const struct iio_event_data *event_data,
				   float* conversion_result)
{
	char time_string[100];
	time_t now = time(NULL);
	struct tm tm_now;
	float scaled_result;
	enum iio_event_direction dir;

	now = time(NULL);
	localtime_r(&now, &tm_now);
	strftime(time_string, sizeof(time_string), "%F %T", &tm_now);

	printf("%s\t", time_string);

	if (g_app_ctx.monitor && event_data) {
		dir = IIO_EVENT_CODE_EXTRACT_DIR(event_data->id);
		printf("threshold %s\t",
		       dir == IIO_EV_DIR_RISING ? "rising" : "falling");
	}

	if (conversion_result) {
		scaled_result = *conversion_result * g_app_ctx.channel_scale +
				g_app_ctx.channel_offset;
		printf("raw: %0.0f\tscaled: %0.3f",
		       *conversion_result, scaled_result);
	}

	printf("\n");
	return 0;
}

static int configure_device_events(void)
{
	int ret;

	if (g_app_ctx.threshold_low_en) {
		ret = set_device_channel_event_attr_int(g_app_ctx.iio_device,
							g_app_ctx.adc_channel,
				      			"thresh_falling_value",
							g_app_ctx.threshold_low);
		if (ret) {
			printf("set thresh_falling_value(%d) error(%d)",
			       g_app_ctx.threshold_low, ret);
			return ret;
		}
	}
	ret = set_device_channel_event_attr_int(g_app_ctx.iio_device,
						g_app_ctx.adc_channel,
						"thresh_falling_en",
						g_app_ctx.threshold_low_en ? 1 : 0);
	if (ret) {
		printf("set thresh_falling_en(%d) error(%d)",
		       g_app_ctx.threshold_low_en ? 1 : 0, ret);
		return ret;
	}

	if (g_app_ctx.threshold_high_en) {
		ret = set_device_channel_event_attr_int(g_app_ctx.iio_device,
							g_app_ctx.adc_channel,
				      			"thresh_rising_value",
							g_app_ctx.threshold_high);
		if (ret) {
			printf("set thresh_rising_value(%d) error(%d)",
			       g_app_ctx.threshold_high, ret);
			return ret;
		}
	}
	ret = set_device_channel_event_attr_int(g_app_ctx.iio_device,
						g_app_ctx.adc_channel,
						"thresh_rising_en",
						g_app_ctx.threshold_high_en ? 1 : 0);
	if (ret) {
		printf("set thresh_rising_en(%d) error(%d)",
		       g_app_ctx.threshold_high_en ? 1 : 0, ret);
		return ret;
	}

	if (g_app_ctx.hysteresis_set) {
		ret = set_device_channel_event_attr_int(g_app_ctx.iio_device,
							g_app_ctx.adc_channel,
				      			"thresh_either_hysteresis",
							g_app_ctx.hysteresis);
		if (ret) {
			printf("set thresh_either_hysteresis(%d) error(%d)",
			       g_app_ctx.hysteresis, ret);
			return ret;
		}
	}

	return 0;
}

int main (int argc, char ** argv)
{
	int next_option;
	int ret = 0;
	struct pollfd poll_fds[2];
	uint64_t event_buff;
	int p_ret;
	float conversion_result;
	struct iio_event_data event_data;

	while ((next_option = getopt_long(argc, argv, g_short_options, g_long_options, NULL)) != -1) {
		switch (next_option) {
		case 'd':
			if (optarg == NULL)
				print_help_exit(stderr, EXIT_FAILURE);
			
			g_app_ctx.iio_device = atoi(optarg);
		break;
		case 'c':
			if (optarg == NULL)
				print_help_exit(stderr, EXIT_FAILURE);
			
			g_app_ctx.adc_channel = atoi(optarg);
		break;
		case 'm':
			g_app_ctx.monitor = true;
		break;
		case 'f':
			if (optarg == NULL)
				print_help_exit(stderr, EXIT_FAILURE);
			
			g_app_ctx.threshold_low = atoi(optarg);
			g_app_ctx.threshold_low_en = true;
		break;
		case 'r':
			if (optarg == NULL)
				print_help_exit(stderr, EXIT_FAILURE);
			
			g_app_ctx.threshold_high = atoi(optarg);
			g_app_ctx.threshold_high_en = true;
		break;
		case 257:
			if (optarg == NULL)
				print_help_exit(stderr, EXIT_FAILURE);
			
			g_app_ctx.hysteresis = atoi(optarg);
			g_app_ctx.hysteresis_set = true;
		break;
		case 'p':
			if (optarg == NULL)
				print_help_exit(stderr, EXIT_FAILURE);
			
			g_app_ctx.poll_period_ms = atoi(optarg);
		break;
		default:
			print_help_exit(stderr, EXIT_FAILURE);
		break;
		}
	}

	get_device_channel_attr_float(g_app_ctx.iio_device,
				      g_app_ctx.adc_channel,
				      "scale", &g_app_ctx.channel_scale);
	get_device_channel_attr_float(g_app_ctx.iio_device,
				      g_app_ctx.adc_channel,
				      "offset", &g_app_ctx.channel_offset);

	g_app_ctx.stop_fd = eventfd(0, EFD_NONBLOCK);
	if (g_app_ctx.stop_fd < 0) {
		fprintf(stderr, "stop_fd create error(%d)\n", errno);
		ret = -errno;
		goto final;
	}
	if (g_app_ctx.monitor) {
		ret = get_device_event_fd(g_app_ctx.iio_device, &g_app_ctx.event_fd);
		if (ret)
			goto final;

		ret = configure_device_events();
		if (ret)
			goto final;
	} else {
		g_app_ctx.event_fd = event_create_timer(g_app_ctx.poll_period_ms,
							g_app_ctx.poll_period_ms);
		if (g_app_ctx.event_fd < 0) {
			fprintf(stderr, "event_fd create timer error(%d)\n", errno);
			ret = -errno;
			goto final;
		}
	}

	do {
		memset(&poll_fds, 0x00, sizeof(poll_fds));
		poll_fds[0].fd = g_app_ctx.stop_fd;
		poll_fds[0].events = POLLIN;
		poll_fds[1].fd = g_app_ctx.event_fd;
		poll_fds[1].events = POLLIN;

		p_ret = poll(poll_fds, 2, -1);
		if (p_ret <= 0) {
			fprintf(stderr, "poll error(%d)\n", errno);
			ret = -errno;
			break;
		}
		if (poll_fds[0].revents & POLLIN) {
			// Stop received
			read(g_app_ctx.stop_fd, &event_buff, sizeof(event_buff));
			break;
		}
		if (poll_fds[1].revents & POLLIN) {
			// Event or poll timeout received
			if (g_app_ctx.monitor) {
				ret = read(g_app_ctx.event_fd, &event_data,
					   sizeof(event_data));
				if (ret == -1) {
					fprintf(stderr,
						"read event data error(%d)",
						errno);
					break;
				}

				if (g_app_ctx.adc_channel != IIO_EVENT_CODE_EXTRACT_CHAN(event_data.id))
					continue;
				if (IIO_EVENT_CODE_EXTRACT_TYPE(event_data.id) != IIO_EV_TYPE_THRESH)
					continue;
				if (IIO_EVENT_CODE_EXTRACT_DIR(event_data.id) == IIO_EV_DIR_EITHER)
					continue;
				if ((IIO_EVENT_CODE_EXTRACT_DIR(event_data.id) == IIO_EV_DIR_RISING) &&
				    !g_app_ctx.threshold_high_en)
					continue;
				if ((IIO_EVENT_CODE_EXTRACT_DIR(event_data.id) == IIO_EV_DIR_FALLING) &&
				    !g_app_ctx.threshold_low_en)
					continue;
			} else {
				read(g_app_ctx.event_fd, &event_buff, sizeof(event_buff));
			}

			ret = get_device_channel_attr_float(g_app_ctx.iio_device,
				      			    g_app_ctx.adc_channel,
				      			    "raw",
							    &conversion_result);
			if (ret <= 0) {
				if (ret == -EAGAIN)
					continue;
				fprintf(stderr, "get conversion result error(%d)\n", ret);
				break;
			}
			print_conversion_result(g_app_ctx.monitor ? &event_data : NULL,
						&conversion_result);
		}
	} while (p_ret > 0);
final:
	if (g_app_ctx.stop_fd > 0)
		close(g_app_ctx.stop_fd);
	if (g_app_ctx.event_fd > 0)
		close(g_app_ctx.event_fd);
	return ret;
}
