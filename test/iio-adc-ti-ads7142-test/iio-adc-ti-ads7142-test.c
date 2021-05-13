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
#include <linux/kernel.h>
#include <linux/iio/events.h>
#include <linux/iio/types.h>
#include <linux/types.h>
#include <sys/eventfd.h>
#include <time.h>
#include <dirent.h>
#include <endian.h>

#include "utils.h"

enum iio_endian {
	IIO_BE,
	IIO_LE,
};

struct scan_info {
	int index;
	char sign;
	int realbits;
	int storagebits;
	int shift;
	enum iio_endian endiannes;
	int enabled;
	uint64_t mask;
};

struct channel_info {
	int channel;
	int channel1;
	int output;
	int diferential;
	enum iio_chan_type type;
	struct scan_info scan;
	bool have_scan;
	float scale;
	bool have_scale;
	float offset;
	bool have_offset;
};

struct device_info {
	char* name;
	int num_channels;
	struct channel_info *channels;
};

union scan_data {
	uint8_t u8;
	int8_t s8;
	uint16_t u16;
	int16_t s16;
	uint32_t u32;
	int32_t s32;
	uint64_t u64;
	int64_t s64;
};

#define TI_ADS7142_BUFFM_NONE				0
#define TI_ADS7142_BUFFM_STOP_BURST			1
#define TI_ADS7142_BUFFM_START_BURST			2
#define TI_ADS7142_BUFFM_PRE_ALERT			3
#define TI_ADS7142_BUFFM_POST_ALERT			4
static const char * const ti_ads7142_buffer_modes[] = {
	[TI_ADS7142_BUFFM_NONE]		= "none",
	[TI_ADS7142_BUFFM_STOP_BURST]	= "stop_burst",
	[TI_ADS7142_BUFFM_START_BURST]	= "start_burst",
	[TI_ADS7142_BUFFM_PRE_ALERT]	= "pre_alert",
	[TI_ADS7142_BUFFM_POST_ALERT]	= "post_alert"
};

static const char * const iio_chan_type_name_spec[] = {
	[IIO_VOLTAGE] = "voltage",
	[IIO_CURRENT] = "current",
	[IIO_POWER] = "power",
	[IIO_ACCEL] = "accel",
	[IIO_ANGL_VEL] = "anglvel",
	[IIO_MAGN] = "magn",
	[IIO_LIGHT] = "illuminance",
	[IIO_INTENSITY] = "intensity",
	[IIO_PROXIMITY] = "proximity",
	[IIO_TEMP] = "temp",
	[IIO_INCLI] = "incli",
	[IIO_ROT] = "rot",
	[IIO_ANGL] = "angl",
	[IIO_TIMESTAMP] = "timestamp",
	[IIO_CAPACITANCE] = "capacitance",
	[IIO_ALTVOLTAGE] = "altvoltage",
	[IIO_CCT] = "cct",
	[IIO_PRESSURE] = "pressure",
	[IIO_HUMIDITYRELATIVE] = "humidityrelative",
	[IIO_ACTIVITY] = "activity",
	[IIO_STEPS] = "steps",
	[IIO_ENERGY] = "energy",
	[IIO_DISTANCE] = "distance",
	[IIO_VELOCITY] = "velocity",
	[IIO_CONCENTRATION] = "concentration",
	[IIO_RESISTANCE] = "resistance",
	[IIO_PH] = "ph",
	[IIO_UVINDEX] = "uvindex",
	[IIO_ELECTRICALCONDUCTIVITY] = "electricalconductivity",
	[IIO_COUNT] = "count",
	[IIO_INDEX] = "index",
	[IIO_GRAVITY]  = "gravity",
	//[IIO_POSITIONRELATIVE]  = "positionrelative",
	//[IIO_PHASE] = "phase",
	//[IIO_MASSCONCENTRATION] = "massconcentration",
};

static const char * const iio_chan_direction_spec[] = {
	"in",
	"out"
};

static const char * const iio_endian_prefix[] = {
	[IIO_BE] = "be",
	[IIO_LE] = "le",
};

const char* g_short_options = "d:c:m:t:p:r:f:b:ih";
const struct option g_long_options[] = {
	{ "iio-device", required_argument, NULL, 'd' },
	{ "adc-channel", required_argument, NULL, 'c' },
	{ "monitor", required_argument, NULL, 'm' },
	{ "trigger", required_argument, NULL, 't' },
	{ "buffer-length", required_argument, NULL, 'b' },
	{ "threshold-low", required_argument, NULL, 'f' },
	{ "threshold-high", required_argument, NULL, 'r' },
	{ "hysteresis", required_argument, NULL, 257 },
	{ "poll-period", required_argument, NULL, 'p'},
	{ "info", no_argument, NULL, 'i' },
	{ "help", no_argument, NULL, 'h' },
	{ NULL, 0, NULL, 0 },
};

static struct {
	bool info;
	int iio_device;
	int adc_channel;
	bool have_adc_channel;
	int monitor;
	char *trigger;
	int buffer_length;
	bool have_buffer_length;
	int threshold_low;
	bool threshold_low_en;
	int threshold_high;
	bool threshold_high_en;
	int hysteresis;
	bool hysteresis_set;

	int poll_period_ms;

	int stop_fd;
	int chrdev_fd;
	int event_fd;

	struct device_info device_info;
	struct channel_info *channel_selected;
} g_app_ctx = {
	.monitor = TI_ADS7142_BUFFM_NONE,
	.poll_period_ms = 500,
};

static void print_help(FILE *output)
{
	fprintf(output,
		"\niio-adc-ti-ads7142-test - ads7142 command line test utility\n"
		"usage: iio-adc-ti-ads7142-test [options]\n"
		"\n"
		"  -d, --iio-device=DEVICE_INDEX\n"
		"  -c, --adc-channel=CHANNEL_INDEX\n"
		"  -m, --monitor=BUFFER_MODE\n"
		"  -b, --buffer-length=BUFFER_LENGTH\n"
		"  -t, --trigger=TRIGGER_NAME\n"
		"  -f, --threshold-low=LOWER_THRESHOLD\n"
		"  -r, --threshold-high=HIGHER_THRESHOLD\n"
		"      --hysteresis=HYSTERESIS\n"
		"  -p, --poll-period=POLL_PERIOD\n"
		"  -i, --info\n"
		"  -h, --help\n"
	);
}

static void print_help_exit(FILE *output, int exit_code)
{
	print_help(output);
	exit(exit_code);
}

static void print_info(FILE *output)
{
	struct channel_info *channel_info;
	int i;

	fprintf(output,
		"Device\tindex:	%d\n"
		"	name:	%s\n",
		g_app_ctx.iio_device,
		g_app_ctx.device_info.name);
	
	for (i = 0; i < g_app_ctx.device_info.num_channels; i++) {
		channel_info = &g_app_ctx.device_info.channels[i];
		fprintf(output,
			"\tChannel\tindex:\t		%d\n"
			"\t	direction:\t	%s\n"
			"\t	type:\t		%s\n"
			"\t	offset:\t		%f\n"
			"\t	scale:\t		%f\n"
			,
			channel_info->channel,
			iio_chan_direction_spec[channel_info->output],
			iio_chan_type_name_spec[channel_info->type],
			channel_info->offset,
			channel_info->scale);
		if (channel_info->have_scan) {
			fprintf(output,
				"\t\tscan\tindex:		%d\n"
				"\t\t	configuration:	%s:%c%d/%d>>%u\n"
				"\t\t	enabled:	%s\n",
				channel_info->scan.index,
				iio_endian_prefix[channel_info->scan.endiannes],
				channel_info->scan.sign,
				channel_info->scan.realbits,
				channel_info->scan.storagebits,
				channel_info->scan.shift,
				channel_info->scan.enabled ? "yes" : "no");
		}
	}
}

static void print_info_exit(FILE *output, int exit_code)
{
	print_info(output);
	exit(exit_code);
}

static int parse_monitor_str(const char* value)
{
	int i;

	if (value == NULL)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ti_ads7142_buffer_modes); i++) {
		if (strcmp(ti_ads7142_buffer_modes[i], value) == 0)
			return i;
	}

	return -ENOENT;
}

static int parse_type_str(const char* value, enum iio_chan_type *result)
{
	int i;

	if (value == NULL)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(iio_chan_type_name_spec); i++) {
		if (strcmp(iio_chan_type_name_spec[i], value) == 0) {
			*result = i;
			return 0;
		}
	}

	return -ENOENT;
}

static int get_device_chrdev_fd(int device, int *chrdev_fd)
{
	int ret;
	char* chrdev_path;

	ret = asprintf(&chrdev_path, "/dev/iio:device%d", device);
	if (ret < 0)
		return -ENOMEM;

	ret = 0;
	*chrdev_fd = open(chrdev_path, O_RDONLY | O_NONBLOCK);
	if (*chrdev_fd == -1) {
		ret = -errno;
		fprintf(stderr, "device(%s) open error(%d)\n", chrdev_path, errno);
	}

	free(chrdev_path);
	return ret;
}

static int get_device_event_fd(int device, int chrdev_fd, int *event_fd)
{
	int ret;

	ret = ioctl(chrdev_fd, IIO_GET_EVENT_FD_IOCTL, event_fd);
	if (ret == -1 || *event_fd == -1) {
		ret = -errno;
		if (ret == -ENODEV)
			fprintf(stderr,
				"this device does not support events\n");
		else
			fprintf(stderr, "failed to retrive event fd\n");
	}

	return ret;
}

static int get_device_attr_string(int device, const char *attr_name,
				  char **value)
{
	int ret;
	char* attr_path;
	char* value_buffer = NULL;

	ret = asprintf(&attr_path, "/sys/bus/iio/devices/iio:device%d/%s",
		       device, attr_name);
	if (ret < 0)
		return -ENOMEM;
	
	ret = file_cat(attr_path, 0, 0, (void**)&value_buffer);
	if (ret >= 0) {
		*value = strndup(value_buffer, ret);
		free(value_buffer);
	}
	free(attr_path);
	return ret >= 0 ? 0 : ret;
}

static int set_device_attr_string(int device, const char *attr_name,
				  const char *value)
{
	int ret;
	char* attr_path;

	ret = asprintf(&attr_path, "/sys/bus/iio/devices/iio:device%d/%s",
		       device, attr_name);
	if (ret < 0)
		return -ENOMEM;
	
	ret = file_printf(attr_path, "%s\n", value);
	free(attr_path);
	return ret >= 0 ? 0 : ret;
}

static int set_device_trigger_attr_string(int device, const char *attr_name,
					  const char *value)
{
	int ret;
	char* attr_path;

	ret = asprintf(&attr_path, "/sys/bus/iio/devices/iio:device%d/trigger/%s",
		       device, attr_name);
	if (ret < 0)
		return -ENOMEM;
	
	ret = file_printf(attr_path, "%s\n", value);
	free(attr_path);
	return ret >= 0 ? 0 : ret;
}

static int get_device_buffer_attr_int(int device, const char *attr_name,
				      int *value)
{
	int ret;
	char* attr_path;

	ret = asprintf(&attr_path, "/sys/bus/iio/devices/iio:device%d/buffer/%s",
		       device, attr_name);
	if (ret < 0)
		return -ENOMEM;
	
	ret = file_scanf(attr_path, "%d", value);
	free(attr_path);
	return ret >= 0 ? 0 : ret;
}

static int set_device_buffer_attr_int(int device, const char *attr_name,
				      int value)
{
	int ret;
	char* attr_path;

	ret = asprintf(&attr_path, "/sys/bus/iio/devices/iio:device%d/buffer/%s",
		       device, attr_name);
	if (ret < 0)
		return -ENOMEM;
	
	ret = file_printf(attr_path, "%d\n", value);
	free(attr_path);
	return ret >= 0 ? 0 : ret;
}

static int get_device_channel_scan_elements(int device,
					    struct channel_info *channel_info)
{
	int ret;
	char* attr_path;
	char* endiannes = NULL;

	ret = asprintf(&attr_path, "/sys/bus/iio/devices/iio:device%d/scan_elements/%s_%s%d_en",
		       device,
		       iio_chan_direction_spec[channel_info->output],
		       iio_chan_type_name_spec[channel_info->type],
		       channel_info->channel);
	if (ret < 0)
		return -ENOMEM;
	
	ret = file_scanf(attr_path, "%d", &channel_info->scan.enabled);
	free(attr_path);

	if (ret < 1) {
		ret = ret < 0 ? ret : -EINVAL;
		goto out;
	}

	ret = asprintf(&attr_path, "/sys/bus/iio/devices/iio:device%d/scan_elements/%s_%s%d_index",
		       device,
		       iio_chan_direction_spec[channel_info->output],
		       iio_chan_type_name_spec[channel_info->type],
		       channel_info->channel);
	if (ret < 0)
		return -ENOMEM;
	
	ret = file_scanf(attr_path, "%d", &channel_info->scan.index);
	free(attr_path);

	if (ret < 1) {
		ret = ret < 0 ? ret : -EINVAL;
		goto out;
	}

	ret = asprintf(&attr_path, "/sys/bus/iio/devices/iio:device%d/scan_elements/%s_%s%d_type",
		       device,
		       iio_chan_direction_spec[channel_info->output],
		       iio_chan_type_name_spec[channel_info->type],
		       channel_info->channel);
	if (ret < 0)
		return -ENOMEM;
	
	ret = file_scanf(attr_path, "%m[a-z]:%c%u/%u>>%u",
			 &endiannes, &channel_info->scan.sign,
			 &channel_info->scan.realbits,
			 &channel_info->scan.storagebits,
			 &channel_info->scan.shift);
	free(attr_path);
	if (ret != 5) {
		ret = ret < 0 ? ret : -EINVAL;
	} else {
		if (strcmp(endiannes, iio_endian_prefix[IIO_BE]) == 0)
			channel_info->scan.endiannes = IIO_BE;
		else
			channel_info->scan.endiannes = IIO_LE;
		
		if (channel_info->scan.realbits == 64)
			channel_info->scan.mask = ~(0ULL);
		else
			channel_info->scan.mask = (1ULL << channel_info->scan.realbits) - 1ULL;

		ret = 0;
	}
	if (endiannes != NULL)
		free(endiannes);
out:
	return ret;
}

static int get_device_scan_size(struct device_info *device_info)
{
	int i;
	struct channel_info *channel_info;
	int ret = 0;

	for (i = 0; i < device_info->num_channels; i++) {
		channel_info = &device_info->channels[i];
		if (!channel_info->have_scan)
			continue;
		if (!channel_info->scan.enabled)
			continue;
		ret += channel_info->scan.storagebits / 8;
	}
	return ret;
}

static int get_channel_by_index(struct device_info *device_info,
				int channel_index,
				struct channel_info **channel_info)
{
	int i;

	for (i = 0; i < device_info->num_channels; i++) {
		if (device_info->channels[i].channel == channel_index) {
			*channel_info = &device_info->channels[i];
			return 0;
		}
	}
	return -ENOENT;
}

static int get_device_channel_scan_elements_attr_int(int device,
						     struct channel_info *channel_info,
						     const char *attr_name,
						     int *value)
{
	int ret;
	char* attr_path;

	ret = asprintf(&attr_path, "/sys/bus/iio/devices/iio:device%d/scan_elements/%s_%s%d_%s",
		       device, iio_chan_direction_spec[channel_info->output],
		       iio_chan_type_name_spec[channel_info->type],
		       channel_info->channel,
		       attr_name);
	if (ret < 0)
		return -ENOMEM;
	
	ret = file_scanf(attr_path, "%d", value);
	free(attr_path);
	return ret >= 0 ? 0 : ret;
}

static int set_device_channel_scan_elements_attr_int(int device,
						     struct channel_info *channel_info,
						     const char *attr_name,
						     int value)
{
	int ret;
	char* attr_path;

	ret = asprintf(&attr_path, "/sys/bus/iio/devices/iio:device%d/scan_elements/%s_%s%d_%s",
		       device, iio_chan_direction_spec[channel_info->output],
		       iio_chan_type_name_spec[channel_info->type],
		       channel_info->channel,
		       attr_name);
	if (ret < 0)
		return -ENOMEM;
	
	ret = file_printf(attr_path, "%d\n", value);
	free(attr_path);
	return ret >= 0 ? 0 : ret;
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
	return ret > 0 ? 0 : ret;
}

static int set_device_buffer_mode(int device, int value)
{
	int ret;
	char* attr_path;

	ret = asprintf(&attr_path, "/sys/bus/iio/devices/iio:device%d/buffer_mode",
		       device);
	if (ret < 0)
		return -ENOMEM;
	
	ret = file_printf(attr_path, "%s\n", ti_ads7142_buffer_modes[value]);
	free(attr_path);
	return ret > 0 ? 0 : -1;
}

static int get_device_channel_info(int device,
				   struct channel_info *channel_info)
{
	channel_info->have_scan = get_device_channel_scan_elements(device,
								   channel_info) == 0;
	channel_info->scale = 1;
	channel_info->offset = 0;
	get_device_channel_attr_float(device,
				      channel_info->channel,
				      "scale", &channel_info->scale);
	get_device_channel_attr_float(device,
				      channel_info->channel,
				      "offset", &channel_info->offset);
	return 0;
}

static int get_device_info(int device, struct device_info *device_info)
{
	DIR *dir;
	const struct dirent *entry;
	char* device_path;
	char* channel_direction;
	int channel1_index;
	char* channel1_type;
	char* channel1_postfix;
	int scan_ret;
	int i;
	bool add_channel;
	struct channel_info channel_info;
	int ret;

	ret = get_device_attr_string(device, "name", &device_info->name);
	if (ret)
		return ret;

	ret = asprintf(&device_path, "/sys/bus/iio/devices/iio:device%d",
		       device);
	if (ret < 0)
		return -ENOMEM;
	
	ret = 0;

	dir = opendir(device_path);
	if (!dir) {
		ret = -errno;
		goto out;
	}

	while ((entry = readdir(dir)) && !ret) {
		channel_direction = NULL;
		channel1_type = NULL;
		channel1_postfix = NULL;
		scan_ret = sscanf(entry->d_name, "%m[a-z]_%m[a-z]%d_%m[a-z]",
				  &channel_direction,
				  &channel1_type, &channel1_index, &channel1_postfix);
		if (scan_ret == 4) {
			add_channel = true;
			for (i = 0; i < device_info->num_channels; i++) {
				if (device_info->channels[i].channel == channel1_index) {
					add_channel = false;
					break;
				}
			}
			if (add_channel) {
				memset(&channel_info, 0x00, sizeof(channel_info));
				channel_info.channel = channel1_index;
				channel_info.channel1 = 0;
				channel_info.output = strcmp(channel_direction, "out") == 0;
				ret = parse_type_str(channel1_type, &channel_info.type);
				if (ret)
					break;

				ret = get_device_channel_info(device, &channel_info);
				if (ret)
					break;
				
				device_info->channels = realloc(device_info->channels,
								sizeof(channel_info) * (device_info->num_channels + 1));
				if (device_info->channels == NULL) {
					ret = -ENOMEM;
					break;
				}
				memcpy(&device_info->channels[device_info->num_channels],
				       &channel_info, sizeof(channel_info));
				device_info->num_channels++;
			}
		}
		if (channel_direction != NULL) {
			free(channel_direction);
			channel_direction = NULL;
		}
		if (channel1_type != NULL) {
			free(channel1_type);
			channel1_type = NULL;
		}
		if (channel1_postfix != NULL) {
			free(channel1_postfix);
			channel1_postfix = NULL;
		}
	}

	closedir(dir);
	if (channel_direction != NULL) {
		free(channel_direction);
		channel_direction = NULL;
	}
	if (channel1_type != NULL) {
		free(channel1_type);
		channel1_type = NULL;
	}
	if (channel1_postfix != NULL) {
		free(channel1_postfix);
		channel1_postfix = NULL;
	}

out:
	free(device_path);
	return ret;
}

static int print_conversion_result(struct channel_info *channel_info,
				   const struct iio_event_data *event_data,
				   float* conversion_result)
{
	char time_string[100];
	time_t now = time(NULL);
	struct tm tm_now;
	float scaled_result;
	enum iio_event_direction dir;
	int channel;

	now = time(NULL);
	localtime_r(&now, &tm_now);
	strftime(time_string, sizeof(time_string), "%F %T", &tm_now);

	printf("%s\t", time_string);

	if (g_app_ctx.monitor && event_data) {
		channel = IIO_EVENT_CODE_EXTRACT_CHAN(event_data->id);
		dir = IIO_EVENT_CODE_EXTRACT_DIR(event_data->id);
		printf("ch %2d. threshold %s\t",
		       channel,
		       dir == IIO_EV_DIR_RISING ? "rising" : "falling");
	}

	if (conversion_result) {
		if (channel_info)
			printf("ch %3d. ", channel_info->channel);
		printf("raw: %5.0f", *conversion_result);
		if (channel_info) {
			scaled_result = *conversion_result * channel_info->scale +
					channel_info->offset;
			printf("\tscaled: %8.3f", scaled_result);
		}
	}

	printf("\n");
	return 0;
}

static int print_channel_scan_result(struct channel_info *channel_info, void *data)
{
	int scan_size;
	union scan_data val;

	scan_size = channel_info->scan.storagebits / 8;
	switch (scan_size) {
	case 1:
		val.u8 = *(uint8_t*)data;
		val.u8 >>= channel_info->scan.shift;
		val.u8 &= channel_info->scan.mask;
		if (channel_info->scan.sign == 's') {
			val.s8 = (int8_t)(val.u8 << (8 - channel_info->scan.realbits)) >>
				 (8 - channel_info->scan.realbits);
			printf("%5.0f ", (float)val.s8);
		} else {
			printf("%5.0f ", (float)val.u8);
		}
	break;
	case 2:
		val.u16 = *(uint16_t*)data;
		if (channel_info->scan.endiannes == IIO_BE)
			val.u16 = be16toh(val.u16);
		else
			val.u16 = le16toh(val.u16);
		val.u16 >>= channel_info->scan.shift;
		val.u16 &= channel_info->scan.mask;
		if (channel_info->scan.sign == 's') {
			val.s16 = (int8_t)(val.u16 << (16 - channel_info->scan.realbits)) >>
				 (16 - channel_info->scan.realbits);
			printf("%5.0f ", (float)val.s16);
		} else {
			printf("%5.0f ", (float)val.u16);
		}
	break;
	case 4:
		val.u32 = *(uint32_t*)data;
		if (channel_info->scan.endiannes == IIO_BE)
			val.u32 = be32toh(val.u32);
		else
			val.u32 = le32toh(val.u32);
		val.u32 >>= channel_info->scan.shift;
		val.u32 &= channel_info->scan.mask;
		if (channel_info->scan.sign == 's') {
			val.s32 = (int8_t)(val.u32 << (32 - channel_info->scan.realbits)) >>
				 (32 - channel_info->scan.realbits);
			printf("%5.0f ", (float)val.s32);
		} else {
			printf("%5.0f ", (float)val.u32);
		}
	break;
	case 8:
		val.u64 = *(uint64_t*)data;
		if (channel_info->scan.endiannes == IIO_BE)
			val.u64 = be64toh(val.u64);
		else
			val.u64 = le64toh(val.u64);
		val.u64 >>= channel_info->scan.shift;
		val.u64 &= channel_info->scan.mask;
		if (channel_info->scan.sign == 's') {
			val.s64 = (int8_t)(val.u64 << (64 - channel_info->scan.realbits)) >>
				 (64 - channel_info->scan.realbits);
			printf("%5.0f ", (float)val.s64);
		} else {
			printf("%5.0f ", (float)val.u64);
		}
	break;
	default:
		return -EINVAL;
	}
	return scan_size;
}

static int print_scan_result(struct device_info *device_info, void *record_data)
{
	int record_index = 0;
	int ret;
	struct channel_info *channel_info;
	char time_string[100];
	time_t now = time(NULL);
	struct tm tm_now;
	int i;

	now = time(NULL);
	localtime_r(&now, &tm_now);
	strftime(time_string, sizeof(time_string), "%F %T", &tm_now);

	printf("%s\t", time_string);

	for (i = 0; i < device_info->num_channels; i++) {
		channel_info = &device_info->channels[i];
		if (!channel_info->have_scan)
			continue;
		if (!channel_info->scan.enabled)
			continue;
		ret = print_channel_scan_result(channel_info,
						&((unsigned char*)record_data)[record_index]);
		if (ret < 0)
			return ret;
		record_index += ret;
	}
	fprintf(stdout, "\n");
	return record_index;
}

static int configure_device_buffers(bool enable)
{
	struct channel_info *channel_info;
	int i;
	int scan_en;
	bool have_to_scan = false;;
	int ret;

	if (g_app_ctx.monitor && enable) {
		ret = set_device_attr_string(g_app_ctx.iio_device, "buffer_mode",
					     ti_ads7142_buffer_modes[g_app_ctx.monitor]);
		if (ret && enable) {
			fprintf(stderr, "Set buffer mode error(%d)", ret);
			return ret;
		}
	} else {
		set_device_attr_string(g_app_ctx.iio_device, "buffer_mode",
				       ti_ads7142_buffer_modes[g_app_ctx.monitor]);
	}

	if (g_app_ctx.trigger && enable) {
		ret = set_device_trigger_attr_string(g_app_ctx.iio_device, "current_trigger",
						     g_app_ctx.trigger);
		if (ret && enable)
			return ret;
	}

	for (i = 0; i < g_app_ctx.device_info.num_channels; i++) {
		channel_info = &g_app_ctx.device_info.channels[i];
		if (!channel_info->have_scan)
			continue;
		
		scan_en = !g_app_ctx.have_adc_channel ? 1
				: g_app_ctx.adc_channel == channel_info->channel
					? 1 : 0;
		scan_en = enable ? scan_en : 0;
		ret = set_device_channel_scan_elements_attr_int(g_app_ctx.iio_device,
								channel_info,
								"en",
								scan_en);
		if (ret && enable)
			return ret;

		if (scan_en)
			have_to_scan = true;
		channel_info->scan.enabled = scan_en;
	}

	if (enable && have_to_scan && g_app_ctx.have_buffer_length) {
		ret = set_device_buffer_attr_int(g_app_ctx.iio_device,
						 "length", g_app_ctx.buffer_length);
		if (ret)
			return ret;
	}

	if (enable && have_to_scan) {
		ret = get_device_buffer_attr_int(g_app_ctx.iio_device,
						 "length", &g_app_ctx.buffer_length);
		if (ret)
			return ret;
	}

	if (enable && have_to_scan) {
		ret = set_device_buffer_attr_int(g_app_ctx.iio_device,
						 "enable", 1);
		if (ret)
			return ret;
	} else {
		set_device_buffer_attr_int(g_app_ctx.iio_device,
					   "enable", 0);
	}
	return 0;
}

static int configure_device_events(void)
{
	int i;
	bool chen;
	struct channel_info *channel_info;
	int channel;
	int ret;

	for (i = 0; i < g_app_ctx.device_info.num_channels; i++) {
		channel_info = &g_app_ctx.device_info.channels[i];
		channel = channel_info->channel;
		chen = (!g_app_ctx.have_adc_channel) ? true
			: g_app_ctx.adc_channel == channel;
		if (g_app_ctx.threshold_low_en && chen) {
			ret = set_device_channel_event_attr_int(g_app_ctx.iio_device,
								channel,
								"thresh_falling_value",
								g_app_ctx.threshold_low);
			if (ret) {
				printf("set thresh_falling_value(%d) error(%d)",
				g_app_ctx.threshold_low, ret);
				return ret;
			}
		}
		ret = set_device_channel_event_attr_int(g_app_ctx.iio_device,
							channel,
							"thresh_falling_en",
							(g_app_ctx.threshold_low_en && chen) ? 1 : 0);
		if (ret) {
			printf("set thresh_falling_en(%d) error(%d)",
			(g_app_ctx.threshold_low_en && chen) ? 1 : 0, ret);
			return ret;
		}

		if (g_app_ctx.threshold_high_en && chen) {
			ret = set_device_channel_event_attr_int(g_app_ctx.iio_device,
								channel,
								"thresh_rising_value",
								g_app_ctx.threshold_high);
			if (ret) {
				printf("set thresh_rising_value(%d) error(%d)",
				g_app_ctx.threshold_high, ret);
				return ret;
			}
		}
		ret = set_device_channel_event_attr_int(g_app_ctx.iio_device,
							channel,
							"thresh_rising_en",
							(g_app_ctx.threshold_high_en && chen) ? 1 : 0);
		if (ret) {
			printf("set thresh_rising_en(%d) error(%d)",
			(g_app_ctx.threshold_high_en && chen) ? 1 : 0, ret);
			return ret;
		}

		if (g_app_ctx.hysteresis_set && chen) {
			ret = set_device_channel_event_attr_int(g_app_ctx.iio_device,
								channel,
								"thresh_either_hysteresis",
								g_app_ctx.hysteresis);
			if (ret) {
				printf("set thresh_either_hysteresis(%d) error(%d)",
				g_app_ctx.hysteresis, ret);
				return ret;
			}
		}
	}
	return 0;
}

static void sig_handler(int signum)
{
	uint64_t event_buff;
	if (g_app_ctx.stop_fd > 0) {
		write(g_app_ctx.stop_fd, &event_buff, sizeof(event_buff));
	}
}

static void register_sig_handlers(void)
{
	struct sigaction sa = { .sa_handler = sig_handler };
	const int signums[] = { SIGINT, SIGTERM, SIGABRT };
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(signums); ++i) {
		ret = sigaction(signums[i], &sa, NULL);
		if (ret) {
			fprintf(stderr, "Failed to register signal handler");
			exit(-1);
		}
	}
}

int main(int argc, char ** argv)
{
	int next_option;
	int ret = 0;
	struct pollfd poll_fds[3];
	uint64_t event_buff;
	int p_ret;
	float conversion_result;
	struct iio_event_data event_data;
	int scan_size;
	void *buffer_data = NULL;
	int i;

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
			g_app_ctx.have_adc_channel = true;
		break;
		case 'm':
			g_app_ctx.monitor = parse_monitor_str(optarg);
			if (g_app_ctx.monitor < 0)
				print_help_exit(stderr, EXIT_FAILURE);
		break;
		case 'b':
			if (optarg == NULL)
				print_help_exit(stderr, EXIT_FAILURE);
			
			g_app_ctx.buffer_length = atoi(optarg);
			g_app_ctx.have_buffer_length = true;
		break;
		case 't':
			if (optarg == NULL)
				print_help_exit(stderr, EXIT_FAILURE);

			g_app_ctx.trigger = strdup(optarg);
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
		case 'i':
			g_app_ctx.info = true;
		break;
		default:
			print_help_exit(stderr, EXIT_FAILURE);
		break;
		}
	}

	if (g_app_ctx.monitor &&
	    (g_app_ctx.monitor <= TI_ADS7142_BUFFM_START_BURST) &&
	    !g_app_ctx.trigger)
		print_help_exit(stderr, EXIT_FAILURE);
	
	ret = get_device_info(g_app_ctx.iio_device, &g_app_ctx.device_info);
	if (ret) {
		fprintf(stderr, "Get device info error(%d)\n", ret);
		print_help_exit(stderr, EXIT_FAILURE);
	}

	if (!g_app_ctx.monitor || g_app_ctx.have_adc_channel) {
		ret = get_channel_by_index(&g_app_ctx.device_info,
					   g_app_ctx.adc_channel,
					   &g_app_ctx.channel_selected);
		if (ret) {
			fprintf(stderr, "Unknown channel(%d)\n",
				g_app_ctx.adc_channel);
			goto final;
		}
	}
	register_sig_handlers();

	g_app_ctx.stop_fd = eventfd(0, EFD_NONBLOCK);
	if (g_app_ctx.stop_fd < 0) {
		fprintf(stderr, "stop_fd create error(%d)\n", errno);
		ret = -errno;
		goto final;
	}

	ret = set_device_buffer_mode(g_app_ctx.iio_device, g_app_ctx.monitor);
	if (ret) {
		fprintf(stderr, "Set buffer mode error(%d)\n", ret);
		goto final;
	}

	if (g_app_ctx.monitor) {
		ret = get_device_chrdev_fd(g_app_ctx.iio_device,
					   &g_app_ctx.chrdev_fd);
		if (ret) {
			goto final;
		}

		if (g_app_ctx.monitor >= TI_ADS7142_BUFFM_PRE_ALERT) {
			ret = get_device_event_fd(g_app_ctx.iio_device,
						  g_app_ctx.chrdev_fd,
						  &g_app_ctx.event_fd);
			if (ret)
				goto final;

			ret = configure_device_events();
			if (ret) {
				fprintf(stderr, "Events configuration error(%d)\n", ret);
				goto final;
			}
		}

		ret = configure_device_buffers(true);
		if (ret) {
			fprintf(stderr, "Buffer configuration error(%d)\n", ret);
			goto final;
		}
	} else {
		configure_device_buffers(false);
		g_app_ctx.event_fd = event_create_timer(g_app_ctx.poll_period_ms,
							g_app_ctx.poll_period_ms);
		if (g_app_ctx.event_fd < 0) {
			fprintf(stderr, "event_fd create timer error(%d)\n", errno);
			ret = -errno;
			goto final;
		}
	}

	if (g_app_ctx.info)
		print_info(stdout);

	if (g_app_ctx.monitor) {
		scan_size = get_device_scan_size(&g_app_ctx.device_info);
		if (!scan_size)
			goto final;
		buffer_data = malloc(scan_size * g_app_ctx.buffer_length);
	}

	do {
		memset(&poll_fds, 0x00, sizeof(poll_fds));
		poll_fds[0].fd = g_app_ctx.stop_fd;
		poll_fds[0].events = POLLIN;
		poll_fds[1].fd = g_app_ctx.event_fd;
		poll_fds[1].events = POLLIN;
		poll_fds[2].fd = g_app_ctx.chrdev_fd;
		poll_fds[2].events = POLLIN;

		p_ret = poll(poll_fds, g_app_ctx.monitor ? 3 : 2, -1);
		if (p_ret <= 0) {
			if (errno != EINTR)
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

				if (g_app_ctx.have_adc_channel && g_app_ctx.adc_channel != IIO_EVENT_CODE_EXTRACT_CHAN(event_data.id))
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

			if (g_app_ctx.monitor) {
				print_conversion_result(NULL, &event_data, NULL);
			} else {
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
				print_conversion_result(g_app_ctx.channel_selected,
							NULL,
							&conversion_result);
			}
		}
		if (poll_fds[2].revents & POLLIN) {
			do {
				ret = read(g_app_ctx.chrdev_fd, buffer_data, scan_size * g_app_ctx.buffer_length);
				if (ret < 0) {
					break;
				}

				for (i = 0; i < ret / scan_size; i++) {
					print_scan_result(&g_app_ctx.device_info,
							  &((char*)buffer_data)[scan_size * i]);
				}
			} while(ret > 0);
		}
	} while (p_ret > 0);

	configure_device_buffers(false);
final:
	if (g_app_ctx.stop_fd > 0)
		close(g_app_ctx.stop_fd);
	if (g_app_ctx.event_fd > 0)
		close(g_app_ctx.event_fd);
	if (g_app_ctx.chrdev_fd > 0)
		close(g_app_ctx.chrdev_fd);
	if (g_app_ctx.trigger)
		free(g_app_ctx.trigger);
	if (g_app_ctx.device_info.name)
		free(g_app_ctx.device_info.name);
	if (g_app_ctx.device_info.channels)
		free(g_app_ctx.device_info.channels);
	if (buffer_data)
		free(buffer_data);
	return ret;
}
