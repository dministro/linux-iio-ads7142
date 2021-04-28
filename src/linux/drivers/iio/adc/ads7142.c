// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Jozsef Horvath <info@ministro.hu>
 *
 */
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/events.h>

#define ADS7142_NAME				"ads7142"

#define ADS7142_DATA_VALID_TIMEOUT		100

/* ADS7142 Opcodes for commands */
/* General */
#define ADS7142_OC_GENERAL			0x00
/* Single Register Read */
#define ADS7142_OC_SINGLE_REG_READ		0x10
/* Single Register Write */
#define ADS7142_OC_SINGLE_REG_WRITE		0x08
/* Single Bit Set */
#define ADS7142_OC_SET_BIT			0x18
/* Single Bit Clear */
#define ADS7142_OC_CLEAR_BIT			0x20
/* Block Register Read */
#define ADS7142_OC_BLOCK_READ			0x30
/* Block Register Write */
#define ADS7142_OC_BLOCK_WRITE			0x28

/* ADS7142 registers */
/* Reset registers */
#define ADS7142_WKEY				0x17
#define ADS7142_DEVICE_RESET			0x14
/* Functional mode select registers */
#define ADS7142_OFFSET_CAL			0x15
#define ADS7142_OPMODE_SEL			0x1C
#define ADS7142_OPMODE_SEL_MANUALCH0		(0)
#define ADS7142_OPMODE_SEL_MANUALSEQ		(4)
#define ADS7142_OPMODE_SEL_MONITORING		(6)
#define ADS7142_OPMODE_SEL_HIGHPREC		(7)
#define ADS7142_OPMODE_STATUS			0x00
#define ADS7142_OPMODE_STATUS_OPMODE_MSK	(3)
#define ADS7142_OPMODE_STATUS_OPMODE_MANUAL	(0)
#define ADS7142_OPMODE_STATUS_OPMODE_AUTO	(2)
#define ADS7142_OPMODE_STATUS_OPMODE_HIGHPREC	(3)
#define ADS7142_OPMODE_STATUS_HS_MODE		BIT(2)

/* Input config register */
#define ADS7142_CH_INPUT_CFG			0x24
#define ADS7142_CH_INPUT_CFG_TCSE		(0)
#define ADS7142_CH_INPUT_CFG_SCSE		(1)
#define ADS7142_CH_INPUT_CFG_SCPD		(2)
/* Analog mux and sequencer registers */
#define ADS7142_AUTO_SEQ_CHEN			0x20
#define ADS7142_AUTO_SEQ_CHEN_CH0		BIT(0)
#define ADS7142_AUTO_SEQ_CHEN_CH1		BIT(1)
#define ADS7142_START_SEQUENCE			0x1E
#define ADS7142_START_SEQUENCE_SEQ_START	BIT(0)
#define ADS7142_ABORT_SEQUENCE			0x1F
#define ADS7142_ABORT_SEQUENCE_SEQ_ABORT	BIT(0)
#define ADS7142_SEQUENCE_STATUS			0x04
#define ADS7142_SEQUENCE_STATUS_SEQ_ERR_ST_MSK	(0x06)
#define ADS7142_SEQUENCE_STATUS_SEQ_DISABLED	(0x00)
#define ADS7142_SEQUENCE_STATUS_SEQ_ENABLED	(0x02)
#define ADS7142_SEQUENCE_STATUS_SEQ_ERROR	(0x06)
/* Oscillator and timing control registers */
#define ADS7142_OSC_SEL				0x18
#define ADS7142_OSC_SEL_HSZ_LP			BIT(0)
#define ADS7142_NCLK_SEL			0x19
/* Data buffer control register */
#define ADS7142_DATA_BUFFER_OPMODE		0x2C
#define ADS7142_DATA_BUFFER_OPMODE_STOP_BURST	(0)
#define ADS7142_DATA_BUFFER_OPMODE_START_BURST	(1)
#define ADS7142_DATA_BUFFER_OPMODE_PRE_ALERT	(4)
#define ADS7142_DATA_BUFFER_OPMODE_POST_ALERT	(6)
#define ADS7142_DOUT_FORMAT_CFG			0x28
#define ADS7142_DOUT_FORMAT_CFG_12B		(0)
#define ADS7142_DOUT_FORMAT_CFG_12BCH		(1)
#define ADS7142_DOUT_FORMAT_CFG_12BCHDV		(2)
#define ADS7142_DATA_BUFFER_STATUS		0x01
/* Accumulator control register */
#define ADS7142_ACC_EN				0x30
#define ADS7142_ACC_CH0_LSB			0x08
#define ADS7142_ACC_CH0_MSB			0x09
#define ADS7142_ACC_CH1_LSB			0x0A
#define ADS7142_ACC_CH1_MSB			0x0B
#define ADS7142_ACC_STATUS			0x02
/* Digital window comparator registers */
#define ADS7142_ALERT_DWC_EN			0x37
#define ADS7142_ALERT_DWC_EN_BLOCK_EN		BIT(0)
#define ADS7142_ALERT_CHEN			0x34
#define ADS7142_DWC_HTH_CH0_LSB			0x38
#define ADS7142_DWC_HTH_CH0_MSB			0x39
#define ADS7142_DWC_LTH_CH0_LSB			0x3A
#define ADS7142_DWC_LTH_CH0_MSB			0x3B
#define ADS7142_DWC_HYS_CH0			0x40
#define ADS7142_DWC_HTH_CH1_LSB			0x3C
#define ADS7142_DWC_HTH_CH1_MSB			0x3D
#define ADS7142_DWC_LTH_CH1_LSB			0x3E
#define ADS7142_DWC_LTH_CH1_MSB			0x3F
#define ADS7142_DWC_HYS_CH1			0x41
#define ADS7142_PRE_ALT_EVT_CNT			0x36
#define ADS7142_ALT_TRIG_CHID			0x03
#define ADS7142_ALT_LOW_FLAGS			0x0C
#define ADS7142_ALT_LOW_FLAGS_CH0		BIT(0)
#define ADS7142_ALT_LOW_FLAGS_CH1		BIT(1)
#define ADS7142_ALT_HIGH_FLAGS			0x0E
#define ADS7142_ALT_HIGH_FLAGS_CH0		BIT(0)
#define ADS7142_ALT_HIGH_FLAGS_CH1		BIT(1)

#define ADS7142_THRESHOLD_MSK			0xFFF
#define ADS7142_HYSTERESIS_MSK			0x3F

struct ads7142_channel_data {
	int status;
	int value;
};

struct ads7142_channel_config {
	bool alert_low;
	bool alert_high;
	int high_threshold;
	int low_threshold;
	int hysteresis;
};

struct ads7142_channel {
	struct ads7142_channel_config config;
	struct ads7142_channel_data data;
	u32 channel;
};

struct ads7142_config {
	bool osc_sel;
	u32 n_clk;
	bool monitoring_mode;
};

struct ads7142_priv {
	struct mutex lock; /* For syncing access to device */
	struct regulator *vref;
	struct regulator *power;
	struct ads7142_config config;
	int channel_count;
	struct ads7142_channel *channels;
	bool suspended;
	bool monitor_pending;
};

static const struct iio_event_spec ads7142_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE)
				| BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE)
				| BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_HYSTERESIS),
	},
};

static int ads7142_reg_write(const struct i2c_client *client, u8 reg, u8 data)
{
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	buf[0] = ADS7142_OC_SINGLE_REG_WRITE;
	buf[1] = reg;
	buf[2] = data;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret >= 0 ? 0 : ret;
}

static int ads7142_reg_read(const struct i2c_client *client, u8 reg, u8 *data)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = ADS7142_OC_SINGLE_REG_READ;
	buf[1] = reg;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data;

	ret = i2c_transfer(client->adapter, msg, 2);

	return ret >= 0 ? 0 : ret;
}

static int ads7142_data_buffer_read(const struct i2c_client *client, int length
				    , void *data)
{
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = I2C_M_RD;
	msg.len = length;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret >= 0 ? 0 : ret;
}

static int ads7142_soft_reset(const struct i2c_client *client)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	buf[0] = ADS7142_OC_GENERAL;
	buf[1] = 0x06;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret >= 0 ? 0 : ret;
}

static int ads7142_get_channel_by_address(struct iio_dev *indio_dev,
					  int address,
					  struct ads7142_channel **channel)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	int i;

	for (i = 0; i < priv->channel_count; i++) {
		if (address == priv->channels[i].channel) {
			*channel = &priv->channels[i];
			return 0;
		}
	}
	return -ENODEV;
}

static int ads7142_sequence_start(struct iio_dev *indio_dev)
{
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);

	return ads7142_reg_write(client, ADS7142_START_SEQUENCE,
				 ADS7142_START_SEQUENCE_SEQ_START);
}

static int ads7142_sequence_abort(struct iio_dev *indio_dev)
{
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);

	return ads7142_reg_write(client, ADS7142_ABORT_SEQUENCE,
				 ADS7142_ABORT_SEQUENCE_SEQ_ABORT);
}

static int ads7142_osc_set(struct iio_dev *indio_dev)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);
	int ret;

	ret = ads7142_reg_write(client, ADS7142_OSC_SEL,
				priv->config.osc_sel ? ADS7142_OSC_SEL_HSZ_LP : 0);
	if (ret)
		return ret;

	return ads7142_reg_write(client, ADS7142_NCLK_SEL,
				 priv->config.n_clk);
}

static int ads7142_input_cfg_set(struct iio_dev *indio_dev)
{
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);

	return ads7142_reg_write(client, ADS7142_CH_INPUT_CFG,
				 ADS7142_CH_INPUT_CFG_TCSE);
}

static int ads7142_dout_format_set(struct iio_dev *indio_dev)
{
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);

	return ads7142_reg_write(client, ADS7142_DOUT_FORMAT_CFG,
				 ADS7142_DOUT_FORMAT_CFG_12BCHDV);
}

static int ads7142_hth_set(struct iio_dev *indio_dev, int channel,
			   int threshold)
{
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);
	int ret;

	if (threshold < 0 || threshold > ADS7142_THRESHOLD_MSK)
		return -EINVAL;

	ret = ads7142_reg_write(client, ADS7142_DWC_HTH_CH0_LSB + channel * 4,
				threshold & 0xFF);
	if (ret)
		return ret;

	ret = ads7142_reg_write(client, ADS7142_DWC_HTH_CH0_MSB + channel * 4,
				(threshold >> 8) & 0xF);
	return ret;
}

static int ads7142_lth_set(struct iio_dev *indio_dev, int channel,
			   int threshold)
{
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);
	int ret;

	if (threshold < 0 || threshold > ADS7142_THRESHOLD_MSK)
		return -EINVAL;

	ret = ads7142_reg_write(client, ADS7142_DWC_LTH_CH0_LSB + channel * 4,
				threshold & 0xFF);
	if (ret)
		return ret;

	ret = ads7142_reg_write(client, ADS7142_DWC_LTH_CH0_MSB + channel * 4,
				(threshold >> 8) & 0xF);
	return ret;
}

static int ads7142_hys_set(struct iio_dev *indio_dev, int channel,
			   int hysteresis)
{
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);
	int ret;

	if (hysteresis < 0 || hysteresis > ADS7142_HYSTERESIS_MSK)
		return -EINVAL;

	ret = ads7142_reg_write(client, ADS7142_DWC_HYS_CH0 + channel,
				hysteresis & ADS7142_HYSTERESIS_MSK);
	return ret;
}

static int ads7142_collect_channel_data(struct iio_dev *indio_dev)
{
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);
	struct ads7142_channel *channel;
	u16 data_buffer;
	u8 data_buffer_status;
	int data_valid;
	int channel_address;
	int value;
	int ret;

	ret = ads7142_reg_read(client, ADS7142_DATA_BUFFER_STATUS,
			       &data_buffer_status);
	if (ret)
		return ret;

	data_buffer_status &= 0x1F;

	do {
		ret = ads7142_data_buffer_read(client, sizeof(data_buffer),
					       &data_buffer);
		if (ret)
			break;
		data_buffer = be16_to_cpu(data_buffer);
		data_valid = data_buffer & 1;
		if (data_valid) {
			channel_address = (data_buffer >> 1) & 0x7;
			value = data_buffer >> 4;
			ret = ads7142_get_channel_by_address(indio_dev,
							     channel_address,
							     &channel);
			if (!ret) {
				channel->data.status = data_valid;
				channel->data.value = value;
			}
		}
	} while (--data_buffer_status);

	return ret;
}

static int ads7142_do_work(struct iio_dev *indio_dev, bool suspend)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);
	int i;
	int alert_ch = 0;
	int ret;

	if (!priv->config.monitoring_mode)
		return 0;

	mutex_lock(&priv->lock);
	priv->monitor_pending = false;

	ret = ads7142_sequence_abort(indio_dev);
	if (ret)
		goto final;

	if (suspend) {
		priv->suspended = true;
		goto final;
	} else {
		priv->suspended = false;
	}

	ret = ads7142_osc_set(indio_dev);
	if (ret)
		goto final;

	ret = ads7142_input_cfg_set(indio_dev);
	if (ret)
		goto final;

	ret = ads7142_dout_format_set(indio_dev);
	if (ret)
		goto final;

	ret = ads7142_reg_write(client, ADS7142_DATA_BUFFER_OPMODE,
				ADS7142_DATA_BUFFER_OPMODE_PRE_ALERT);
	if (ret)
		goto final;

	ret = ads7142_reg_write(client, ADS7142_OPMODE_SEL,
				ADS7142_OPMODE_SEL_MONITORING);
	if (ret)
		goto final;

	ret = ads7142_reg_write(client, ADS7142_AUTO_SEQ_CHEN,
				ADS7142_AUTO_SEQ_CHEN_CH0
				| ADS7142_AUTO_SEQ_CHEN_CH1);
	if (ret)
		goto final;

	ret = ads7142_reg_write(client, ADS7142_PRE_ALT_EVT_CNT, 0);
	if (ret)
		goto final;

	ret = ads7142_reg_write(client, ADS7142_ALT_LOW_FLAGS,
				ADS7142_ALT_LOW_FLAGS_CH0
				| ADS7142_ALT_LOW_FLAGS_CH1);
	if (ret)
		goto final;

	ret = ads7142_reg_write(client, ADS7142_ALT_HIGH_FLAGS,
				ADS7142_ALT_HIGH_FLAGS_CH0
				| ADS7142_ALT_HIGH_FLAGS_CH1);
	if (ret)
		goto final;

	for (i = 0; i < priv->channel_count; i++) {
		ret = ads7142_hth_set(indio_dev, priv->channels[i].channel,
				      priv->channels[i].config.high_threshold);
		if (ret)
			goto final;

		ret = ads7142_lth_set(indio_dev, priv->channels[i].channel,
				      priv->channels[i].config.low_threshold);
		if (ret)
			goto final;

		ret = ads7142_hys_set(indio_dev, priv->channels[i].channel,
				      priv->channels[i].config.hysteresis);
		if (ret)
			goto final;

		if (priv->channels[i].config.alert_low ||
		    priv->channels[i].config.alert_high) {
			alert_ch |= 1 << priv->channels[i].channel;
		}
	}

	ret = ads7142_reg_write(client, ADS7142_ALERT_DWC_EN,
				alert_ch ? ADS7142_ALERT_DWC_EN_BLOCK_EN : 0);
	if (ret)
		goto final;

	ret = ads7142_reg_write(client, ADS7142_ALERT_CHEN,
				alert_ch);
	if (ret)
		goto final;

	if (alert_ch) {
		ret = ads7142_sequence_start(indio_dev);
		priv->monitor_pending = !ret;
	}
final:
	mutex_unlock(&priv->lock);
	return ret;
}

static int ads7142_read_channel_manual(struct iio_dev *indio_dev,
				       int address, int *val)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);
	u16 data_buffer;
	int ret;

	if (address < 0 || address > 1)
		return -EINVAL;

	mutex_lock(&priv->lock);
	ret = ads7142_sequence_abort(indio_dev);
	if (ret)
		goto final;

	ret = ads7142_osc_set(indio_dev);
	if (ret)
		goto final;

	ret = ads7142_input_cfg_set(indio_dev);
	if (ret)
		goto final;

	ret = ads7142_dout_format_set(indio_dev);
	if (ret)
		goto final;

	ret = ads7142_reg_write(client, ADS7142_OPMODE_SEL,
				ADS7142_OPMODE_SEL_MANUALSEQ);
	if (ret)
		goto final;

	ret = ads7142_reg_write(client, ADS7142_AUTO_SEQ_CHEN,
				1 << address);
	if (ret)
		goto final;

	ret = ads7142_sequence_start(indio_dev);
	if (ret)
		goto final;

	ret = ads7142_data_buffer_read(client, sizeof(data_buffer),
				       &data_buffer);
	if (ret)
		goto abort;

	*val = (be16_to_cpu(data_buffer) >> 4);

abort:
	ret = ads7142_sequence_abort(indio_dev);
final:
	mutex_unlock(&priv->lock);
	return ret;
}

static int ads7142_read_channel_monitor(struct iio_dev *indio_dev,
					int address, int *val)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct ads7142_channel *channel;
	int ret;

	if (address < 0 || address > 1)
		return -EINVAL;

	ret = ads7142_get_channel_by_address(indio_dev, address, &channel);
	if (ret)
		return ret;

	mutex_lock(&priv->lock);
	if (!channel->data.status) {
		ret = -EINVAL;
	} else {
		*val = channel->data.value;
		channel->data.status = 0;
		ret = 0;
	}
	mutex_unlock(&priv->lock);
	return ret;
}

static int ads7142_read_channel(struct iio_dev *indio_dev,
				int address, int *val)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);

	if (priv->config.monitoring_mode)
		return ads7142_read_channel_monitor(indio_dev, address, val);
	return ads7142_read_channel_manual(indio_dev, address, val);
}

static irqreturn_t ads7142_ist(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct ads7142_channel *channel;
	u8 low_flags;
	u8 high_flags;
	u8 seq_st;
	int i;
	int ret;
	s64 timestamp = iio_get_time_ns(indio_dev);

	mutex_lock(&priv->lock);
	if (!priv->config.monitoring_mode || !priv->monitor_pending ||
	    priv->suspended) {
		mutex_unlock(&priv->lock);
		return IRQ_NONE;
	}

	ret = ads7142_reg_read(client, ADS7142_SEQUENCE_STATUS, &seq_st);
	if (ret) {
		dev_err(indio_dev->dev.parent,
			"%s: SEQUENCE_STATUS reg read error(%i)",
			__func__, ret);
		goto final;
	}

	if ((seq_st & ADS7142_SEQUENCE_STATUS_SEQ_ERR_ST_MSK)
	    != ADS7142_SEQUENCE_STATUS_SEQ_ENABLED) {
		dev_err(indio_dev->dev.parent,
			"%s: SEQUENCE_STATUS error(%i)",
			__func__, seq_st);
		goto final;
	}

	ret = ads7142_reg_read(client, ADS7142_ALT_LOW_FLAGS, &low_flags);
	if (ret) {
		dev_err(indio_dev->dev.parent,
			"%s: ALT_LOW_FLAGS reg read error(%i)",
			__func__, ret);
		goto final;
	}

	ret = ads7142_reg_read(client, ADS7142_ALT_HIGH_FLAGS, &high_flags);
	if (ret) {
		dev_err(indio_dev->dev.parent,
			"%s: ALT_HIGH_FLAGS reg read error(%i)",
			__func__, ret);
		goto final;
	}

	ret = ads7142_collect_channel_data(indio_dev);
	if (ret)
		goto final;

	for (i = 0; i < priv->channel_count; i++) {
		channel = &priv->channels[i];
		if (channel->config.alert_low &&
		    (low_flags & (1 << channel->channel))) {
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    channel->channel,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_FALLING),
				       timestamp);
		}

		if (channel->config.alert_high &&
		    (low_flags & (1 << channel->channel))) {
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							    channel->channel,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_RISING),
				       timestamp);
		}
	}

final:
	mutex_unlock(&priv->lock);

	ret = ads7142_do_work(indio_dev, false);
	if (ret) {
		dev_err(indio_dev->dev.parent,
			"%s: start operation error(%i)",
			__func__, ret);
		return IRQ_NONE;
	}
	return IRQ_HANDLED;
}

static int ads7142_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = ads7142_read_channel(indio_dev, chan->address, val);
		if (!ret)
			ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = priv->config.n_clk;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		if (IS_ERR(priv->vref)) {
			ret = -EINVAL;
		} else {
			*val = regulator_get_voltage(priv->vref) / 1000;
			*val2 = chan->scan_type.realbits;
			ret = IIO_VAL_FRACTIONAL_LOG2;
		}
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int ads7142_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		priv->config.n_clk = val;
		if (priv->config.monitoring_mode)
			ret = ads7142_do_work(indio_dev, false);
		else
			ret = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ads7142_read_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int *val, int *val2)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct ads7142_channel *channel;
	int ret;

	if (!priv->config.monitoring_mode)
		return -EINVAL;

	ret = ads7142_get_channel_by_address(indio_dev, chan->address,
					     &channel);
	if (ret)
		return ret;

	switch (info) {
	case IIO_EV_INFO_VALUE:
		if (dir == IIO_EV_DIR_RISING)
			*val = channel->config.high_threshold;
		else
			*val = channel->config.low_threshold;
		ret = IIO_VAL_INT;
	break;
	case IIO_EV_INFO_HYSTERESIS:
		*val = channel->config.hysteresis;
		ret = IIO_VAL_INT;
	break;
	default:
		ret = -EINVAL;
	break;
	}
	return ret;
}

static int ads7142_write_event_value(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     enum iio_event_info info,
				     int val, int val2)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct ads7142_channel *channel;
	bool have_to_do = false;
	int ret;

	if (!priv->config.monitoring_mode)
		return -EINVAL;

	ret = ads7142_get_channel_by_address(indio_dev, chan->address,
					     &channel);
	if (ret)
		return ret;

	mutex_lock(&priv->lock);
	switch (info) {
	case IIO_EV_INFO_VALUE:
		if (val < 0 || val > ADS7142_THRESHOLD_MSK) {
			ret = -EINVAL;
		} else {
			if (dir == IIO_EV_DIR_RISING) {
				if (val != channel->config.high_threshold) {
					channel->config.high_threshold = val;
					have_to_do = true;
				}
			} else {
				if (val != channel->config.low_threshold) {
					channel->config.low_threshold = val;
					have_to_do = true;
				}
			}
		}
	break;
	case IIO_EV_INFO_HYSTERESIS:
		if (val < 0 || val > ADS7142_HYSTERESIS_MSK) {
			ret = -EINVAL;
		} else {
			if (val != channel->config.hysteresis) {
				channel->config.hysteresis = val;
				have_to_do = true;
			}
		}
	break;
	default:
		ret = -EINVAL;
	break;
	}
	mutex_unlock(&priv->lock);
	if (!ret && have_to_do)
		ret = ads7142_do_work(indio_dev, false);
	return ret;
}

static int ads7142_read_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct ads7142_channel *channel;
	int ret;

	if (!priv->config.monitoring_mode)
		return -EINVAL;

	if (type != IIO_EV_TYPE_THRESH)
		return -EINVAL;

	ret = ads7142_get_channel_by_address(indio_dev, chan->address,
					     &channel);
	if (ret)
		return ret;

	if (dir == IIO_EV_DIR_RISING)
		ret = channel->config.alert_high ? 1 : 0;
	else
		ret = channel->config.alert_low ? 1 : 0;

	return ret;
}

static int ads7142_write_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      int state)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct ads7142_channel *channel;
	bool have_to_do = false;
	int ret;

	if (!priv->config.monitoring_mode)
		return -EINVAL;

	if (type != IIO_EV_TYPE_THRESH)
		return -EINVAL;

	ret = ads7142_get_channel_by_address(indio_dev, chan->address,
					     &channel);
	if (ret)
		return ret;

	mutex_lock(&priv->lock);
	if (dir == IIO_EV_DIR_RISING) {
		if (channel->config.alert_high != state) {
			channel->config.alert_high = state;
			have_to_do = true;
		}
	} else {
		if (channel->config.alert_low != state) {
			channel->config.alert_low = state;
			have_to_do = true;
		}
	}
	mutex_unlock(&priv->lock);

	if (have_to_do)
		ret = ads7142_do_work(indio_dev, false);

	return ret;
}

static const struct iio_info ads7142_iio_info = {
	.read_raw		= ads7142_read_raw,
	.write_raw		= ads7142_write_raw,
	.read_event_value	= ads7142_read_event_value,
	.write_event_value	= ads7142_write_event_value,
	.read_event_config	= ads7142_read_event_config,
	.write_event_config	= ads7142_write_event_config,
};

static int ads7142_parse_channel_config_of(struct device *dev,
					   struct iio_dev *indio_dev)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct device_node *channel_node;
	struct iio_chan_spec *iio_channels;
	struct iio_chan_spec *iio_channel;
	struct ads7142_channel *ads_channel;
	int channel_index = 0;
	int ret;

	priv->channel_count = of_get_available_child_count(dev->of_node);
	if (!priv->channel_count) {
		dev_err(dev, "dt: there is no channel definition");
		return -ENODEV;
	}

	priv->channels = devm_kcalloc(dev, priv->channel_count,
				      sizeof(*priv->channels),
				      GFP_KERNEL);
	if (!priv->channels)
		return -ENOMEM;

	indio_dev->num_channels = priv->channel_count;
	iio_channels = devm_kcalloc(dev, priv->channel_count,
				    sizeof(*iio_channels),
				    GFP_KERNEL);
	if (!iio_channels)
		return -ENOMEM;

	indio_dev->channels = iio_channels;

	for_each_available_child_of_node(dev->of_node, channel_node) {
		ads_channel = &priv->channels[channel_index];

		ret = of_property_read_u32(channel_node, "reg",
					   &ads_channel->channel);
		if (ret)
			goto err;

		iio_channel = &iio_channels[channel_index];
		iio_channel->type = IIO_VOLTAGE;
		iio_channel->indexed = 1;
		iio_channel->info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
						  | BIT(IIO_CHAN_INFO_SAMP_FREQ);
		if (!IS_ERR(priv->vref))
			iio_channel->info_mask_separate |= BIT(IIO_CHAN_INFO_SCALE);
		iio_channel->scan_type.sign = 'u';
		iio_channel->scan_type.realbits = 12;
		iio_channel->scan_type.storagebits = 16;
		iio_channel->scan_type.shift = 0;
		iio_channel->scan_type.endianness = IIO_CPU;
		iio_channel->address = ads_channel->channel;
		iio_channel->scan_index = ads_channel->channel;
		iio_channel->channel = ads_channel->channel;
		if (priv->config.monitoring_mode) {
			iio_channel->event_spec = ads7142_events;
			iio_channel->num_event_specs = ARRAY_SIZE(ads7142_events);
		}

		ads_channel->config.high_threshold = ADS7142_THRESHOLD_MSK;
		ret = of_property_read_u32(channel_node, "ti,threshold-rising",
					   &ads_channel->config.high_threshold);
		ads_channel->config.alert_high = !ret;
		ret = of_property_read_u32(channel_node, "ti,threshold-falling",
					   &ads_channel->config.low_threshold);
		ads_channel->config.alert_low = !ret;
		ret = of_property_read_u32(channel_node, "ti,hysteresis",
					   &ads_channel->config.hysteresis);
		channel_index++;
	}

	return 0;
err:
	of_node_put(channel_node);
	return ret;
}

static int ads7142_parse_config_of(struct device *dev,
				   struct iio_dev *indio_dev)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);

	priv->config.osc_sel = of_property_read_bool(dev->of_node,
						     "ti,osc-sel");
	of_property_read_u32(dev->of_node, "ti,n-clk", &priv->config.n_clk);
	priv->config.monitoring_mode = of_property_read_bool(dev->of_node,
							     "ti,monitoring-mode");

	return ads7142_parse_channel_config_of(dev, indio_dev);
}

static int ads7142_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct ads7142_priv *priv;
	int ret;

	ret = ads7142_soft_reset(client);
	if (ret)
		return ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);

	indio_dev->dev.parent = &client->dev;
	indio_dev->dev.of_node = client->dev.of_node;
	indio_dev->name = ADS7142_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ads7142_iio_info;

	mutex_init(&priv->lock);

	priv->vref = devm_regulator_get_optional(&client->dev, "vref");
	if (!IS_ERR(priv->vref)) {
		ret = regulator_enable(priv->vref);
		if (ret)
			goto err;
	}

	priv->power = devm_regulator_get_optional(&client->dev, "power");
	if (!IS_ERR(priv->power)) {
		ret = regulator_enable(priv->power);
		if (ret)
			goto err_regulator;
	}

	ret = ads7142_parse_config_of(&client->dev, indio_dev);
	if (ret)
		goto err_regulator;

	if (!client->irq && priv->config.monitoring_mode) {
		ret = -EINVAL;
		dev_err(&client->dev, "Interrupt not specified\n");
		goto err_regulator;
	}
	if (client->irq && priv->config.monitoring_mode) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
						NULL, ads7142_ist,
						IRQF_ONESHOT | IRQF_SHARED,
						dev_name(&client->dev),
						indio_dev);
		if (ret) {
			dev_err(&client->dev, "Unable to request IRQ %i",
				client->irq);
			goto err_regulator;
		}
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&client->dev, "Failed to register iio device\n");
		goto err_regulator;
	}

	return ads7142_do_work(indio_dev, false);
err_regulator:
	if (!IS_ERR(priv->vref))
		regulator_disable(priv->vref);
	if (!IS_ERR(priv->power))
		regulator_disable(priv->power);
err:
	mutex_destroy(&priv->lock);

	return ret;
}

static int ads7142_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads7142_priv *priv = iio_priv(indio_dev);

	if (!IS_ERR(priv->vref))
		regulator_disable(priv->vref);
	if (!IS_ERR(priv->power))
		regulator_disable(priv->power);
	mutex_destroy(&priv->lock);
	iio_device_unregister(indio_dev);

	return 0;
}

static int __maybe_unused ads7142_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct ads7142_priv *priv = iio_priv(indio_dev);
	int ret;

	if (priv->config.monitoring_mode) {
		ret = ads7142_do_work(indio_dev, true);
		if (ret)
			return ret;
	}
	if (!IS_ERR(priv->vref))
		regulator_disable(priv->vref);
	if (!IS_ERR(priv->power))
		regulator_disable(priv->power);
	return 0;
}

static int __maybe_unused ads7142_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct ads7142_priv *priv = iio_priv(indio_dev);
	int ret;

	if (!IS_ERR(priv->vref)) {
		ret = regulator_enable(priv->vref);
		if (ret)
			return ret;
	}
	if (!IS_ERR(priv->power)) {
		ret = regulator_enable(priv->power);
		if (ret)
			return ret;
	}
	if (priv->config.monitoring_mode) {
		ret = ads7142_do_work(indio_dev, false);
		if (ret)
			return ret;
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(ads7142_pm_ops, ads7142_suspend, ads7142_resume);

static const struct i2c_device_id ads7142_id[] = {
	{ ADS7142_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ads7142_id);

static const struct of_device_id ads7142_of_match[] = {
	{ .compatible = "ti,ads7142" },
	{}
};
MODULE_DEVICE_TABLE(of, ads7142_of_match);

static struct i2c_driver ads7142_driver = {
	.driver = {
		.name = ADS7142_NAME,
		.of_match_table = ads7142_of_match,
		.pm = &ads7142_pm_ops,
	},
	.probe		= ads7142_probe,
	.remove		= ads7142_remove,
	.id_table	= ads7142_id,
};

module_i2c_driver(ads7142_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jozsef Horvath <info@ministro.hu>");
MODULE_DESCRIPTION("Texas Instruments ADS7142 ADC driver");
