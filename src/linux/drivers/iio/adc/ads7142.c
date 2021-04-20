// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Jozsef Horvath <info@ministro.hu>
 *
 */
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/iio.h>

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
#define ADS7142_OPMODE_SEL_MANUALCH0		(0 << 0)
#define ADS7142_OPMODE_SEL_MANUALSEQ		(4 << 0)
#define ADS7142_OPMODE_SEL_MONITORING		(6 << 0)
#define ADS7142_OPMODE_SEL_HIGHPREC		(7 << 0)
#define ADS7142_OPMODE_STATUS			0x00
#define ADS7142_OPMODE_STATUS_OPMODE_MSK	(4 << 0)
#define ADS7142_OPMODE_STATUS_OPMODE_MANUAL	(0 << 0)
#define ADS7142_OPMODE_STATUS_OPMODE_AUTO	(2 << 0)
#define ADS7142_OPMODE_STATUS_OPMODE_HIGHPREC	(3 << 0)
#define ADS7142_OPMODE_STATUS_HS_MODE		(1 << 2)

/* Input config register */
#define ADS7142_CH_INPUT_CFG			0x24
#define ADS7142_CH_INPUT_CFG_TCSE		(0 << 0)
#define ADS7142_CH_INPUT_CFG_SCSE		(1 << 0)
#define ADS7142_CH_INPUT_CFG_SCPD		(2 << 0)
/* Analog mux and sequencer registers */
#define ADS7142_AUTO_SEQ_CHEN			0x20
#define ADS7142_AUTO_SEQ_CHEN_CH0		(1 << 0)
#define ADS7142_AUTO_SEQ_CHEN_CH1		(1 << 1)
#define ADS7142_START_SEQUENCE			0x1E
#define ADS7142_START_SEQUENCE_SEQ_START	(1 << 0)
#define ADS7142_ABORT_SEQUENCE			0x1F
#define ADS7142_ABORT_SEQUENCE_SEQ_ABORT	(1 << 0)
#define ADS7142_SEQUENCE_STATUS			0x04
#define ADS7142_SEQUENCE_STATUS_SEQ_ERR_ST_MSK	(4 << 1)
#define ADS7142_SEQUENCE_STATUS_SEQ_DISABLED	(0 << 1)
#define ADS7142_SEQUENCE_STATUS_SEQ_ENABLED	(1 << 1)
#define ADS7142_SEQUENCE_STATUS_SEQ_ERROR	(3 << 1)
/* Oscillator and timing control registers */
#define ADS7142_OSC_SEL				0x18
#define ADS7142_OSC_SEL_HSZ_LP			(1 << 0)
#define ADS7142_NCLK_SEL			0x19
/* Data buffer control register */
#define ADS7142_DATA_BUFFER_OPMODE		0x2C
#define ADS7142_DATA_BUFFER_OPMODE_STOP_BURST	(0 << 0)
#define ADS7142_DATA_BUFFER_OPMODE_START_BURST	(1 << 0)
#define ADS7142_DATA_BUFFER_OPMODE_PRE_ALERT	(4 << 0)
#define ADS7142_DATA_BUFFER_OPMODE_POST_ALERT	(6 << 0)
#define ADS7142_DOUT_FORMAT_CFG			0x28
#define ADS7142_DOUT_FORMAT_CFG_12B		(0 << 0)
#define ADS7142_DOUT_FORMAT_CFG_12BCH		(1 << 0)
#define ADS7142_DOUT_FORMAT_CFG_12BCHDV		(2 << 0)
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
#define ADS7142_ALT_HIGH_FLAGS			0x0E

struct ads7142_channel_data {
	int status;
	int data;
};

struct ads7142_channel_config {
	int status;
	int alert;
	int pre_alert_cnt;
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
	struct ads7142_config config;
	int channel_count;
	struct ads7142_channel *channels;
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

static int ads7142_data_buffer_read(const struct i2c_client *client, int length, void *data)
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

static int ads7142_sequence_start(struct iio_dev *indio_dev)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);

	return ads7142_reg_write(client, ADS7142_START_SEQUENCE,
				 ADS7142_START_SEQUENCE_SEQ_START);
}

static int ads7142_sequence_abort(struct iio_dev *indio_dev)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
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
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);

	return ads7142_reg_write(client, ADS7142_CH_INPUT_CFG,
				 ADS7142_CH_INPUT_CFG_TCSE);
}

static int ads7142_dout_format_set(struct iio_dev *indio_dev)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);

	return ads7142_reg_write(client, ADS7142_DOUT_FORMAT_CFG,
				 ADS7142_DOUT_FORMAT_CFG_12BCHDV);
}

static int ads7142_get_channel_config_by_address(struct iio_dev *indio_dev,
						 int address,
						 struct ads7142_channel_config **config)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	int i;

	for (i = 0; i < priv->channel_count; i++) {
		if(address == priv->channels[i].channel) {
			*config = &priv->channels[i].config;
			return 0;
		}
	}
	return -ENODEV;
}

static int ads7142_do_work(struct iio_dev *indio_dev)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct i2c_client *client = to_i2c_client(indio_dev->dev.parent);

	mutex_lock(&priv->lock);
	if (priv->config.monitoring_mode) {

	}
	mutex_unlock(&priv->lock);
	return 0;
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

	ret = ads7142_data_buffer_read(client, sizeof(data_buffer), &data_buffer);
	if (ret)
		goto abort;

	*val = (be16_to_cpu(data_buffer) >> 4);

abort:
	ret = ads7142_sequence_abort(indio_dev);
final:
	mutex_unlock(&priv->lock);
	return ret;
}

static int ads7142_read_channel(struct iio_dev *indio_dev,
				int address, int *val)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);

	if (!priv->config.monitoring_mode)
		return ads7142_read_channel_manual(indio_dev, address, val);
	return -EINVAL;
}

static irqreturn_t ads7142_ist(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct ads7142_priv *priv = iio_priv(indio_dev);

	return IRQ_HANDLED;
}

static int ads7142_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct ads7142_channel_config *channel_config;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = ads7142_read_channel(indio_dev, chan->address, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = priv->config.n_clk;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (IS_ERR(priv->vref)) {
			return -EINVAL;
		} else {
			*val = regulator_get_voltage(priv->vref) / 1000;
			*val2 = chan->scan_type.realbits;
			return IIO_VAL_FRACTIONAL_LOG2;
		}
	default:
		return -EINVAL;
	}
	return 0;
}

static int ads7142_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		priv->config.n_clk = val;
		if (priv->config.monitoring_mode) {
			ret = ads7142_do_work(indio_dev);
		} else {
			ret = 0;
		}
	break;
	default:
		ret = -EINVAL;
	break;
	}

	return ret;
}

static const struct iio_info ads7142_iio_info = {
	.read_raw	= ads7142_read_raw,
	.write_raw	= ads7142_write_raw,
};

static int ads7142_parse_channel_config_of(struct device *dev,
					   struct iio_dev *indio_dev)
{
	struct ads7142_priv *priv = iio_priv(indio_dev);
	struct device_node *channel_node;
	struct iio_chan_spec *iio_channels;
	struct iio_chan_spec *iio_channel;
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
		ret = of_property_read_u32(channel_node, "reg",
					   &priv->channels[channel_index].channel);
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
		iio_channel->address = priv->channels[channel_index].channel;
		iio_channel->scan_index = priv->channels[channel_index].channel;
		iio_channel->channel = priv->channels[channel_index].channel;
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

	priv->vref = devm_regulator_get(&client->dev, "vref");
	if (!IS_ERR(priv->vref)) {
		ret = regulator_enable(priv->vref);
		if (ret)
			goto err;
	}

	ret = ads7142_parse_config_of(&client->dev, indio_dev);
	if (ret)
		goto err_regulator;

	if (!client->irq && priv->config.monitoring_mode) {
		ret = -EINVAL;
		dev_err(&client->dev, "Monitoring mode requested,"
		       "but no IRQ specified\n");
		goto err_regulator;
	}
	else if (client->irq && priv->config.monitoring_mode) {
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

	return ads7142_do_work(indio_dev);
err_regulator:
	if (!IS_ERR(priv->vref))
		regulator_disable(priv->vref);
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
	mutex_destroy(&priv->lock);
	iio_device_unregister(indio_dev);

	return 0;
}

#ifdef CONFIG_PM
static int ads7142_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct ads7142_priv *priv = iio_priv(indio_dev);

	if (!IS_ERR(priv->vref))
		regulator_disable(priv->vref);
	return 0;
}

static int ads7142_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct ads7142_priv *priv = iio_priv(indio_dev);
	int ret;

	if (!IS_ERR(priv->vref)) {
		ret = regulator_enable(priv->vref);
		if (ret)
			return ret;
	}
	return 0;
}
#endif

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
