/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT u_blox_max7q

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <app/drivers/sensor/location.h>

#include "minmea.h"

LOG_MODULE_REGISTER(max7q, CONFIG_SENSOR_LOG_LEVEL);

struct nmea_data {
	float latitude;
	float longitude;
	uint8_t satellites;
	float speed;
	float altitude;
	float accuracy;
};

/** NMEA 0183 sentences vary in length, but each sentence is limited to 79 characters or
 * less. This length limit excludes the $ and the [CR][LF] characters. The data field block,
 * including delimiters is limited to 74 characters or less
 */
#define BUF_SIZE_MAX 85
struct max7q_data {
	/** RX/TX buffer. */
	uint8_t buf[BUF_SIZE_MAX];
	/** RX/TX buffer pointer. */
	uint8_t *buf_ptr;
	/** RX/TX buffer counter. */
	uint8_t buf_ctr;
	/** NMEA sample. */
	struct nmea_data nmea;

	const struct device *dev;
	/** NMEA sample ready handler */
	sensor_trigger_handler_t nmea_handler;
	struct k_work trig_work;
	bool trigger_enable;
	bool gps_power_enable;
};

struct max7q_config {
	/** UART instance. */
	const struct device *uart;
#ifdef CONFIG_MAX7Q_POWER_ENABLE
	/** enable gpio (optional). */
	struct gpio_dt_spec enable;
#endif
};

/** @brief GPS UART configuration. */
static const struct uart_config uart_config = {
	//.baudrate = 9600U,
	.baudrate = DT_PROP(DT_BUS(DT_ALIAS(gps0)), current_speed),
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
};

static void decode_nmea_handler(struct max7q_data *data)
{
	char *line = data->buf;
	struct nmea_data *nmea = &data->nmea;

	switch (minmea_sentence_id(line, false)) {
	case MINMEA_SENTENCE_RMC: {
		struct minmea_sentence_rmc frame;
		if (minmea_parse_rmc(&frame, line)) {
			if (!frame.valid) {
				LOG_DBG("$xxRMC sentence is not valid");
				break;
			}
			LOG_DBG("$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d",
				frame.latitude.value, frame.latitude.scale,
				frame.longitude.value, frame.longitude.scale,
				frame.speed.value, frame.speed.scale);
#if 0
			printk("$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
				minmea_tocoord(&frame.latitude),
				minmea_tocoord(&frame.longitude),
				minmea_tofloat(&frame.speed));
#endif
		}
		else {
			LOG_DBG("$xxRMC sentence is not parsed");
		}
	} break;

	case MINMEA_SENTENCE_GGA: {
		struct minmea_sentence_gga frame;
		struct sensor_value lat, lng, alt, acc;
		int err;
		if (minmea_parse_gga(&frame, line)) {
			if (frame.fix_quality < 1) {
				LOG_DBG("$xxGGA sentence is not valid");
				break;
			}
			LOG_DBG("$xxGGA: fix quality: %d\n", frame.fix_quality);
			nmea->latitude = minmea_tocoord(&frame.latitude);
			nmea->longitude = minmea_tocoord(&frame.longitude);
			nmea->satellites = frame.satellites_tracked;
			nmea->altitude = minmea_tocoord(&frame.altitude) * 100;
			nmea->accuracy = minmea_tocoord(&frame.hdop);
			/** %f not work with CDC_ACM console */
			/*
			printk("$xxGGA: lat: %f, lng: %f, alt: %f, acc: %f\n",
				nmea->latitude,
				nmea->longitude,
				nmea->altitude,
				nmea->accuracy);
			*/

			err = sensor_value_from_double(&lat, nmea->latitude);
			err |= sensor_value_from_double(&lng, nmea->longitude);
			err |= sensor_value_from_double(&alt, nmea->altitude);
			err |= sensor_value_from_double(&acc, nmea->accuracy);
			if (err) {
				LOG_ERR("Can't convert from double to sensor value.");
				break;
			}

			LOG_DBG("$xxGGA: lat: %d.%06d, lng: %d.%06d, sat: %u, alt: %d.%06d, acc: %d.%06d",
					lat.val1, lat.val2,
					lng.val1, lng.val2,
					nmea->satellites,
					alt.val1, alt.val2,
					acc.val1, acc.val2);

			/** Inform client handlers that we have location fix */
			if (data->trigger_enable) {
				k_work_submit(&data->trig_work);
			}
		} else {
			LOG_DBG("$xxGGA sentence is not parsed");
		}
	} break;

	case MINMEA_SENTENCE_ZDA: {
		struct minmea_sentence_zda frame;
		if (minmea_parse_zda(&frame, line)) {
			LOG_DBG("$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d",
				frame.time.hours,
				frame.time.minutes,
				frame.time.seconds,
				frame.date.day,
				frame.date.month,
				frame.date.year,
				frame.hour_offset,
				frame.minute_offset);
		} else {
			LOG_DBG("$xxZDA sentence is not parsed");
		}
	} break;

	case MINMEA_INVALID: {
		LOG_DBG("$xxxxx sentence is not valid");
	} break;

	default: {
		LOG_DBG("$xxxxx sentence is not parsed");
	} break;
	}
}

/**
 * @brief UART RX handler.
 *
 * @param dev Sensor instance.
 */
static void uart_cb_rx_handler(const struct device *dev)
{
	const struct max7q_config *config = dev->config;
	struct max7q_data *data = dev->data;
	uint8_t c;

	if (uart_fifo_read(config->uart, &c, 1) != 1) {
		LOG_ERR("Failed to read UART");
		return;
	}

	if (c == '$') {
		/* Restart a new frame */
		data->buf_ptr = data->buf;
		data->buf_ctr = 0U;
	}

	if (data->buf_ctr < BUF_SIZE_MAX) {
		*data->buf_ptr++ = c;
		data->buf_ctr++;
	}

	if (c == '\n') {
		LOG_HEXDUMP_DBG(data->buf, data->buf_ctr, "RX");
		decode_nmea_handler(data);
		data->buf_ptr = data->buf;
		data->buf_ctr = 0U;
		memset(data->buf_ptr, 0, BUF_SIZE_MAX);
	}
}

/**
 * @brief UART TX handler.
 *
 * @param dev Sensor instance.
 */
static void uart_cb_tx_handler(const struct device *dev)
{
	const struct max7q_config *config = dev->config;
	struct max7q_data *data = dev->data;
	int n;

	if (data->buf_ctr > 0U) {
		n = uart_fifo_fill(config->uart, data->buf_ptr, data->buf_ctr);
		data->buf_ctr -= (uint8_t)n;
		data->buf_ptr += (uint8_t)n;
		return;
	}

	if (uart_irq_tx_complete(config->uart) > 0) {
		/* Disable transmission */
		data->buf_ptr = data->buf;
		uart_irq_tx_disable(config->uart);
		uart_irq_rx_enable(config->uart);
	}
}

/**
 * @brief UART IRQ handler.
 *
 * @param dev UART device instance.
 * @param user_data User data (sensor instance).
 */
static void uart_cb_handler(const struct device *dev, void *user_data)
{
	const struct device *sensor = user_data;

	if ((uart_irq_update(dev) > 0) && (uart_irq_is_pending(dev) > 0)) {
		if (uart_irq_rx_ready(dev)) {
			uart_cb_rx_handler(sensor);
		}

		if (uart_irq_tx_ready(dev)) {
			uart_cb_tx_handler(sensor);
		}
	}
}

/*******************************************************************************
 * API
 ******************************************************************************/

static int max7q_sample_fetch(const struct device *dev,
			      enum sensor_channel chan)
{
	//const struct max7q_config *config = dev->config;

	LOG_DBG("dev_name: %s, chan: %d", dev->name, chan);

	return 0;
}

static int max7q_channel_get(const struct device *dev,enum sensor_channel chan,
			     struct sensor_value *val)
{
	struct max7q_data *data = dev->data;
	struct nmea_data *nmea = &data->nmea;
	double tmp = 0.0f;

	switch((enum location_channel)chan) {
	case SENSOR_CHAN_NAV_LATITUDE:
		tmp = nmea->latitude;;
		break;
	case SENSOR_CHAN_NAV_LONGITUDE:
		tmp = nmea->longitude;;
		break;
	case SENSOR_CHAN_NAV_SATELLITES:
		tmp = (double)nmea->satellites;;
		break;
	case SENSOR_CHAN_NAV_ALTITUDE:
		tmp = nmea->altitude;;
		break;
	case SENSOR_CHAN_NAV_ACCURACY:
		tmp = nmea->accuracy;;
		break;
	default:
		return -ENOTSUP;
	}

	return sensor_value_from_double(val, tmp);
}

static int max7q_config_attr(const struct device *dev,
		enum sensor_channel chan,
		enum sensor_attribute attr,
		const struct sensor_value *val)
{
	struct max7q_data *data = dev->data;
#ifdef CONFIG_MAX7Q_POWER_ENABLE
	const struct max7q_config *config = dev->config;
#endif
	bool enable;

	switch (attr) {
	case SENSOR_ATTR_ALERT:
		enable = !!(val->val1);
		if (data->trigger_enable == enable) {
			return -EALREADY;
		}
		data->trigger_enable = enable;
		LOG_INF("NMEA Trigger: %d", enable);
		break;
	case SENSOR_ATTR_CONFIGURATION:
		enable = !!(val->val1);
		if (data->gps_power_enable == enable) {
			return -EALREADY;
		}
		data->gps_power_enable = enable;
		if (enable) {
#ifdef CONFIG_MAX7Q_POWER_ENABLE
			LOG_INF("GPS power ON");
			gpio_pin_set_dt(&config->enable, 1);
#endif
			uart_irq_rx_enable(config->uart);
		} else {
			uart_irq_rx_disable(config->uart);
#ifdef CONFIG_MAX7Q_POWER_ENABLE
			gpio_pin_set_dt(&config->enable, 0);
			LOG_INF("GPS power OFF");
#endif
		}
		break;
	default:
		LOG_DBG("%s: attribute not supported.", dev->name);
		return -ENOTSUP;
	}

	return 0;
}

static int max7q_config_get(const struct device *dev,
		enum sensor_channel chan,
		enum sensor_attribute attr,
		const struct sensor_value *val)
{
	struct max7q_data *data = dev->data;

	switch (attr) {
	case SENSOR_ATTR_ALERT:
		return data->trigger_enable;
	case SENSOR_ATTR_CONFIGURATION:
		return data->gps_power_enable;
	default:
		LOG_DBG("%s: attribute not supported.", dev->name);
	}

	return -ENOTSUP;
}

static int max7q_attr_set(const struct device *dev, enum sensor_channel chan,
			  enum sensor_attribute attr,
			  const struct sensor_value *val)
{
	LOG_DBG("dev_name: %s, chan: %d, attr: %d", dev->name, chan, attr);

	switch (chan) {
	case SENSOR_CHAN_ALL:
		return max7q_config_attr(dev, chan, attr, val);
		break;
	default:
		LOG_WRN("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	return 0;
}

static int max7q_attr_get(const struct device *dev, enum sensor_channel chan,
			  enum sensor_attribute attr,
			  struct sensor_value *val)
{
	LOG_DBG("dev_name: %s, chan: %d, attr: %d", dev->name, chan, attr);

	switch (chan) {
	case SENSOR_CHAN_ALL:
		return max7q_config_get(dev, chan, attr, val);
		break;
	default:
		LOG_WRN("attr_get() not supported on this channel.");
		return -ENOTSUP;
	}

	return 0;
}

int max7q_trigger_set(const struct device *dev,
		const struct sensor_trigger *trig,
		sensor_trigger_handler_t handler)
{
	struct max7q_data *data = dev->data;

	LOG_DBG("dev_name: %s, trig: chan: %d, type: %d", dev->name,
			trig->chan, trig->type);

	data->nmea_handler = handler;
	if (handler == NULL) {
		return -EINVAL;
	}

	return 0;
}

static void max7q_work_cb(struct k_work *work)
{
	struct max7q_data *data =
		CONTAINER_OF(work, struct max7q_data, trig_work);

	LOG_DBG("NMEA trig work_cb");

	struct sensor_trigger nmea_trigger = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};

	if (likely(data->nmea_handler != NULL)) {
		data->nmea_handler(data->dev, &nmea_trigger);
	}
}

static const struct sensor_driver_api max7q_api = {
	.sample_fetch = max7q_sample_fetch,
	.channel_get = max7q_channel_get,
	.attr_set = max7q_attr_set,
	.attr_get = max7q_attr_get,
	.trigger_set = max7q_trigger_set,
};

static int max7q_init(const struct device *dev)
{
	const struct max7q_config *config = dev->config;
	struct max7q_data *data = dev->data;
	int ret;

	/* configure UART */
	if (!device_is_ready(config->uart)) {
		LOG_ERR("UART not ready");
		return -ENODEV;
	}

	ret = uart_configure(config->uart, &uart_config);
	if (ret < 0) {
		LOG_ERR("UART config err: %d", ret);
		return ret;
	}

	uart_irq_callback_user_data_set(config->uart, uart_cb_handler,
					(void *)dev);

#ifdef CONFIG_MAX7Q_POWER_ENABLE
	/* configure GPS enable gpio */
	if (!device_is_ready(config->enable.port)) {
		LOG_ERR("Enable GPIO controller not ready");
		return -ENODEV;
	}

	/* Configure GPS module enable gpio */
	ret = gpio_pin_configure_dt(&config->enable, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Could not configure enable GPIO (%d)", ret);
		return ret;
	}
	LOG_INF("GPS enable gpio configured");
#endif

	data->dev = dev;
	data->trig_work.handler = max7q_work_cb;
	data->trigger_enable = false;
	data->gps_power_enable = false;

	LOG_INF("UART for GPS ready");

	return 0;
}

#define MAX7Q_DEFINE(idx)                                                          \
	static struct max7q_data max7q_data_##idx = {                              \
	};                                                                         \
                                                                                   \
	static const struct max7q_config max7q_config_##idx = {                    \
		.uart = DEVICE_DT_GET(DT_INST_BUS(idx)),                           \
		IF_ENABLED(CONFIG_MAX7Q_POWER_ENABLE,                              \
			   (.enable = GPIO_DT_SPEC_INST_GET_OR(idx, enable_gpios,  \
							      {}), )) /* */        \
	};                                                                         \
                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, max7q_init, NULL, &max7q_data_##idx,            \
			      &max7q_config_##idx, POST_KERNEL,                    \
			      CONFIG_SENSOR_INIT_PRIORITY, &max7q_api);

DT_INST_FOREACH_STATUS_OKAY(MAX7Q_DEFINE)
