/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>

#include "config.h"
#include "accelerometer.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(helium_mapper_accel);

const struct device *dev;

int print_accels(void)
{
	int err;
	struct s_accel_values accel[3] = {
		{.sign = ""},
		{.sign = ""},
		{.sign = ""}
	};

	err = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	if (err < 0) {
		LOG_ERR("%s: sensor_sample_fetch() failed: %d", dev->name, err);
		return err;
	}

	for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
		err = sensor_channel_get(dev, channels[i], &accel[i].val);
		if (err < 0) {
			LOG_ERR("%s: sensor_channel_get(%c) failed: %d\n",
					dev->name, 'X' + i, err);
			return err;
		}
		if ((accel[i].val.val1 < 0) || (accel[i].val.val2 < 0)) {
			accel[i].sign = "-";
			accel[i].val.val1 = abs(accel[i].val.val1);
			accel[i].val.val2 = abs(accel[i].val.val2);
		}
	}

	LOG_INF("%s: %d, %s%d.%06d, %s%d.%06d, %s%d.%06d (m/s^2)",
		dev->name, status_get_acc_events(),
		accel[0].sign, accel[0].val.val1, accel[0].val.val2,
		accel[1].sign, accel[1].val.val1, accel[1].val.val2,
		accel[2].sign, accel[2].val.val1, accel[2].val.val2);

	return 0;
}

int accel_set_trigger_handler(sensor_trigger_handler_t handler)
{
	int err = 0;
	struct sensor_trigger trig;
	enum sensor_channel chan = SENSOR_CHAN_ACCEL_XYZ;

	struct sensor_value attr = {
		.val1 = 0,
		.val2 = 0,
	};

	// Acceleration in m/s^2
	attr.val1 = 20;
	err = sensor_attr_set(dev, chan,
			SENSOR_ATTR_FULL_SCALE,
			&attr);
	if (err != 0) {
		LOG_ERR("Failed to set scale: %d", err);
		return err;
	}
	LOG_INF("Acceleration in %d.%d m/s^2", attr.val1, attr.val2);

	// Sampling frequency in Hz
	attr.val1 = 416;
	err = sensor_attr_set(dev, chan,
			SENSOR_ATTR_SAMPLING_FREQUENCY,
			&attr);
	if (err != 0) {
		LOG_ERR("Failed to set odr: %d", err);
		return err;
	}
	LOG_INF("Sampling at %u Hz", attr.val1);

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = chan;

	err = sensor_trigger_set(dev, &trig, handler);
	if (err != 0) {
		LOG_ERR("Failed to set trigger: %d", err);
	}

	return err;
}

int init_accel(void)
{
	dev = DEVICE_DT_GET(DT_ALIAS(accel0));
	if (!device_is_ready(dev)) {
		LOG_ERR("%s: device not ready.", dev->name);
		return -ENODEV;
	}

	print_accels();

	return 0;
}
