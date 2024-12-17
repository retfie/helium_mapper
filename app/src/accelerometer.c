/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

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

	err = sensor_sample_fetch(dev);
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

	if (IS_ENABLED(CONFIG_LIS2DH_ODR_RUNTIME)) {
		struct sensor_value attr = {
			.val1 = 10,
			.val2 = 0,
		};

		err = sensor_attr_set(dev, chan,
				SENSOR_ATTR_SAMPLING_FREQUENCY,
				&attr);
		if (err != 0) {
			LOG_ERR("Failed to set odr: %d", err);
			return err;
		}
		LOG_INF("Sampling at %u Hz", attr.val1);

		/* set slope threshold to 30 dps */
		sensor_degrees_to_rad(30, &attr); /* convert to rad/s */

		if (sensor_attr_set(dev, chan,
					SENSOR_ATTR_SLOPE_TH, &attr) < 0) {
			LOG_ERR("Accel: cannot set slope threshold.\n");
			return err;
		}

		/* set slope duration to 4 samples */
		attr.val1 = 4;
		attr.val2 = 0;

		if (sensor_attr_set(dev, chan,
					SENSOR_ATTR_SLOPE_DUR, &attr) < 0) {
			LOG_ERR("Accel: cannot set slope duration.\n");
			return err;
		}

#if CONFIG_LIS2DH_ACCEL_HP_FILTERS
		/* Set High Pass filter for int 1 */
		attr.val1 = 1U;
		attr.val2 = 0;
		if (sensor_attr_set(dev, chan,
				    SENSOR_ATTR_CONFIGURATION, &attr) < 0) {
			LOG_ERR("Accel: cannot set high pass filter for int 1.");
			return err;
		}
#endif
	}

	trig.type = SENSOR_TRIG_DELTA;
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
