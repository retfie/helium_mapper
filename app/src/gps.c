/*
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

#include <app/drivers/sensor/location.h>

#include "config.h"
#include "gps.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(helium_mapper_gps);

static const struct device *dev;
int64_t gps_on_time = 0;

void read_location(struct s_mapper_data *mapper_data)
{
	struct sensor_value lat, lng, alt, acc, sat;
	double tmp;
	int err;

	err = sensor_channel_get(dev, SENSOR_CHAN_NAV_LATITUDE, &lat);
	if (err < 0) {
		LOG_WRN("Unable to get latitude");
		return;
	}
	tmp = sensor_value_to_double(&lat);
	mapper_data->lat = (uint32_t)(tmp * 100000);

	err = sensor_channel_get(dev, SENSOR_CHAN_NAV_LONGITUDE, &lng);
	if (err < 0) {
		LOG_WRN("Unable to get longitude");
		return;
	}
	tmp = sensor_value_to_double(&lng);
	mapper_data->lng = (uint32_t)(tmp * 100000);

	err = sensor_channel_get(dev, SENSOR_CHAN_NAV_SATELLITES, &sat);
	if (err < 0) {
		LOG_WRN("Unable to get satellites");
		return;
	}
	mapper_data->satellites = sat.val1;

	err = sensor_channel_get(dev, SENSOR_CHAN_NAV_ALTITUDE, &alt);
	if (err < 0) {
		LOG_WRN("Unable to get altitude");
		return;
	}
	tmp = sensor_value_to_double(&alt);
	mapper_data->alt = (uint16_t)(tmp);

	err = sensor_channel_get(dev, SENSOR_CHAN_NAV_ACCURACY, &acc);
	if (err < 0) {
		LOG_WRN("Unable to get accuracy");
		return;
	}
	tmp = sensor_value_to_double(&acc);
	mapper_data->accuracy = (uint16_t)(tmp * 100);

	LOG_DBG("lat: %d.%06d, lng: %d.%06d, sat: %d, alt: %d.%06d, acc: %d.%06d",
			lat.val1, lat.val2,
			lng.val1, lng.val2,
			sat.val1,
			alt.val1, alt.val2,
			acc.val1, acc.val2);
}

int gps_enable(int enable)
{
	struct sensor_value attr = { 0, 0 };
	uint64_t gps_total_on_time = status_get_gps_total_on_time();
	enum sensor_channel chan;
	int64_t delta;
	int err;

	chan = SENSOR_CHAN_ALL;
	err = sensor_attr_get(dev, chan, SENSOR_ATTR_CONFIGURATION, &attr);
	if (err == -ENOTSUP) {
		LOG_ERR("%s: can't set gps power attr for %s, err: %d",
				dev->name, enable ? "enable" : "disable", err);
		return err;
	}

	if (err == enable) {
		return 0;
	}

	/* Save GPS ON time */
	if (enable == GPS_ENABLE) {
		gps_on_time = k_uptime_get();
		status_set_gps_pwr_on(true);
	}
	if (enable == GPS_DISABLE) {
		status_set_gps_pwr_on(false);
		delta = k_uptime_delta(&gps_on_time);
		gps_total_on_time += (delta / 1000);
		status_set_gps_total_on_time(gps_total_on_time);
		LOG_INF("GPS was ON for %lld sec, total: %lld sec",
			delta / 1000, gps_total_on_time);
	}

	attr.val1 = enable;
	attr.val2 = 0;
	chan = SENSOR_CHAN_ALL;
	err = sensor_attr_set(dev, chan, SENSOR_ATTR_CONFIGURATION, &attr);
	if (err == -EALREADY) {
		LOG_WRN("GPS already %s", enable ? "ON" : "OFF");
		return 0;
	}
	if (err < 0) {
		LOG_ERR("%s: can't set gps power attr for %s, err: %d",
				dev->name, enable ? "enable" : "disable", err);
		return err;
	}

	return 0;
}

void nmea_trigger_enable(int enable)
{
	struct sensor_value attr = { 0, 0 };
	enum sensor_channel chan;
	int err;

	if (enable) {
		err = gps_enable(GPS_ENABLE);
		if (err) {
			return;
		}
	}

	/* First check state of the trigger and compare with requested state */
	chan = SENSOR_CHAN_ALL;
	err = sensor_attr_get(dev, chan, SENSOR_ATTR_ALERT, &attr);
	if (err == -ENOTSUP) {
		LOG_ERR("%s: can't get nmea trig attr for %s, err: %d",
				dev->name, enable ? "set" : "clear", err);
		return;
	}

	if (err == enable) {
		return;
	}

	/* Set new state of NMEA trigger and wait for location fix if it's enabled */
	attr.val1 = enable;
	attr.val2 = 0;
	chan = SENSOR_CHAN_ALL;
	err = sensor_attr_set(dev, chan, SENSOR_ATTR_ALERT, &attr);
	if (err == -EALREADY) {
		LOG_WRN("nmea trig already %s", enable ? "set" : "clear");
		return;
	}
	if (err < 0) {
		LOG_ERR("%s: can't set nmea trig attr for %s, err: %d",
				dev->name, enable ? "set" : "clear", err);
		return;
	}

	return;
}

int gps_set_trigger_handler(sensor_trigger_handler_t handler)
{
	struct sensor_trigger trig;
	int err;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ALL;

	err = sensor_trigger_set(dev, &trig, handler);
	if (err != 0) {
		LOG_ERR("Failed to set trigger: %d", err);
		return err;
	}

	LOG_INF("gps trigger handler set");

	return 0;
}

int init_gps(void)
{
	dev = DEVICE_DT_GET(DT_ALIAS(gps0));
	if (!device_is_ready(dev)) {
		LOG_ERR("%s: device not ready.", dev->name);
		return -ENODEV;
	}

	LOG_INF("%s device is ready.", dev->name);

	return 0;
}
