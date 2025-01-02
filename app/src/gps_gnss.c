/*
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include "config.h"
#include "gps_gnss.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gps_gnss);

#define GNSS_MODEM DEVICE_DT_GET(DT_ALIAS(gnss))

static const struct device *dev = GNSS_MODEM;
gnss_fix_cb_t gnss_fix_cb = NULL;

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	uint64_t timepulse_ns;
	k_ticks_t timepulse;

	if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
		if (gnss_get_latest_timepulse(dev, &timepulse) == 0) {
			timepulse_ns = k_ticks_to_ns_near64(timepulse);
			LOG_INF("Got a fix @ %lld ns", timepulse_ns);
		} else {
			LOG_INF("Got a fix!");
			if (gnss_fix_cb) {
				gnss_fix_cb(data);
			}
		}
	}
}
GNSS_DATA_CALLBACK_DEFINE(GNSS_MODEM, gnss_data_cb);

void gnss_enable(bool enable)
{
	int ret;

	if (enable) {
		ret = pm_device_runtime_get(dev);
		if (ret) {
			LOG_ERR("%s: PM can't runtime get", dev->name);
		}
	} else {
		ret = pm_device_runtime_put(dev);
		if (ret) {
			LOG_ERR("%s: PM can't runtime put", dev->name);
		}
	}
}

int init_gps_gnss(gnss_fix_cb_t cb)
{
	if (!device_is_ready(dev)) {
		LOG_ERR("%s: device not ready.", dev->name);
		return -ENODEV;
	}

	gnss_fix_cb = cb;

	LOG_INF("%s device is ready.", dev->name);

	return 0;
}
