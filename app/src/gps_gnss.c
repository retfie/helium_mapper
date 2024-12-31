/*
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gnss/gnss_publish.h>
#include <zephyr/device.h>

#include "config.h"
#include "gps_gnss.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gps_gnss);

#define GNSS_MODEM DEVICE_DT_GET(DT_ALIAS(gnss))

static const struct device *dev = GNSS_MODEM;

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	uint64_t timepulse_ns;
	k_ticks_t timepulse;

	if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
		if (gnss_get_latest_timepulse(dev, &timepulse) == 0) {
			timepulse_ns = k_ticks_to_ns_near64(timepulse);
			printf("Got a fix @ %lld ns\n", timepulse_ns);
		} else {
			printf("Got a fix!\n");
		}
	}
}
GNSS_DATA_CALLBACK_DEFINE(GNSS_MODEM, gnss_data_cb);

int init_gps_gnss(void)
{
	int ret;

	if (!device_is_ready(dev)) {
		LOG_ERR("%s: device not ready.", dev->name);
		return -ENODEV;
	}

#if CONFIG_PM_DEVICE
#if 1
	// "gnss-nmea-generic" init as pm_device_init_suspended(dev)
	ret = pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);
	if (ret) {
		LOG_ERR("pm_device_action_run failed");
	}
#else
	// "u-blox,nmea-max7q"; init as pm_device_runtime_enable(dev)
	// and it's already enabled, but could be controlled via:
	//ret = pm_device_runtime_get(dev);	// runtime enable
	//ret = pm_device_runtime_put(dev);	// runtime disable
	//if (ret) {
	//	LOG_ERR("pm_device_action_run failed");
	//}
#endif
#endif

	LOG_INF("%s device is ready.", dev->name);

	return 0;
}
