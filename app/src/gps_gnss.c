/*
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/timeutil.h>

#if defined(CONFIG_ARCH_POSIX) && defined(CONFIG_EXTERNAL_LIBC)
#include <time.h>
#else
#include <zephyr/posix/time.h>
#endif

#include "gps_gnss.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gps_gnss);

#define GNSS_MODEM DEVICE_DT_GET(DT_ALIAS(gnss))

static const struct device *dev = GNSS_MODEM;
gnss_fix_cb_t gnss_fix_cb;
bool trigger_enable;
struct gnss_data m_gnss_data;

void set_system_time(const struct gnss_time *utc)
{
	struct timespec tp;
	struct tm tm;
	int ret;

	clock_gettime(CLOCK_REALTIME, &tp);
	gmtime_r(&tp.tv_sec, &tm);

	/* centry_year range is 0 - 99, but tm_year is years after 1900,
	   so for centry_year >= 70 consider it's before 2000, although
	   this will be valid only until 2069 */
	if (utc->century_year >= 70) {
		tm.tm_year = utc->century_year;
	} else {
		tm.tm_year = utc->century_year + 100;
	}
	tm.tm_mon = utc->month - 1;
	tm.tm_mday = utc->month_day;

	tm.tm_hour = utc->hour;
	tm.tm_min = utc->minute;
	tm.tm_sec = utc->millisecond / 1000;

	LOG_DBG("utc: year %3u, month %2u, day %2u, hour %2u, min %2u, ms  %u",
		utc->century_year, utc->month,
		utc->month_day, utc->hour,
		utc->minute, utc->millisecond);

	LOG_DBG(" tm: year %3u, month %2u, day %2u, hour %2u, min %2u, sec %u",
		tm.tm_year, tm.tm_mon,
		tm.tm_mday, tm.tm_hour,
		tm.tm_min, tm.tm_sec);

	/* Note range allows for a leap second */
	if ((tm.tm_sec < 0) || (tm.tm_sec > 60)) {
		LOG_ERR("Invalid second");
		return;
	}

	tp.tv_sec = timeutil_timegm(&tm);
	if (tp.tv_sec == -1) {
		LOG_ERR("Failed to calculate seconds since Epoch");
		return;
	}
	tp.tv_nsec = 0;

	ret = clock_settime(CLOCK_REALTIME, &tp);
	if (ret != 0) {
		LOG_ERR("Could not set date %d", ret);
		return;
	}
}

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	uint64_t timepulse_ns;
	k_ticks_t timepulse;

	if (data->info.fix_status == GNSS_FIX_STATUS_NO_FIX) {
		return;
	}

	if (gnss_get_latest_timepulse(dev, &timepulse) == 0) {
		timepulse_ns = k_ticks_to_ns_near64(timepulse);
		LOG_INF("Got a fix @ %lld ns", timepulse_ns);
		return;
	}

	LOG_INF("Got a fix!");

	if (trigger_enable) {
		set_system_time(&data->utc);
		/* copy location data to local struct */
		memcpy(&m_gnss_data, data, sizeof(const struct gnss_data));
		if (gnss_fix_cb) {
			gnss_fix_cb();
		}
	}
}
GNSS_DATA_CALLBACK_DEFINE(GNSS_MODEM, gnss_data_cb);

int gnss_trigger_set(bool enable)
{
	int ret;

	if (enable) {
		ret = gnss_enable(GPS_ENABLE);
		if (ret) {
			return ret;
		}
	}

	if (trigger_enable == enable)
	{
		LOG_WRN("gnss: trigger is already: %d", enable);
		return -EALREADY;
	}

	trigger_enable = enable;

	return 0;
}

int gnss_enable(bool enable)
{
	int ret = 0;

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

	return ret;
}

int print_location_to_str(char *str, uint16_t strsize)
{
	int ret;
	struct navigation_data *nav_data = &m_gnss_data.nav_data;
	struct gnss_info *info = &m_gnss_data.info;

	const char *fmt = "lat: %s%lli.%09lli, lng: %s%lli.%09lli, "
			  "bearing %u.%03u, speed %u.%03u, alt: %s%i.%03i, "
			  "sat: %d, acc: %u.%03u";
	char *lat_sign = nav_data->latitude < 0 ? "-" : "";
	char *lon_sign = nav_data->longitude < 0 ? "-" : "";
	char *alt_sign = nav_data->altitude < 0 ? "-" : "";

	ret = snprintk(str, strsize, fmt,
		       lat_sign,
		       llabs(nav_data->latitude) / 1000000000,
		       llabs(nav_data->latitude) % 1000000000,
		       lon_sign,
		       llabs(nav_data->longitude) / 1000000000,
		       llabs(nav_data->longitude) % 1000000000,
		       nav_data->bearing / 1000, nav_data->bearing % 1000,
		       nav_data->speed / 1000, nav_data->speed % 1000,
		       alt_sign, abs(nav_data->altitude) / 1000,
		       abs(nav_data->altitude) % 1000,
		       info->satellites_cnt,
		       info->hdop / 1000, info->hdop % 1000);

	return (strsize < ret) ? -ENOMEM : 0;
}

void read_location(struct s_mapper_data *mapper_data)
{
	mapper_data->lat = m_gnss_data.nav_data.latitude / 1000;
	mapper_data->lng = m_gnss_data.nav_data.longitude / 1000;
	mapper_data->satellites = m_gnss_data.info.satellites_cnt;
	mapper_data->alt = m_gnss_data.nav_data.altitude / 1000;
	mapper_data->accuracy = m_gnss_data.info.hdop / 1000;
}

int init_gps_gnss(gnss_fix_cb_t cb)
{
	if (!device_is_ready(dev)) {
		LOG_ERR("%s: device not ready.", dev->name);
		return -ENODEV;
	}

	trigger_enable = false;

	gnss_fix_cb = cb;

	LOG_INF("%s device is ready.", dev->name);

	return 0;
}
