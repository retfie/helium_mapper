/*
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __HELIUM_MAPPER_GPS_GNSS_H__
#define __HELIUM_MAPPER_GPS_GNSS_H__

#include <zephyr/drivers/gnss.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

enum gps_power_mode_t {
	GPS_DISABLE = 0,
	GPS_ENABLE,
};

enum gps_trig_mode_t {
	GPS_TRIG_DISABLE = 0,
	GPS_TRIG_ENABLE,
};

typedef void (*gnss_fix_cb_t)(const struct gnss_data *data);

int init_gps_gnss(gnss_fix_cb_t cb);
int gnss_enable(bool enable);
int gnss_trigger_set(bool enable);

#endif /* __HELIUM_MAPPER_GPS_GNSS_H__ */
