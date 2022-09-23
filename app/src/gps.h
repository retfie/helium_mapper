/*
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __HELIUM_MAPPER_GPS_H__
#define __HELIUM_MAPPER_GPS_H__

#include <zephyr/drivers/sensor.h>

#include "lorawan_config.h"

enum gps_power_mode_t {
	GPS_DISABLE = 0,
	GPS_ENABLE,
};

enum gps_trig_mode_t {
	GPS_TRIG_DISABLE = 0,
	GPS_TRIG_ENABLE,
};

int init_gps(void);

int gps_enable(int enable);

int gps_set_trigger_handler(sensor_trigger_handler_t handler);

void nmea_trigger_enable(int enable);

void read_location(struct s_mapper_data *mapper_data);

#endif /* __HELIUM_MAPPER_GPS_H__ */
