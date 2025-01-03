/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _APP_ACCEL_H_
#define _APP_ACCEL_H_

#include <zephyr/drivers/sensor.h>

static const enum sensor_channel channels[] = {
	SENSOR_CHAN_ACCEL_X,
	SENSOR_CHAN_ACCEL_Y,
	SENSOR_CHAN_ACCEL_Z,
};

struct s_accel_values {
	struct sensor_value val;
	const char *sign;
};

int init_accel(void);
int accel_set_trigger_handler(sensor_trigger_handler_t handler);
int print_accels(void);

#endif // _APP_ACCEL_H_
