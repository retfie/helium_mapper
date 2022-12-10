/*
*
* SPDX-License-Identifier: Apache-2.0
*/

#ifndef __ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_LOCATION_H__
#define __ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_LOCATION_H__

#include <zephyr/drivers/sensor.h>

/** @brief LOCATION custom channels. */
enum location_channel {
	/** Latitudal position in decimal degrees (0 to +-180) */
	SENSOR_CHAN_NAV_LATITUDE = SENSOR_CHAN_PRIV_START,
	/** Longitudal position in decimal degrees (0 to +-180) */
	SENSOR_CHAN_NAV_LONGITUDE,
	/** Number of tracked satellites */
	SENSOR_CHAN_NAV_SATELLITES,
	/** Altitude */
	SENSOR_CHAN_NAV_ALTITUDE,
	/** Relative accuracy of horizontal position */
	SENSOR_CHAN_NAV_ACCURACY,
};

#endif /* __ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_LOCATION_H__ */
