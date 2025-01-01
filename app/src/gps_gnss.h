/*
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __HELIUM_MAPPER_GPS_GNSS_H__
#define __HELIUM_MAPPER_GPS_GNSS_H__

#include <zephyr/drivers/gnss.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

typedef void (*gnss_fix_cb_t)(const struct gnss_data *data);

int init_gps_gnss(gnss_fix_cb_t cb);
void gnss_enable(bool enable);

#endif /* __HELIUM_MAPPER_GPS_GNSS_H__ */
