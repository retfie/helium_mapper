/*
 * Copyright (c) 2018-2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2019-2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "battery.h"

LOG_MODULE_REGISTER(battery, CONFIG_ADC_LOG_LEVEL);

#define VBATT DT_PATH(vbatt)
#define ZEPHYR_USER DT_PATH(zephyr_user)

struct divider_config {
	struct gpio_dt_spec power_gpios;
	/* output_ohm is used as a flag value: if it is nonzero then
	 * the battery is measured through a voltage divider;
	 * otherwise it is assumed to be directly connected to Vdd.
	 */
	uint32_t output_ohm;
	uint32_t full_ohm;
};

struct divider_data {
	const struct device *adc;
	struct adc_dt_spec adc_channel;
	struct adc_channel_cfg adc_cfg;
	struct adc_sequence adc_seq;
	int16_t raw;
};

#if DT_NODE_EXISTS(VBATT) && \
	DT_NODE_HAS_PROP(VBATT, io_channels)

static const struct divider_config divider_config = {
	.power_gpios = GPIO_DT_SPEC_GET_OR(VBATT, power_gpios, {}),
	.output_ohm = DT_PROP(VBATT, output_ohms),
	.full_ohm = DT_PROP(VBATT, full_ohms),
};

static struct divider_data divider_data = {
	.adc_channel = ADC_DT_SPEC_GET_BY_IDX(VBATT, 0),
};

#else
#if DT_NODE_EXISTS(ZEPHYR_USER) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER, io_channels)

static const struct divider_config divider_config;

static struct divider_data divider_data = {
	.adc_channel = ADC_DT_SPEC_GET_BY_IDX(ZEPHYR_USER, 0),
};

#else
#error "No suitable devicetree overlay specified, vbatt or zephyr,user"
#endif	/* zephyr_user */
#endif	/* vbatt */

static int divider_setup(void)
{
	const struct divider_config *cfg = &divider_config;
	const struct gpio_dt_spec *gcp = &cfg->power_gpios;
	struct divider_data *ddp = &divider_data;
	const struct device *adc_dev = ddp->adc_channel.dev;
	struct adc_sequence *asp = &ddp->adc_seq;
	int rc;

	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC device is not ready %s", adc_dev->name);
		return -ENOENT;
	}

	if (gcp->port) {
		if (!device_is_ready(gcp->port)) {
			LOG_ERR("%s: device not ready", gcp->port->name);
			return -ENOENT;
		}
		rc = gpio_pin_configure_dt(gcp, GPIO_OUTPUT_INACTIVE);
		if (rc != 0) {
			LOG_ERR("Failed to control feed %s.%u: %d",
				gcp->port->name, gcp->pin, rc);
			return rc;
		}
	}

	*asp = (struct adc_sequence){
		.buffer = &ddp->raw,
		.buffer_size = sizeof(ddp->raw),
		.oversampling = 4,
		.calibrate = true,
	};

	rc = adc_channel_setup_dt(&ddp->adc_channel);
	if (rc < 0) {
		LOG_ERR("Could not setup channel #%d (%d)",
				ddp->adc_channel.channel_id, rc);
		return rc;
	}

	rc = adc_sequence_init_dt(&ddp->adc_channel, asp);
	if (rc < 0) {
		LOG_ERR("Could not init sequence on channel #%d (%d)",
				ddp->adc_channel.channel_id, rc);
		return rc;
	}

	LOG_DBG("%s, setup channel #%d (%d)", adc_dev->name,
			ddp->adc_channel.channel_id, rc);

	return rc;
}

static bool battery_ok;

static int battery_setup(void)
{
	int rc = divider_setup();

	battery_ok = (rc == 0);
	LOG_DBG("Battery setup: %d %d", rc, battery_ok);
	return rc;
}

SYS_INIT(battery_setup, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int battery_measure_enable(bool enable)
{
	int rc = -ENOENT;

	if (battery_ok) {
		const struct gpio_dt_spec *gcp = &divider_config.power_gpios;

		rc = 0;
		if (gcp->port) {
			rc = gpio_pin_set_dt(gcp, enable);
		}
	}
	return rc;
}

int battery_sample(void)
{
	int rc = -ENOENT;

	if (battery_ok) {
		struct divider_data *ddp = &divider_data;
		const struct device *adc_dev = ddp->adc_channel.dev;
		const struct divider_config *dcp = &divider_config;
		struct adc_sequence *sp = &ddp->adc_seq;
		int32_t val_mv;

		rc = adc_read(adc_dev, sp);
		if (rc < 0) {
			return rc;
		}

		val_mv = ddp->raw;
		sp->calibrate = false;

		rc = adc_raw_to_millivolts_dt(&ddp->adc_channel, &val_mv);
		if (rc < 0) {
			LOG_ERR("value in mV not available (%d)", rc);
			return rc;
		}

		if (dcp->output_ohm != 0) {
			rc = val_mv * (uint64_t)dcp->full_ohm
				/ dcp->output_ohm;
			LOG_DBG("raw (0x%x) %u ~ %u mV => %d mV",
					ddp->raw, ddp->raw, val_mv, rc);
		} else {
			rc = val_mv;
			LOG_DBG("raw (0x%x) %u ~ %u mV",
					ddp->raw, ddp->raw, val_mv);
		}
	}

	return rc;
}

unsigned int battery_level_pptt(unsigned int batt_mV,
				const struct battery_level_point *curve)
{
	const struct battery_level_point *pb = curve;

	if (batt_mV >= pb->lvl_mV) {
		/* Measured voltage above highest point, cap at maximum. */
		return pb->lvl_pptt;
	}
	/* Go down to the last point at or below the measured voltage. */
	while ((pb->lvl_pptt > 0)
	       && (batt_mV < pb->lvl_mV)) {
		++pb;
	}
	if (batt_mV < pb->lvl_mV) {
		/* Below lowest point, cap at minimum */
		return pb->lvl_pptt;
	}

	/* Linear interpolation between below and above points. */
	const struct battery_level_point *pa = pb - 1;

	return pb->lvl_pptt
	       + ((pa->lvl_pptt - pb->lvl_pptt)
		  * (batt_mV - pb->lvl_mV)
		  / (pa->lvl_mV - pb->lvl_mV));
}

int read_battery(int *batt)
{
	int err;
	int batt_mV;

	err = battery_measure_enable(true);
	if (err != 0) {
		LOG_ERR("Failed initialize battery measurement: %d", err);
		return err;
	}
	batt_mV = battery_sample();
	if (batt_mV < 0) {
		LOG_ERR("Failed to read battery voltage: %d", batt_mV);
		return batt_mV;
	}

	unsigned int batt_pptt = battery_level_pptt(batt_mV, levels);
	LOG_DBG("Battery: %d mV; %u pptt", batt_mV, batt_pptt);
	battery_measure_enable(false);

	*batt = batt_mV;

	return 0;
}
