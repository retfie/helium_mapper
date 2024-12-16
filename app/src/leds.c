/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "leds.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(helium_mapper_led);

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);
static struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(LED_BLUE_NODE, gpios);

int init_leds(void)
{
	int err = 0;

	if (!led_green.port) {
		LOG_INF("Green LED not available");
	} else if (!device_is_ready(led_green.port)) {
		LOG_ERR("Green LED device not ready");
		led_green.port = NULL;
		err = -ENODEV;
	} else {
		/* Init green led as output and turn it on boot */
		err = gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
		if (err) {
			LOG_ERR("failed to configure Green LED gpio: %d", err);
			led_green.port = NULL;
		}
	}

	if (!led_blue.port) {
		LOG_INF("Blue LED not available");
	} else if (!device_is_ready(led_blue.port)) {
		LOG_ERR("Blue LED device not ready");
		led_blue.port = NULL;
		err = -ENODEV;
	} else {
		/* Init blue led as output and turn it on boot */
		err = gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_ACTIVE);
		if (err) {
			LOG_ERR("failed to configure Blue LED gpio: %d", err);
			led_blue.port = NULL;
		}
	}

	return err;
}

void led_enable(const struct gpio_dt_spec *led, int enable) {
	if (led->port) {
		gpio_pin_set_dt(led, enable);
	}
}

void led_join(int value)
{
	led_enable(&led_green, value);
}

void led_msg(int value)
{
	led_enable(&led_blue, value);
}

void led_error(int value)
{
	led_enable(&led_blue, value);
}
