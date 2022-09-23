/*
 * Helium mapper sample application
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/printk.h>
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
#include <zephyr/usb/usb_device.h>
#endif
#include <zephyr/drivers/uart.h>

#include "lorawan_config.h"
#include "battery.h"
#include "nvm.h"
#include "gps.h"
#if IS_ENABLED(CONFIG_SHELL)
#include "shell.h"
#endif
#if IS_ENABLED(CONFIG_BT)
#include "ble.h"
#endif

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(helium_mapper);

#define LED_GREEN_NODE DT_ALIAS(green_led)
#define LED_BLUE_NODE DT_ALIAS(blue_led)
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);
static struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(LED_BLUE_NODE, gpios);

struct s_lorawan_config lorawan_config = {
	.dev_eui = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 },
	.app_eui = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 },
	.app_key = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F },
	.lora_mode = LORAWAN_ACT_OTAA,
	.data_rate = LORAWAN_DR_3,
	.lora_class = LORAWAN_CLASS_A,
	.confirmed_msg = LORAWAN_MSG_UNCONFIRMED,
	.app_port = 2,
	.auto_join = false,
	.send_repeat_time = 0,
	.send_min_delay = 30,
	.max_gps_on_time = 300,
};

struct s_status lorawan_status = {
	.joined = false,
	.gps_pwr_on = false,
	.delayed_active = false,
	.last_pos_send = 0,
	.last_accel_event = 0,
	.acc_events = 0,
};

struct s_mapper_data mapper_data;
char *data_ptr = (char*)&mapper_data;


struct s_helium_mapper_ctx {
	const struct device *lora_dev;
	const struct device *accel_dev;
	struct k_timer send_timer;
	struct k_timer delayed_timer;
	struct k_timer gps_off_timer;
};

struct s_helium_mapper_ctx g_ctx;

/* Event FIFO */

K_FIFO_DEFINE(evt_fifo);

enum evt_t {
	EV_TIMER,
	EV_ACC,
	EV_SEND,
	EV_NMEA_TRIG_ENABLE,
	EV_NMEA_TRIG_DISABLE,
};

struct app_evt_t {
	sys_snode_t node;
	enum evt_t event_type;
};

#define FIFO_ELEM_MIN_SZ        sizeof(struct app_evt_t)
#define FIFO_ELEM_MAX_SZ        sizeof(struct app_evt_t)
#define FIFO_ELEM_COUNT         10
#define FIFO_ELEM_ALIGN         sizeof(unsigned int)

K_HEAP_DEFINE(event_elem_pool, FIFO_ELEM_MAX_SZ * FIFO_ELEM_COUNT + 256);

static inline void app_evt_free(struct app_evt_t *ev)
{
	k_heap_free(&event_elem_pool, ev);
}

static inline void app_evt_put(struct app_evt_t *ev)
{
	k_fifo_put(&evt_fifo, ev);
}

static inline struct app_evt_t *app_evt_get(void)
{
	return k_fifo_get(&evt_fifo, K_NO_WAIT);
}

static inline void app_evt_flush(void)
{
	struct app_evt_t *ev;

	do {
		ev = app_evt_get();
		if (ev) {
			app_evt_free(ev);
		}
	} while (ev != NULL);
}

static inline struct app_evt_t *app_evt_alloc(void)
{
	struct app_evt_t *ev;

	ev = k_heap_alloc(&event_elem_pool,
			  sizeof(struct app_evt_t),
			  K_NO_WAIT);
	if (ev == NULL) {
		LOG_ERR("APP event allocation failed!");
		app_evt_flush();

		ev = k_heap_alloc(&event_elem_pool,
				  sizeof(struct app_evt_t),
				  K_NO_WAIT);
		if (ev == NULL) {
			LOG_ERR("APP event memory corrupted.");
			__ASSERT_NO_MSG(0);
			return NULL;
		}
		return NULL;
	}

	return ev;
}

static K_SEM_DEFINE(evt_sem, 0, 1);	/* starts off "not available" */

void update_gps_off_timer(void) {
	struct s_helium_mapper_ctx *ctx = &g_ctx;
	uint32_t timeout = lorawan_config.max_gps_on_time;

	LOG_INF("GPS off timer start for %d sec", timeout);

	// TODO: restart with full period, even if the old one not expired yet
	k_timer_start(&ctx->gps_off_timer, K_SECONDS(timeout), K_NO_WAIT);
}

static void send_timer_handler(struct k_timer *timer)
{
	struct app_evt_t *ev;

	LOG_INF("Timer handler");

	ev = app_evt_alloc();
	ev->event_type = EV_TIMER;
	app_evt_put(ev);
	k_sem_give(&evt_sem);
}

static void delayed_timer_handler(struct k_timer *timer)
{
	struct app_evt_t *ev;

	LOG_INF("Delayed timer handler");

	lorawan_status.delayed_active = false;

	ev = app_evt_alloc();
	ev->event_type = EV_NMEA_TRIG_ENABLE;
	app_evt_put(ev);
	k_sem_give(&evt_sem);
}

static void gps_off_timer_handler(struct k_timer *timer)
{
	struct app_evt_t *ev;

	LOG_INF("GPS off timer handler");

	gps_enable(GPS_DISABLE);

	ev = app_evt_alloc();
	ev->event_type = EV_NMEA_TRIG_DISABLE;
	app_evt_put(ev);
	k_sem_give(&evt_sem);
}

static void gps_trigger_handler(const struct device *dev,
		const struct sensor_trigger *trig)
{
	struct app_evt_t *ev;

	LOG_INF("GPS trigger handler");

	/** Disable NMEA trigger after successful location fix */
	nmea_trigger_enable(GPS_TRIG_DISABLE);

	ev = app_evt_alloc();
	ev->event_type = EV_SEND;
	app_evt_put(ev);
	k_sem_give(&evt_sem);
}

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

static void dl_callback(uint8_t port, bool data_pending,
			int16_t rssi, int8_t snr,
			uint8_t len, const uint8_t *data)
{
	LOG_INF("Port %d, Pending %d, RSSI %ddB, SNR %ddBm", port, data_pending, rssi, snr);
	if (data) {
		LOG_HEXDUMP_INF(data, len, "Payload: ");
	}
}

struct lorawan_downlink_cb downlink_cb = {
	.port = LW_RECV_PORT_ANY,
	.cb = dl_callback
};

static void lorwan_datarate_changed(enum lorawan_datarate dr)
{
	uint8_t unused, max_size;

	lorawan_get_payload_sizes(&unused, &max_size);
	LOG_INF("New Datarate: DR_%d, Max Payload %d", dr, max_size);
}

static const enum sensor_channel channels[] = {
	SENSOR_CHAN_ACCEL_X,
	SENSOR_CHAN_ACCEL_Y,
	SENSOR_CHAN_ACCEL_Z,
};

struct s_accel_values {
	struct sensor_value val;
	const char *sign;
};

static int print_accels(const struct device *dev)
{
	int err;
	struct s_accel_values accel[3] = {
		{.sign = ""},
		{.sign = ""},
		{.sign = ""}
	};

	err = sensor_sample_fetch(dev);
	if (err < 0) {
		LOG_ERR("%s: sensor_sample_fetch() failed: %d", dev->name, err);
		return err;
	}

	for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
		err = sensor_channel_get(dev, channels[i], &accel[i].val);
		if (err < 0) {
			LOG_ERR("%s: sensor_channel_get(%c) failed: %d\n",
					dev->name, 'X' + i, err);
			return err;
		}
		if ((accel[i].val.val1 < 0) || (accel[i].val.val2 < 0)) {
			accel[i].sign = "-";
			accel[i].val.val1 = abs(accel[i].val.val1);
			accel[i].val.val2 = abs(accel[i].val.val2);
		}
	}

	LOG_INF("%s: %d, %s%d.%06d, %s%d.%06d, %s%d.%06d (m/s^2)",
		dev->name, lorawan_status.acc_events,
		accel[0].sign, accel[0].val.val1, accel[0].val.val2,
		accel[1].sign, accel[1].val.val1, accel[1].val.val2,
		accel[2].sign, accel[2].val.val1, accel[2].val.val2);

	return 0;
}

#ifdef CONFIG_LIS2DH_TRIGGER
static void trigger_handler(const struct device *dev,
		const struct sensor_trigger *trig)
{
	struct app_evt_t *ev;

	LOG_INF("ACC trigger handler");

	/* bounce very "touchy" accell sensor */
	if ((k_uptime_get_32() - lorawan_status.last_accel_event) < 5000) {
		return;
	}
	lorawan_status.last_accel_event = k_uptime_get_32();
	lorawan_status.acc_events++;

	ev = app_evt_alloc();
	ev->event_type = EV_ACC;
	app_evt_put(ev);
	k_sem_give(&evt_sem);
}
#endif

int init_accel(struct s_helium_mapper_ctx *ctx)
{
	const struct device *accel_dev;
	int err = 0;

	accel_dev = DEVICE_DT_GET(DT_ALIAS(accel0));
	if (!device_is_ready(accel_dev)) {
		LOG_ERR("%s: device not ready.", accel_dev->name);
		return -ENODEV;
	}

	ctx->accel_dev = accel_dev;

	print_accels(accel_dev);

#if CONFIG_LIS2DH_TRIGGER
	struct sensor_trigger trig;
	enum sensor_channel chan = SENSOR_CHAN_ACCEL_XYZ;

	if (IS_ENABLED(CONFIG_LIS2DH_ODR_RUNTIME)) {
		struct sensor_value attr = {
			.val1 = 10,
			.val2 = 0,
		};

		err = sensor_attr_set(accel_dev, chan,
				SENSOR_ATTR_SAMPLING_FREQUENCY,
				&attr);
		if (err != 0) {
			LOG_ERR("Failed to set odr: %d", err);
			return err;
		}
		LOG_INF("Sampling at %u Hz", attr.val1);

		/* set slope threshold to 30 dps */
		sensor_degrees_to_rad(30, &attr); /* convert to rad/s */

		if (sensor_attr_set(accel_dev, chan,
					SENSOR_ATTR_SLOPE_TH, &attr) < 0) {
			LOG_ERR("Accel: cannot set slope threshold.\n");
			return err;
		}

		/* set slope duration to 4 samples */
		attr.val1 = 4;
		attr.val2 = 0;

		if (sensor_attr_set(accel_dev, chan,
					SENSOR_ATTR_SLOPE_DUR, &attr) < 0) {
			LOG_ERR("Accel: cannot set slope duration.\n");
			return err;
		}
	}

	trig.type = SENSOR_TRIG_DELTA;
	trig.chan = chan;

	err = sensor_trigger_set(accel_dev, &trig, trigger_handler);
	if (err != 0) {
		LOG_ERR("Failed to set trigger: %d", err);
	}
#endif
	return err;
}

int init_lora(void) {
	const struct device *lora_dev;
	struct lorawan_join_config join_cfg;
	int ret;

	lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
	if (!device_is_ready(lora_dev)) {
		LOG_ERR("%s: device not ready.", lora_dev->name);
		return -ENODEV;
	}

	ret = lorawan_start();
	if (ret < 0) {
		LOG_ERR("lorawan_start failed: %d", ret);
		return ret;
	}

	lorawan_register_downlink_callback(&downlink_cb);
	lorawan_register_dr_changed_callback(lorwan_datarate_changed);
	lorawan_set_datarate(lorawan_config.data_rate);

	join_cfg.mode = lorawan_config.lora_mode;
	join_cfg.dev_eui = lorawan_config.dev_eui;
	join_cfg.otaa.join_eui = lorawan_config.app_eui;
	join_cfg.otaa.app_key = lorawan_config.app_key;
	join_cfg.otaa.nwk_key = lorawan_config.app_key;

	if (lorawan_config.auto_join) {
		LOG_INF("Joining network over OTAA");
		ret = lorawan_join(&join_cfg);
		if (ret < 0) {
			LOG_ERR("lorawan_join_network failed: %d", ret);
			return ret;
		}
		lorawan_status.joined = true;

		/* Turn green led off on join success */
		led_enable(&led_green, 1);
	}

	return 0;
}

void init_timers(struct s_helium_mapper_ctx *ctx)
{
	k_timer_init(&ctx->send_timer, send_timer_handler, NULL);
	k_timer_init(&ctx->delayed_timer, delayed_timer_handler, NULL);
	k_timer_init(&ctx->gps_off_timer, gps_off_timer_handler, NULL);

	if (lorawan_config.send_repeat_time) {
		k_timer_start(&ctx->send_timer, K_SECONDS(lorawan_config.send_repeat_time),
				K_SECONDS(lorawan_config.send_repeat_time));
	}
}

void send_event(struct s_helium_mapper_ctx *ctx) {
	time_t min_delay = lorawan_config.send_min_delay * 1000;
	time_t last_pos_send = lorawan_status.last_pos_send;
	struct app_evt_t *ev;

	if (!lorawan_status.joined) {
		LOG_WRN("Not joined");
		return;
	}

	if (!lorawan_config.send_repeat_time) {
		LOG_WRN("Periodic send is disabled");
		return;
	}

	if (lorawan_status.delayed_active) {
		time_t time_left = k_timer_remaining_get(&ctx->delayed_timer);
		LOG_INF("Delayed timer already active, %lld sec left",
				time_left / 1000);
		return;
	}

	if ((k_uptime_get_32() - last_pos_send) > min_delay) {
		/* Enable NMEA trigger and wait for location fix */
		ev = app_evt_alloc();
		ev->event_type = EV_NMEA_TRIG_ENABLE;
		app_evt_put(ev);
		k_sem_give(&evt_sem);
	} else {
		time_t now_ms = k_uptime_get_32();
		time_t wait_time =
			abs(min_delay - (now_ms - last_pos_send) >= 0)
			? (min_delay - (now_ms - last_pos_send)) : min_delay;

		LOG_INF("Delayed timer start for %lld sec", wait_time / 1000);
		k_timer_start(&ctx->delayed_timer, K_MSEC(wait_time), K_NO_WAIT);
		lorawan_status.delayed_active = true;
	}
}

void lora_send_msg(struct s_helium_mapper_ctx *ctx)
{
	int batt_mV;
	int err;

	memset(data_ptr, 0, sizeof(struct s_mapper_data));

	err = read_battery(&batt_mV);
	if (err == 0) {
		mapper_data.battery = (uint16_t)batt_mV;
	}

	read_location(&mapper_data);

	LOG_HEXDUMP_DBG(data_ptr, sizeof(struct s_mapper_data),
			"mapper_data");

	LOG_INF("Lora send -------------->");

	led_enable(&led_blue, 0);
	err = lorawan_send(lorawan_config.app_port,
			data_ptr, sizeof(struct s_mapper_data),
			lorawan_config.confirmed_msg);
	if (err < 0) {
		//TODO: make special LED pattern in this case
		lorawan_status.msgs_failed++;
		LOG_ERR("lorawan_send failed: %d", err);
	} else {
		lorawan_status.msgs_sent++;
		LOG_INF("Data sent!");
	}
	led_enable(&led_blue, 1);

	/* Remember last send time */
	lorawan_status.last_pos_send = k_uptime_get();
}

void app_evt_handler(struct app_evt_t *ev, struct s_helium_mapper_ctx *ctx)
{
	switch (ev->event_type) {
	case EV_TIMER:
		LOG_INF("Event Timer");
		send_event(ctx);
		break;

	case EV_ACC:
		LOG_INF("Event ACC");
		print_accels(ctx->accel_dev);
		send_event(ctx);
		break;

	case EV_NMEA_TRIG_ENABLE:
		LOG_INF("Event NMEA_TRIG_ENABLE");
		nmea_trigger_enable(GPS_TRIG_ENABLE);
		update_gps_off_timer();
		break;

	case EV_NMEA_TRIG_DISABLE:
		LOG_INF("Event NMEA_TRIG_DISABLE");
		nmea_trigger_enable(GPS_TRIG_DISABLE);
		break;

	case EV_SEND:
		LOG_INF("Event SEND");
		lora_send_msg(ctx);
		break;

	default:
		LOG_ERR("Unknown event");
		break;

	} /* switch */
}

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
int init_usb_console(void)
{
	if (usb_enable(NULL)) {
		return -ENODEV;
	}

	return 0;
}
#endif

void main(void)
{
	struct s_helium_mapper_ctx *ctx = &g_ctx;
	struct app_evt_t *ev;
	int ret;

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
	ret = init_usb_console();
	if (ret) {
		return;
	}
#endif

	ret = init_leds();
	if (ret) {
		return;
	}

	ret = load_config();
	if (ret) {
		return;
	}

#if IS_ENABLED(CONFIG_SHELL)
	ret = init_shell();
	if (ret) {
		return;
	}
#endif

#if IS_ENABLED(CONFIG_BT)
	ret = init_ble();
	if (ret) {
		return;
	}
#endif

	ret = init_accel(ctx);
	if (ret) {
		return;
	}

	ret = init_gps();
	if (ret) {
		return;
	}

	ret = gps_set_trigger_handler(gps_trigger_handler);
	if (ret) {
		return;
	}

	ret = init_lora();
	if (ret) {
		LOG_ERR("Rebooting in 10 sec.");
		k_sleep(K_SECONDS(10));
		sys_reboot(SYS_REBOOT_WARM);
		return;
	}

	init_timers(ctx);

	while (true) {
		LOG_INF("Waiting for events...");

		k_sem_take(&evt_sem, K_FOREVER);

		while ((ev = app_evt_get()) != NULL) {
			app_evt_handler(ev, ctx);
			app_evt_free(ev);
		}
	}
}
