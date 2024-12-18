/*
 * Helium mapper sample application
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/printk.h>
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
#include <zephyr/usb/usb_device.h>
#endif
#include <zephyr/drivers/uart.h>

#include "config.h"
#include "leds.h"
#include "lorawan_app.h"
#if IS_ENABLED(CONFIG_BATTERY)
#include "battery.h"
#endif
#include "nvm.h"
#if IS_ENABLED(CONFIG_UBLOX_MAX7Q)
#include "gps.h"
#endif
#if IS_ENABLED(CONFIG_SENSOR)
#include "accelerometer.h"
#endif
#if IS_ENABLED(CONFIG_SHELL)
#include "shell.h"
#endif
#if IS_ENABLED(CONFIG_BT)
#include "ble.h"
#endif
#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
#include "encryption.h"
#endif

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(helium_mapper);

struct s_helium_mapper_ctx {
	const struct device *lora_dev;
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
	EV_GPS_FIX,
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

void update_gps_off_timer(struct s_helium_mapper_ctx *ctx) {
	uint32_t timeout = config_get_max_gps_on_time();

	LOG_INF("GPS off timer start for %d sec", timeout);

	k_timer_start(&ctx->gps_off_timer, K_SECONDS(timeout), K_NO_WAIT);
}

void update_send_timer(struct s_helium_mapper_ctx *ctx) {
	uint32_t time = config_get_send_repeat_time();

	if (time) {
		LOG_INF("Send interval timer start for %d sec", time);
		k_timer_start(&ctx->send_timer,
				K_SECONDS(time),
				K_SECONDS(time));
	}
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

	status_set_delayed_active(false);

	ev = app_evt_alloc();
	ev->event_type = EV_NMEA_TRIG_ENABLE;
	app_evt_put(ev);
	k_sem_give(&evt_sem);
}

static void gps_off_timer_handler(struct k_timer *timer)
{
	struct app_evt_t *ev;

	LOG_INF("GPS off timer handler");

#if IS_ENABLED(CONFIG_UBLOX_MAX7Q)
	gps_enable(GPS_DISABLE);
#endif

	ev = app_evt_alloc();
	ev->event_type = EV_NMEA_TRIG_DISABLE;
	app_evt_put(ev);
	k_sem_give(&evt_sem);
}

#if IS_ENABLED(CONFIG_UBLOX_MAX7Q)
static void gps_trigger_handler(const struct device *dev,
		const struct sensor_trigger *trig)
{
	struct app_evt_t *ev;

	LOG_INF("GPS trigger handler");

	/** Disable NMEA trigger after successful location fix */
	nmea_trigger_enable(GPS_TRIG_DISABLE);

	ev = app_evt_alloc();
	ev->event_type = EV_GPS_FIX;
	app_evt_put(ev);
	k_sem_give(&evt_sem);
}
#endif

#if IS_ENABLED(CONFIG_LIS2DH_TRIGGER)
static void accel_trigger_handler(const struct device *dev,
		const struct sensor_trigger *trig)
{
	struct app_evt_t *ev;

	LOG_INF("ACC trigger handler");

	/* bounce very "touchy" accell sensor */
	if ((k_uptime_get_32() - status_get_last_accel_event()) < 5000) {
		return;
	}
	status_set_last_accel_event(k_uptime_get_32());
	status_inc_acc_events();

	ev = app_evt_alloc();
	ev->event_type = EV_ACC;
	app_evt_put(ev);
	k_sem_give(&evt_sem);
}
#endif

void init_timers(struct s_helium_mapper_ctx *ctx)
{
	k_timer_init(&ctx->send_timer, send_timer_handler, NULL);
	k_timer_init(&ctx->delayed_timer, delayed_timer_handler, NULL);
	k_timer_init(&ctx->gps_off_timer, gps_off_timer_handler, NULL);

	update_send_timer(ctx);
}

void send_event(struct s_helium_mapper_ctx *ctx) {
	time_t min_delay = config_get_send_min_delay() * 1000;
	time_t last_pos_send = status_get_last_pos_send();
	struct app_evt_t *ev;

	if (!status_is_joined()) {
		LOG_WRN("Not joined");
		return;
	}

	if (!config_get_send_repeat_time()) {
		LOG_WRN("Periodic send is disabled");
		return;
	}

	if (status_get_delayed_active()) {
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
		status_set_delayed_active(true);
	}
}

#if IS_ENABLED(CONFIG_SHELL)
void shell_cb(enum shell_cmd_event event, void *data) {
	struct s_helium_mapper_ctx *ctx = (struct s_helium_mapper_ctx *)data;

	switch (event) {
	case SHELL_CMD_SEND_TIMER_SET:
		update_send_timer(ctx);
		break;
	case SHELL_CMD_SEND_TIMER_GET:
		time_t time_st_left = k_timer_remaining_get(&ctx->send_timer);
		LOG_INF("Send timer %lld sec left", time_st_left / 1000);
		break;
	default:
		LOG_WRN("Unknown shell cmd event");
		break;
	} /* switch */
}
#endif

void app_evt_handler(struct app_evt_t *ev, struct s_helium_mapper_ctx *ctx)
{
	switch (ev->event_type) {
	case EV_TIMER:
		LOG_INF("Event Timer");
		send_event(ctx);
		break;

	case EV_ACC:
		LOG_INF("Event ACC");
		print_accels();
		send_event(ctx);
		break;

	case EV_NMEA_TRIG_ENABLE:
		LOG_INF("Event NMEA_TRIG_ENABLE");
#if IS_ENABLED(CONFIG_UBLOX_MAX7Q)
		nmea_trigger_enable(GPS_TRIG_ENABLE);
#endif
		update_gps_off_timer(ctx);
		break;

	case EV_NMEA_TRIG_DISABLE:
		LOG_INF("Event NMEA_TRIG_DISABLE");
#if IS_ENABLED(CONFIG_UBLOX_MAX7Q)
		nmea_trigger_enable(GPS_TRIG_DISABLE);
#endif
		/* If we aren't able to get gps fix during the whole
		   GPS ON Interval, send lora message with other telemetry
		   data and old position data if available.
		*/
#if IS_ENABLED(CONFIG_LORA)
		if (!status_get_gps_fix()) {
			lora_send_msg();
		}
#endif
		break;

	case EV_GPS_FIX:
		LOG_INF("Event GPS_FIX");
		status_set_gps_fix(true);
#if IS_ENABLED(CONFIG_LORA)
		lora_send_msg();
#endif
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
		LOG_ERR("USB console init failed");
		return -ENODEV;
	}

	return 0;
}
#endif

int main(void)
{
	struct s_helium_mapper_ctx *ctx = &g_ctx;
	enum init_error_t err_num;
	struct app_evt_t *ev;
	int ret;

	LOG_INF("Main start");

	ret = init_leds();
	if (ret) {
		return ret;
	}

#if IS_ENABLED(CONFIG_SETTINGS)
	ret = load_config();
	if (ret) {
		status_set_boot_status(ERROR_CONFIG);
		goto fail;
	}
#endif

	init_timers(ctx);

#if IS_ENABLED(CONFIG_BT)
	ret = init_ble();
	if (ret) {
		err_num = ERROR_BLE;
		goto fail;
	}
#endif

#if IS_ENABLED(CONFIG_SENSOR)
	ret = init_accel();
	if (ret) {
		status_set_boot_status(ERROR_ACCEL);
		goto fail;
	}
#if IS_ENABLED(CONFIG_LIS2DH_TRIGGER)
	ret = accel_set_trigger_handler(accel_trigger_handler);
	if (ret) {
		LOG_ERR("accel_set_trigger_handler failed");
		status_set_boot_status(ERROR_ACCEL_TRIGGER);
		goto fail;
	}
#endif
#endif

#if IS_ENABLED(CONFIG_UBLOX_MAX7Q)
	ret = init_gps();
	if (ret) {
		LOG_ERR("init_gps failed");
		status_set_boot_status(ERROR_GPS);
		goto fail;
	}

	ret = gps_set_trigger_handler(gps_trigger_handler);
	if (ret) {
		LOG_ERR("gps_set_trigger_handler  failed");
		status_set_boot_status(ERROR_GPS_TRIGGER);
		goto fail;
	}
#endif

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
	ret = init_usb_console();
	if (ret) {
		status_set_boot_status(ERROR_USB);
		goto fail;
	}
#endif

#if IS_ENABLED(CONFIG_SHELL)
	ret = init_shell();
	if (ret) {
		status_set_boot_status(ERROR_SHELL);
		goto fail;
	}

	shell_register_cb(shell_cb, ctx);
#endif

#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
	ret = init_encryption();
	if (ret) {
		status_set_boot_status(ERROR_ENCRYPTION);
		goto fail;
	}
#endif

#if IS_ENABLED(CONFIG_LORA)
	ret = init_lora();
	if (ret) {
		LOG_ERR("Rebooting in 30 sec.");
		k_sleep(K_SECONDS(30));
		sys_reboot(SYS_REBOOT_WARM);
		status_set_boot_status(ERROR_LORA);
		goto fail;
	}
#endif

	status_set_boot_status(BOOT_COMPLETE);
	LOG_INF("%s", init_error_to_string(BOOT_COMPLETE));

	while (true) {
		LOG_INF("Waiting for events...");

		k_sem_take(&evt_sem, K_FOREVER);

		while ((ev = app_evt_get()) != NULL) {
			app_evt_handler(ev, ctx);
			app_evt_free(ev);
		}
	}

fail:
	err_num = status_get_boot_status();
	if (err_num != BOOT_COMPLETE)
	{
		LOG_ERR("App init fail: %s", init_error_to_string(err_num));
	}

	while (true) {
		led_error(LED_OFF);
		k_sleep(K_MSEC(250));
		led_error(LED_ON);
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
