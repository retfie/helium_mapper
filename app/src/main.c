/*
 * Helium mapper sample application
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/printk.h>
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
#include <zephyr/usb/usb_device.h>
#endif
#include <zephyr/drivers/uart.h>

#include "config.h"
#include "lorawan_app.h"
#if IS_ENABLED(CONFIG_BATTERY)
#include "battery.h"
#endif
#include "nvm.h"
#include "gps.h"
#if IS_ENABLED(CONFIG_SHELL)
#include "shell.h"
#endif
#if IS_ENABLED(CONFIG_BT)
#include "ble.h"
#endif

#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
#include <tinycrypt/constants.h>
#include <tinycrypt/cbc_mode.h>
#include <tinycrypt/ctr_prng.h>
#include <zephyr/drivers/entropy.h>

#define TC_ALIGN_UP(N,PAGE)	(((N) + (PAGE) - 1) & ~((PAGE) - 1))
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

#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
#define TC_AES_IV_NBYTES 16
static uint8_t payload_clearbuf[TC_ALIGN_UP(sizeof(mapper_data),TC_AES_BLOCK_SIZE)];
static uint8_t payload_encbuf[sizeof(payload_clearbuf) + TC_AES_BLOCK_SIZE];
static uint8_t enc_aes_iv[TC_AES_IV_NBYTES];	/* Temporary buffer for AES IV. */
#endif

struct s_helium_mapper_ctx {
	const struct device *lora_dev;
	const struct device *accel_dev;
	struct k_timer send_timer;
	struct k_timer delayed_timer;
	struct k_timer gps_off_timer;
	bool gps_fix;
#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
	TCCtrPrng_t prng;
	const struct device *entropy_dev;
#endif
};

struct s_helium_mapper_ctx g_ctx = {
	.gps_fix = false,
};

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
		dev->name, status_get_acc_events(),
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

#if IS_ENABLED(CONFIG_SENSOR)
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

#if CONFIG_LIS2DH_ACCEL_HP_FILTERS
		/* Set High Pass filter for int 1 */
		attr.val1 = 1U;
		attr.val2 = 0;
		if (sensor_attr_set(accel_dev, chan,
					SENSOR_ATTR_CONFIGURATION, &attr) < 0) {
			LOG_ERR("Accel: cannot set high pass filter for int 1.");
			return err;
		}
#endif
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

#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
static bool should_encrypt_payload()
{
	return lorawan_config.payload_key[0] != 0
		|| memcmp(&lorawan_config.payload_key[0],
			  &lorawan_config.payload_key[1],
			  sizeof(lorawan_config.payload_key)-1);
}

static int encrypt_payload(struct s_helium_mapper_ctx *ctx)
{
	int err;
	struct tc_aes_key_sched_struct tc_sched;

	/* Sanity check. */
	assert (sizeof(lorawan_config.payload_key) == TC_AES_KEY_SIZE);

	memset(&payload_clearbuf, 0, sizeof(payload_clearbuf));
	memset(&payload_encbuf, 0, sizeof(payload_encbuf));

	memcpy(&payload_clearbuf, &mapper_data, sizeof(mapper_data));

	(void)tc_aes128_set_encrypt_key(&tc_sched, lorawan_config.payload_key);

	/* Use HWRNG to seed the PRNG.  Reuse the payload_encbuf buffer. */
	err = entropy_get_entropy(ctx->entropy_dev,
				  &payload_encbuf[0],
				  sizeof(payload_encbuf));
	if (err) {
		LOG_ERR("ENTROPY: failed to obtain entropy.");
		return -EIO;
	}
	err = tc_ctr_prng_reseed(&ctx->prng, &payload_encbuf[0],
				 sizeof(payload_encbuf), 0, 0);
	if (err != TC_CRYPTO_SUCCESS) {
		LOG_ERR("PRNG failed");
		return -EIO;
	}

	/* fill-in IV from PRNG. */
	err = tc_ctr_prng_generate(&ctx->prng, NULL, 0,
				   &enc_aes_iv[0], sizeof(enc_aes_iv));
	if (err != TC_CRYPTO_SUCCESS) {
		LOG_ERR("PRNG failed: %d", err);
		return -EIO;
	}
	memset(&payload_encbuf, 0, sizeof(payload_encbuf));
	err = tc_cbc_mode_encrypt(payload_encbuf,
				  sizeof(payload_encbuf),
				  &payload_clearbuf[0],
				  sizeof(payload_clearbuf),
				  &enc_aes_iv[0],
				  &tc_sched);
	if (err != TC_CRYPTO_SUCCESS) {
		LOG_ERR("payload encrypt failed: %d", err);
		return -EIO;
	}
	return 0;
}
#endif

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
		print_accels(ctx->accel_dev);
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
		if (!ctx->gps_fix) {
			lora_send_msg();
		}
#endif
		break;

	case EV_GPS_FIX:
		LOG_INF("Event GPS_FIX");
		ctx->gps_fix = true;
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
	struct app_evt_t *ev;
	int ret;

	LOG_INF("Main start");

	ret = init_leds();
	if (ret) {
		return 1;
	}

#if IS_ENABLED(CONFIG_SETTINGS)
	ret = load_config();
	if (ret) {
		goto fail;
	}
#endif

	init_timers(ctx);

#if IS_ENABLED(CONFIG_SENSOR)
	ret = init_accel(ctx);
	if (ret) {
		goto fail;
	}
#endif

#if IS_ENABLED(CONFIG_UBLOX_MAX7Q)
	ret = init_gps();
	if (ret) {
		LOG_ERR("init_gps failed");
		goto fail;
	}

	ret = gps_set_trigger_handler(gps_trigger_handler);
	if (ret) {
		LOG_ERR("gps_set_trigger_handler  failed");
		goto fail;
	}
#endif

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
	ret = init_usb_console();
	if (ret) {
		goto fail;
	}
#endif

#if IS_ENABLED(CONFIG_SHELL)
	ret = init_shell();
	if (ret) {
		goto fail;
	}

	shell_register_cb(shell_cb, ctx);
#endif

#if IS_ENABLED(CONFIG_BT)
	ret = init_ble();
	if (ret) {
		goto fail;
	}
#endif

#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
	uint8_t entropy[128];

	ctx->entropy_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));
	if (!device_is_ready(ctx->entropy_dev)) {
		LOG_ERR("ENTROPY: random device not ready!");
		return 1;
	}
	ret = entropy_get_entropy(ctx->entropy_dev, &entropy[0], sizeof(entropy));
	if (ret) {
		LOG_ERR("ENTROPY: failed to obtain entropy.");
		return 1;
	}

	ret = tc_ctr_prng_init(&ctx->prng, &entropy[0], sizeof(entropy), 0, 0U);
	if (ret != TC_CRYPTO_SUCCESS) {
		LOG_ERR("PRNG init error.");
		return 1;
	}
#endif

#if IS_ENABLED(CONFIG_LORA)
	ret = init_lora();
	if (ret) {
		LOG_ERR("Rebooting in 30 sec.");
		k_sleep(K_SECONDS(30));
		sys_reboot(SYS_REBOOT_WARM);
		goto fail;
	}
#endif

	while (true) {
		LOG_INF("Waiting for events...");

		k_sem_take(&evt_sem, K_FOREVER);

		while ((ev = app_evt_get()) != NULL) {
			app_evt_handler(ev, ctx);
			app_evt_free(ev);
		}
	}

fail:
	while (true) {
		if (led_blue.port) {
			gpio_pin_set_dt(&led_blue, 0);
			k_sleep(K_MSEC(250));
			gpio_pin_set_dt(&led_blue, 1);
		}
		k_sleep(K_SECONDS(1));
	}
	return 0;
}
