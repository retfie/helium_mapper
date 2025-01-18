/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/lorawan/lorawan.h>

#include "config.h"
#include "leds.h"
#include "lorawan_app.h"
#if IS_ENABLED(CONFIG_BATTERY)
#include "battery.h"
#endif
#if IS_ENABLED(CONFIG_UBLOX_MAX7Q)
#include "gps.h"
#endif
#if IS_ENABLED(CONFIG_GNSS)
#include "gps_gnss.h"
#endif
#if IS_ENABLED(CONFIG_SHELL)
#include "shell.h"
#endif
#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
#include "encryption.h"
#endif

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lorawan_app);

#define LORAWAN_JOIN_BACKOFF_TIME_SEC 15

#define LORA_JOIN_THREAD_STACK_SIZE 2048
#define LORA_JOIN_THREAD_PRIORITY 10
K_KERNEL_STACK_MEMBER(lora_join_thread_stack, LORA_JOIN_THREAD_STACK_SIZE);

struct k_thread thread;
struct k_sem lora_join_sem;
struct k_timer lora_join_timer;

enum lorawan_state_e {
	NOT_JOINED,
	JOINED,
};

static void dl_callback(uint8_t port, uint8_t flags,
			int16_t rssi, int8_t snr,
			uint8_t len, const uint8_t *data)
{
	LOG_INF("Port %d, Pending 0x%x, RSSI %ddB, SNR %ddBm", port, flags, rssi, snr);
	if (data) {
		LOG_HEXDUMP_INF(data, len, "Payload: ");
#if IS_ENABLED(CONFIG_SHELL)
		dl_shell_cmd_exec(len, data);
#endif
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

static const char *lorawan_state_str(enum lorawan_state_e state)
{
	switch(state) {
	case NOT_JOINED:
		return "NOT_JOINED";
	case JOINED:
		return "JOINED";
	}

	return "UNKNOWN";
}

void lorawan_state(enum lorawan_state_e state)
{
	uint32_t join_try_interval_sec = config_get_join_try_interval();

	LOG_INF("LoraWAN state set to: %s", lorawan_state_str(state));

	switch (state) {
	case NOT_JOINED:
		/* Turn green led on to indicate not joined state */
		led_join(LED_ON);

		if (!config_get_auto_join()) {
			LOG_WRN("Join is not enabled");
			break;
		}

		status_set_joined(false);
		LOG_INF("Lora join timer start for %d sec", join_try_interval_sec);
		k_timer_start(&lora_join_timer, K_SECONDS(join_try_interval_sec),
			      K_NO_WAIT);
		k_sem_give(&lora_join_sem);
		break;

	case JOINED:
		/* Turn green led off on join success */
		led_join(LED_OFF);

		status_set_joined(true);
		status_set_join_retry_sessions_count(0);
		LOG_INF("Stop Lora join retry timer");
		k_timer_stop(&lora_join_timer);
		break;

	default:
		LOG_ERR("Unknown LoraWAN state");
		break;
	} /* switch */
}

static void lora_join_timer_handler(struct k_timer *timer)
{
	LOG_INF("LoraWAN join timer handler");

	/* If not joined within 'join_try_interval', try again */
	if (!status_is_joined()) {
		lorawan_state(NOT_JOINED);
	}
}

int join_lora(void) {
	struct lorawan_join_config join_cfg;
	int join_try_count = config_get_join_try_count();
	int retry = join_try_count;
	int ret = 0;

	join_cfg.mode = config_get_lora_mode();
	join_cfg.dev_eui = config_get_dev_eui();
	join_cfg.otaa.join_eui = config_get_app_eui();
	join_cfg.otaa.app_key = config_get_app_key();
	join_cfg.otaa.nwk_key = config_get_app_key();
	join_cfg.otaa.dev_nonce = 0u;

	if (config_get_auto_join()) {
		while (retry--) {
			LOG_INF("Joining network over OTAA. Attempt: %d",
					join_try_count - retry);
			ret = lorawan_join(&join_cfg);
			if (ret == 0) {
				break;
			}
			LOG_ERR("lorawan_join_network failed: %d", ret);
			k_sleep(K_SECONDS(LORAWAN_JOIN_BACKOFF_TIME_SEC));
		}

		if (ret == 0) {
			lorawan_state(JOINED);
		}
	}

	return ret;
}

static void lora_join_thread(void *data) {
	uint16_t retry_count_conf = config_get_max_join_retry_sessions_count();
	uint16_t retry_count;
	int err;

	while (1) {
		retry_count = status_get_join_retry_sessions_count();
		k_sem_take(&lora_join_sem, K_FOREVER);
		err = join_lora();
		if (err) {
			retry_count++;
			status_set_join_retry_sessions_count(retry_count);
		}

		if (retry_count > retry_count_conf) {
			LOG_ERR("Reboot in 30sec");
			k_sleep(K_SECONDS(30));
			sys_reboot(SYS_REBOOT_WARM);
			return; /* won't reach this */
		}
	}
}

int init_lora(void) {
	const struct device *lora_dev;
	int ret;
	uint8_t data_rate = config_get_data_rate();

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
	lorawan_set_datarate(data_rate);

	k_sem_init(&lora_join_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&thread, lora_join_thread_stack,
			K_THREAD_STACK_SIZEOF(lora_join_thread_stack),
			(k_thread_entry_t)lora_join_thread, NULL, NULL, NULL,
			K_PRIO_PREEMPT(LORA_JOIN_THREAD_PRIORITY), 0,
			K_SECONDS(1));

	k_thread_name_set(&thread, "lora_join");

	k_timer_init(&lora_join_timer, lora_join_timer_handler, NULL);

	/* make initial join */
	lorawan_state(NOT_JOINED);

	return 0;
}

void lora_send_msg(void)
{
	int err;
	int64_t last_pos_send_ok_sec;
	int64_t delta_sent_ok_sec;
	uint8_t msg_type = config_get_confirmed_msg();
	uint32_t inactive_time_window_sec = config_get_max_inactive_time_window();
	uint32_t max_failed_msgs = config_get_max_failed_msg();
	uint8_t app_port = config_get_app_port();
	uint32_t msgs_sent = status_get_msgs_sent();
	uint32_t msgs_failed = status_get_msgs_failed();
	uint32_t msgs_failed_total = status_get_msgs_failed_total();
	size_t payload_size = 0;
	uint8_t *payload = NULL;
	struct s_mapper_data mapper_data;

	if (!status_is_joined()) {
		LOG_WRN("Not joined");
		return;
	}

	memset(&mapper_data, 0, sizeof(struct s_mapper_data));

	mapper_data.fix = status_get_gps_fix() ? 1 : 0;

#if IS_ENABLED(CONFIG_BATTERY)
	int batt_mV;
	err = read_battery(&batt_mV);
	if (err == 0) {
		mapper_data.battery = (uint16_t)batt_mV;
	}
#endif

#if IS_ENABLED(CONFIG_UBLOX_MAX7Q)
	read_location(&mapper_data);
#endif
#if IS_ENABLED(CONFIG_GNSS)
	read_location(&mapper_data);
#endif

	LOG_HEXDUMP_DBG(&mapper_data, sizeof(mapper_data),
			"mapper_data_clear");

	/* Send at least one confirmed msg on every 10 to check connectivity */
	if (msg_type == LORAWAN_MSG_UNCONFIRMED &&
			!(msgs_sent % 10)) {
		msg_type = LORAWAN_MSG_CONFIRMED;
	}

	LOG_INF("Lora send -------------->");

#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
	if (should_encrypt_payload()) {
		err = encrypt_payload(&mapper_data, &payload, &payload_size);
		if (err)
			return;
	} else
#endif
	{
		/* No payload key. Send in clear. */
		payload_size = sizeof(struct s_mapper_data);
		payload = (uint8_t *)&mapper_data;
	}

	led_msg(LED_ON);
	err = lorawan_send(app_port, payload, payload_size, msg_type);
	if (err < 0) {
		//TODO: make special LED pattern in this case
		msgs_failed++;
		status_set_msgs_failed(msgs_failed);
		msgs_failed_total++;
		status_set_msgs_failed_total(msgs_failed_total);
		LOG_ERR("lorawan_send failed: %d", err);
	} else {
		msgs_sent++;
		status_set_msgs_sent(msgs_sent);
		msgs_failed = 0;
		status_set_msgs_failed(msgs_failed);
		/* Remember last successfuly send message time */
		status_set_last_pos_send_ok(k_uptime_get());
		LOG_INF("Data sent!");
	}
	led_msg(LED_OFF);

	/* Remember last send time */
	status_set_last_pos_send(k_uptime_get());

	status_set_gps_fix(false);

	last_pos_send_ok_sec = status_get_last_pos_send_ok();
	delta_sent_ok_sec = k_uptime_delta(&last_pos_send_ok_sec) / 1000;
	LOG_INF("delta_sent_ok_sec: %lld", delta_sent_ok_sec);

	if (msgs_failed > max_failed_msgs ||
			delta_sent_ok_sec > inactive_time_window_sec) {
		LOG_ERR("Too many failed msgs: Try to re-join.");
		lorawan_state(NOT_JOINED);
	}
}
