/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "config.h"
#include <zephyr/lorawan/lorawan.h>

struct s_config config = {
	.dev_eui = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 },
	.app_eui = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 },
	.app_key = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		     0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F },
	.lora_mode = LORAWAN_ACT_OTAA,
	.data_rate = LORAWAN_DR_3,
	.lora_class = LORAWAN_CLASS_A,
	.confirmed_msg = LORAWAN_MSG_UNCONFIRMED,
	.app_port = 2,
	.auto_join = false,
	.send_repeat_time = 0,
	.send_min_delay = 30,
	.max_gps_on_time = 300,
	/* max join attempt in one join session */
	.join_try_count = 5,
	/* max join sessions before give up and reboot. 20 * 5 = 100 join attempts */
	.max_join_retry_sessions_count = 20,
	/* max join session interval in sec */
	.join_try_interval = 300,
	.max_inactive_time_window = 3 * 3600,
	.max_failed_msg = 120,
#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
	.payload_key = {0},
#endif
};

struct s_status lorawan_status = {
	.joined = false,
	.delayed_active = false,
	.gps_pwr_on = false,
	.last_pos_send = 0,
	.last_pos_send_ok = 0,
	.last_accel_event = 0,
	.msgs_sent = 0,
	.msgs_failed = 0,
	.msgs_failed_total = 0,
	.gps_total_on_time = 0,
	.acc_events = 0,
	.join_retry_sessions_count = 0,
};

struct s_config *get_config(void)
{
	return &config;
}

uint8_t config_get_app_port(void)
{
	return config.app_port;
}

bool config_get_auto_join(void)
{
	return config.auto_join;
}

uint8_t *config_get_dev_eui(void)
{
	return config.dev_eui;
}

uint8_t *config_get_app_eui(void)
{
	return config.app_eui;
}

uint8_t *config_get_app_key(void)
{
	return config.app_key;
}

void config_set_auto_join(bool state)
{
	config.auto_join = state;
}

uint8_t config_get_data_rate(void)
{
	return config.data_rate;
}

void config_set_data_rate(uint8_t rate)
{
	config.data_rate = rate;
}

uint8_t config_get_confirmed_msg(void)
{
	return config.confirmed_msg;
}

void config_set_confirmed_msg(uint8_t type)
{
	config.confirmed_msg = type;
}

uint32_t config_get_max_inactive_time_window(void)
{
	return config.max_inactive_time_window;
}

uint32_t config_get_max_failed_msg(void)
{
	return config.max_failed_msg;
}

uint32_t config_get_send_repeat_time(void)
{
	return config.send_repeat_time;
}

void config_set_send_repeat_time(uint32_t time)
{
	config.send_repeat_time = time;
}

uint32_t config_get_join_try_interval(void)
{
	return config.join_try_interval;
}

uint8_t config_get_join_try_count(void)
{
	return config.join_try_count;
}

uint8_t config_get_lora_mode(void)
{
	return config.lora_mode;
}

uint32_t config_get_max_gps_on_time(void)
{
	return config.max_gps_on_time;
}

void config_set_max_gps_on_time(uint32_t time)
{
	config.max_gps_on_time = time;
}

uint32_t config_get_send_min_delay(void)
{
	return config.send_min_delay;
}

void config_set_send_min_delay(uint32_t time)
{
	config.send_min_delay = time;
}

uint16_t config_get_max_join_retry_sessions_count(void)
{
	return config.max_join_retry_sessions_count;
}

#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
uint8_t *config_get_payload_key(void)
{
	return config.payload_key;
}
#endif

bool status_is_joined(void)
{
	return lorawan_status.joined;
}

void status_set_joined(bool state)
{
	lorawan_status.joined = state;
}

bool status_get_gps_pwr_on(void)
{
	return lorawan_status.gps_pwr_on;
}

void status_set_gps_pwr_on(bool state)
{
	lorawan_status.gps_pwr_on = state;
}

uint64_t status_get_gps_total_on_time(void)
{
	return lorawan_status.gps_total_on_time;
}

void status_set_gps_total_on_time(uint64_t time)
{
	lorawan_status.gps_total_on_time = time;
}

bool status_get_delayed_active(void)
{
	return lorawan_status.delayed_active;
}

void status_set_delayed_active(bool state)
{
	lorawan_status.delayed_active = state;
}

uint32_t status_get_acc_events(void)
{
	return lorawan_status.acc_events;
}

void status_inc_acc_events(void)
{
	lorawan_status.acc_events++;
}

time_t status_get_last_accel_event(void)
{
	return lorawan_status.last_accel_event;
}

void status_set_last_accel_event(time_t last_accel_time)
{
	lorawan_status.last_accel_event = last_accel_time;
}

time_t status_get_last_pos_send(void)
{
	return lorawan_status.last_pos_send;
}

void status_set_last_pos_send(time_t last_pos_send)
{
	lorawan_status.last_pos_send = last_pos_send;
}

time_t status_get_last_pos_send_ok(void)
{
	return lorawan_status.last_pos_send_ok;
}

void status_set_last_pos_send_ok(time_t last_pos_send_ok)
{
	lorawan_status.last_pos_send_ok = last_pos_send_ok;
}

uint16_t status_get_join_retry_sessions_count(void)
{
	return lorawan_status.join_retry_sessions_count;
}

void status_set_join_retry_sessions_count(uint16_t count)
{
	lorawan_status.join_retry_sessions_count = count;
}

uint32_t status_get_msgs_sent(void)
{
	return lorawan_status.msgs_sent;
}

void status_set_msgs_sent(uint32_t msgs)
{
	lorawan_status.msgs_sent = msgs;
}

uint32_t status_get_msgs_failed(void)
{
	return lorawan_status.msgs_failed;
}

void status_set_msgs_failed(uint32_t msgs)
{
	lorawan_status.msgs_failed = msgs;
}

uint32_t status_get_msgs_failed_total(void)
{
	return lorawan_status.msgs_failed_total;
}

void status_set_msgs_failed_total(uint32_t msgs)
{
	lorawan_status.msgs_failed_total = msgs;
}
