/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "config.h"

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

uint32_t config_get_max_gps_on_time(void)
{
	return lorawan_config.max_gps_on_time;
}

uint32_t config_get_send_repeat_time(void)
{
	return lorawan_config.send_repeat_time;
}

uint32_t config_get_send_min_delay(void)
{
	return lorawan_config.send_min_delay;
}

bool status_is_joined(void)
{
	return lorawan_status.joined;
}

void status_set_joined(bool state)
{
	lorawan_status.joined = state;
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

