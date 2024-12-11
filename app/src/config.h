/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

#include <stdio.h>
#include <zephyr/lorawan/lorawan.h>

struct s_lorawan_config
{
	/* OTAA Device EUI MSB */
	char dev_eui[8];
	/* OTAA Application EUI MSB */
	uint8_t app_eui[8];
	/* OTAA Application Key MSB */
	uint8_t app_key[16];
	/* LoRaWAN activation mode */
	uint8_t lora_mode;
	/* Data rate (depnends on Region) */
	uint8_t data_rate;
	/* LoRaWAN class */
	uint8_t lora_class;
	/* Type of messages: confirmed or un-confirmed */
	uint8_t confirmed_msg;
	/* Data port to send data */
	uint8_t app_port;
	/* Flag if node joins automatically after reboot */
	bool auto_join;
	/* Send repeat time in seconds */
	uint32_t send_repeat_time;
	/* Min delay time for sensors in seconds */
	uint32_t send_min_delay;
	/* Max GPS on time in seconds */
	uint32_t max_gps_on_time;
	/* Max attempt to join network */
	uint8_t join_try_count;
	/* Max LoraWAN join sessions retry count before reboot */
	uint16_t max_join_retry_sessions_count;
	/* Max LoraWAN join window interval in seconds */
	uint32_t join_try_interval;
	/* Max time window of no ack'd msg received before re-join in seconds */
	uint32_t max_inactive_time_window;
	/* Number of failed message before re-join */
	uint32_t max_failed_msg;
#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
	/* AES128 key for encrypting the payload. */
	uint8_t payload_key[128/8];
#endif
};

//TODO to remove and replace with methods !!!
extern struct s_lorawan_config lorawan_config;

struct s_mapper_data
{
	uint32_t lat;
	uint32_t lng;
	uint16_t alt;
	uint16_t accuracy;
	uint16_t battery;
	uint8_t fix;
	uint8_t satellites;
} __packed;

struct s_status {
	bool joined;
	bool delayed_active;
	bool gps_pwr_on;
	time_t last_pos_send;
	time_t last_pos_send_ok;
	time_t last_accel_event;
	uint32_t msgs_sent;
	uint32_t msgs_failed;
	uint32_t msgs_failed_total;
	uint64_t gps_total_on_time;
	uint32_t acc_events;
	uint16_t join_retry_sessions_count;
};

extern struct s_status lorawan_status;

uint32_t config_get_max_gps_on_time(void);
uint32_t config_get_send_repeat_time(void);
uint32_t config_get_send_min_delay(void);
bool status_is_joined(void);
void status_set_joined(bool state);
bool status_get_delayed_active(void);
void status_set_delayed_active(bool state);
uint32_t status_get_acc_events(void);
void status_inc_acc_events(void);
time_t status_get_last_accel_event(void);
void status_set_last_accel_event(time_t last_accel_time);
time_t status_get_last_pos_send(void);

#endif /* _APP_CONFIG_H_ */
