/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

#include <stdio.h>
#include <stdbool.h>
#include <zephyr/sys/util.h>

#define PAYLOAD_KEY_SIZE 128/8

struct s_config
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
	uint8_t payload_key[PAYLOAD_KEY_SIZE];
#endif
};

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

enum init_error_t {
	BOOT_COMPLETE,
	ERROR_CONFIG,
	ERROR_BLE,
	ERROR_ACCEL,
	ERROR_ACCEL_TRIGGER,
	ERROR_GPS,
	ERROR_GPS_TRIGGER,
	ERROR_USB,
	ERROR_SHELL,
	ERROR_ENCRYPTION,
	ERROR_LORA,
};

struct s_status {
	enum init_error_t boot_status;
	bool joined;
	bool delayed_active;
	bool gps_pwr_on;
	bool gps_fix;
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

struct s_config *get_config(void);
uint8_t config_get_app_port(void);
uint8_t *config_get_dev_eui(void);
uint8_t *config_get_app_eui(void);
uint8_t *config_get_app_key(void);
bool config_get_auto_join(void);
void config_set_auto_join(bool state);
uint8_t config_get_data_rate(void);
void config_set_data_rate(uint8_t rate);
uint8_t config_get_confirmed_msg(void);
void config_set_confirmed_msg(uint8_t type);
uint32_t config_get_max_inactive_time_window(void);
uint32_t config_get_max_failed_msg(void);
uint32_t config_get_join_try_interval(void);
uint8_t config_get_join_try_count(void);
uint8_t config_get_lora_mode(void);
uint32_t config_get_max_gps_on_time(void);
void config_set_max_gps_on_time(uint32_t time);
uint32_t config_get_send_repeat_time(void);
void config_set_send_repeat_time(uint32_t time);
uint32_t config_get_send_min_delay(void);
void config_set_send_min_delay(uint32_t time);
uint16_t config_get_max_join_retry_sessions_count(void);
#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
uint8_t *config_get_payload_key(void);
#endif
enum init_error_t status_get_boot_status(void);
void status_set_boot_status(enum init_error_t err_num);
char const *init_error_to_string(enum init_error_t err_num);
bool status_is_joined(void);
void status_set_joined(bool state);
bool status_get_gps_pwr_on(void);
void status_set_gps_pwr_on(bool state);
bool status_get_gps_fix(void);
void status_set_gps_fix(bool state);
uint64_t status_get_gps_total_on_time(void);
void status_set_gps_total_on_time(uint64_t time);
bool status_get_delayed_active(void);
void status_set_delayed_active(bool state);
uint32_t status_get_acc_events(void);
void status_inc_acc_events(void);
time_t status_get_last_accel_event(void);
void status_set_last_accel_event(time_t last_accel_time);
time_t status_get_last_pos_send(void);
void status_set_last_pos_send(time_t last_pos_send);
time_t status_get_last_pos_send_ok(void);
void status_set_last_pos_send_ok(time_t last_pos_send_ok);
uint16_t status_get_join_retry_sessions_count(void);
void status_set_join_retry_sessions_count(uint16_t count);
uint32_t status_get_msgs_sent(void);
void status_set_msgs_sent(uint32_t msgs);
uint32_t status_get_msgs_failed(void);
void status_set_msgs_failed(uint32_t msgs);
uint32_t status_get_msgs_failed_total(void);
void status_set_msgs_failed_total(uint32_t msgs);

#endif /* _APP_CONFIG_H_ */
