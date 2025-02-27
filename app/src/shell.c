/*
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/shell/shell.h>
#include <version.h>
#include <app_version.h>
#include <zephyr/posix/time.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/lorawan/lorawan.h>

#include "config.h"
#include "battery.h"
#include "nvm.h"
#if IS_ENABLED(CONFIG_UBLOX_MAX7Q)
#include "gps.h"
#endif
#if IS_ENABLED(CONFIG_GNSS)
#include "gps_gnss.h"
#endif
#include "shell.h"
#include "lorawan_app.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(helium_mapper_shell);

struct shell_ctx_s {
	shell_cmd_cb_t shell_cb;
	void *data;
} shell_ctx = { .shell_cb = NULL, .data = NULL };

void shell_register_cb(shell_cmd_cb_t cb, void *data)
{
	shell_ctx.shell_cb = cb;
	shell_ctx.data = data;
}

size_t lorawan_hex2bin(const char *hex, size_t hexlen, uint8_t *buf, size_t buflen)
{
	uint8_t dec;

	if (buflen < hexlen / 2 + hexlen % 2) {
		return 0;
	}

	/* if hexlen is uneven, return whole number of hexlen / 2 */
	if (hexlen % 2) {
		return hexlen / 2;
	}

	/* regular hex conversion */
	for (size_t i = 0; i < hexlen / 2; i++) {
		if (char2hex(hex[2 * i], &dec) < 0) {
			return 0;
		}
		buf[i] = dec << 4;

		if (char2hex(hex[2 * i + 1], &dec) < 0) {
			return 0;
		}
		buf[i] += dec;
	}

	return hexlen / 2 + hexlen % 2;
}

#if IS_ENABLED(CONFIG_SHELL_START_OBSCURED)
static void login_init(void)
{
	LOG_INF("Welcome to helium mapper");
	if (!CONFIG_SHELL_CMD_ROOT[0]) {
		shell_set_root_cmd("login");
	}
}

static int check_passwd(char *passwd)
{
	char *password = config_get_password();
	return strcmp(passwd, password);
}

static int cmd_login(const struct shell *shell, size_t argc, char **argv)
{
	static uint32_t attempts;

	if (check_passwd(argv[1]) != 0) {
		shell_error(shell, "Incorrect password!");
		attempts++;
		if (attempts > 3) {
			k_sleep(K_SECONDS(attempts));
		}
		return -EINVAL;
	}

	/* clear history so password not visible there */
	z_shell_history_purge(shell->history);
	shell_obscure_set(shell, false);
	shell_set_root_cmd(NULL);
	shell_prompt_change(shell, "uart:~$ ");
	shell_print(shell, "Welcome to helium mapper\n");
	shell_print(shell, "Hit tab for help.\n");
	attempts = 0;
	return 0;
}

static int cmd_logout(const struct shell *shell, size_t argc, char **argv)
{
	shell_set_root_cmd("login");
	shell_obscure_set(shell, true);
	shell_prompt_change(shell, "login: ");
	shell_print(shell, "\n");
	return 0;
}

static int cmd_passwd(const struct shell *shell, size_t argc, char **argv)
{
	size_t len = strlen(argv[1]);

	if (argc < 2 || argc > 2)
	{
		shell_help(shell);
		return -EINVAL;
	}

	if (len < 5 || len > (PASSWORD_MAX_SIZE - 1))
	{
		shell_print(shell, "Please choose password between 5 and %d char",
				PASSWORD_MAX_SIZE - 1);
		return -EINVAL;
	}

	config_set_password(argv[1]);

#if IS_ENABLED(CONFIG_SETTINGS)
	hm_lorawan_nvm_save_settings(argv[0]);
#endif
	return 0;
}
#endif

static int cmd_config(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	uint8_t *dev_eui = config_get_dev_eui();
	uint8_t *app_eui = config_get_app_eui();
	uint8_t *app_key = config_get_app_key();
#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
	uint8_t *payload_key = config_get_payload_key();
#endif
	int i;

	shell_print(shell, "Device config:");
	shell_print(shell, "  RAK4631 Helium mapper (built: %s %s)", __DATE__, __TIME__);
	shell_print(shell, "    Kernel ver:    %s", STRINGIFY(BUILD_VERSION));
	shell_print(shell, "    App ver:       %s", STRINGIFY(APP_BUILD_VERSION));

	shell_fprintf(shell, SHELL_NORMAL, "  Dev EUI          ");
	for (i = 0; i < DEV_EUI_SIZE; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02X", *(dev_eui++) & 0xFF);
	}
	shell_print(shell, "");

	shell_fprintf(shell, SHELL_NORMAL, "  APP EUI          ");
	for (i = 0; i < APP_EUI_SIZE; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02X", *(app_eui++) & 0xFF);
	}
	shell_print(shell, "");

	shell_fprintf(shell, SHELL_NORMAL, "  APP key          ");
	for (i = 0; i < APP_KEY_SIZE; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02X", *(app_key++) & 0xFF);
	}
	shell_print(shell, "");

	shell_print(shell, "  Auto join        %s", config_get_auto_join() ? "true" : "false");
	shell_print(shell, "  Data rate/DR+    %d", config_get_data_rate());
	shell_print(shell, "  Confirmed msgs   %s", config_get_confirmed_msg() ? "true" : "false");
	shell_print(shell, "  Max failed msgs  %d", config_get_max_failed_msg());
	shell_print(shell, "  Inactive window  %d sec", config_get_max_inactive_time_window());
	shell_print(shell, "  Send interval    %d sec", config_get_send_repeat_time());
	shell_print(shell, "  Min delay        %d sec", config_get_send_min_delay());
	shell_print(shell, "  Max GPS ON time  %d sec", config_get_max_gps_on_time());

#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
	shell_fprintf(shell, SHELL_NORMAL, "  Payload key      ");
	for (i = 0; i < PAYLOAD_KEY_SIZE; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02X", *(payload_key++) & 0xFF);
	}
	shell_print(shell, "");
#endif

	return 0;
}
SHELL_CMD_ARG_REGISTER(config, NULL, "Show helium_mapper config", cmd_config, 1, 0);

static int cmd_status(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct timespec tp;
	struct tm tm;
	int64_t last_pos_send = status_get_last_pos_send();
	int64_t last_pos_send_ok = status_get_last_pos_send_ok();
	int64_t last_accel_event = status_get_last_accel_event();
	int64_t delta_sent = k_uptime_delta(&last_pos_send) / 1000;
	int64_t delta_sent_ok = k_uptime_delta(&last_pos_send_ok) / 1000;
	int64_t delta_acc = k_uptime_delta(&last_accel_event) / 1000;
	uint16_t join_retry_sessions_count = status_get_join_retry_sessions_count();
	enum init_error_t err_num = status_get_boot_status();

	clock_gettime(CLOCK_REALTIME, &tp);
	gmtime_r(&tp.tv_sec, &tm);

	shell_print(shell, "Device status:");
	shell_print(shell, "  boot status      %s", init_error_to_string(err_num));
	shell_print(shell, "  joined           %s", status_is_joined() ? "true" : "false");
	shell_print(shell, "  delayed active   %s", status_get_delayed_active() ? "true" : "false");
	shell_print(shell, "  gps power on     %s", status_get_gps_pwr_on() ? "true" : "false");
	shell_print(shell, "  messages sent    %d", status_get_msgs_sent());
	shell_print(shell, "  messages failed  %d", status_get_msgs_failed());
	shell_print(shell, "  msg failed total %d", status_get_msgs_failed_total());
	shell_print(shell, "  Accel events     %d", status_get_acc_events());
	shell_print(shell, "  Join retry sess  %d", join_retry_sessions_count);
	shell_print(shell, "  Total GPS ON     %lld sec", status_get_gps_total_on_time());
	shell_print(shell, "  last msg sent    %lld sec", delta_sent);
	shell_print(shell, "  last msg sent OK %lld sec", delta_sent_ok);
	shell_print(shell, "  last acc event   %lld sec", delta_acc);
	shell_print(shell, "  Uptime           %04d-%02u-%02u %02u:%02u:%02u",
		    tm.tm_year - 70,
		    tm.tm_mon,
		    tm.tm_mday - 1,
		    tm.tm_hour,
		    tm.tm_min,
		    tm.tm_sec);

	return 0;
}
SHELL_CMD_ARG_REGISTER(status, NULL, "Show helium_mapper status", cmd_status, 1, 0);

#if IS_ENABLED(CONFIG_BATTERY)
static int cmd_battery(const struct shell *shell, size_t argc, char **argv)
{
	uint16_t batt_pptt;
	int batt_mV;
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	err = read_battery(&batt_mV);
	if (err < 0) {
		return err;
	}

	batt_pptt = battery_level_pptt(batt_mV, levels);
	shell_print(shell, "Battery: %d mV, %u %%", batt_mV, batt_pptt / 100);

	return 0;
}
SHELL_CMD_ARG_REGISTER(battery, NULL, "Show battery status", cmd_battery, 1, 0);
#endif

#if IS_ENABLED(CONFIG_UBLOX_MAX7Q)
static int cmd_location(const struct shell *shell, size_t argc, char **argv)
{
	struct s_mapper_data data;
	struct sensor_value lat, lng, alt, acc;
	double lat_f, lng_f, alt_f, acc_f;
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	read_location(&data);

	lat_f = data.lat;
	lat_f /= 100000;
	lng_f = data.lng;
	lng_f /= 100000;
	alt_f = data.alt;
	acc_f = data.accuracy;
	acc_f /= 100;

	err = sensor_value_from_double(&lat, lat_f);
	err |= sensor_value_from_double(&lng, lng_f);
	err |= sensor_value_from_double(&alt, alt_f);
	err |= sensor_value_from_double(&acc, acc_f);
	if (err) {
		LOG_ERR("Can't convert from double to sensor value.");
		return -EINVAL;
	}

	shell_print(shell, "lat: %d.%06d, lng: %d.%06d, alt: %d.%06d, acc: %d.%06d",
			lat.val1, lat.val2,
			lng.val1, lng.val2,
			alt.val1, alt.val2,
			acc.val1, acc.val2);

	return 0;
}
SHELL_CMD_ARG_REGISTER(location, NULL, "Show GPS location", cmd_location, 1, 0);
#endif

#if IS_ENABLED(CONFIG_GNSS)
#define GNSS_LOG_BUF_SIZE 128
static int cmd_location(const struct shell *shell, size_t argc, char **argv)
{
	static char dump_buf[GNSS_LOG_BUF_SIZE];
	int ret;

	ret = print_location_to_str(dump_buf, GNSS_LOG_BUF_SIZE);
	if (ret < 0)
	{
		shell_print(shell, "err: %d", ret);
	} else {
		shell_print(shell, "%s", dump_buf);
	}

	return 0;
}
SHELL_CMD_ARG_REGISTER(location, NULL, "Show GPS location", cmd_location, 1, 0);
#endif

static int cmd_reboot(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "Reboot...");
	sys_reboot(SYS_REBOOT_WARM);

	return 0;
}
SHELL_CMD_ARG_REGISTER(reboot, NULL, "Reboot deice", cmd_reboot, 1, 0);

static int shell_lorawan_hexdump(const struct shell *shell, char *buf, size_t len, const char *label)
{
	shell_fprintf(shell, SHELL_NORMAL, "%s", label);
	for (int i = 0; i < len; i++) {
		shell_fprintf(shell, SHELL_NORMAL, "%02X", (unsigned char)buf[i] & 0xFFu);
	}
	shell_print(shell, "");

	return 0;
}

#define BUF_SIZE MAX(APP_KEY_SIZE, PAYLOAD_KEY_SIZE)

static int cmd_lorawan_keys(const struct shell *shell, size_t argc, char **argv)
{
	bool save = false;
	size_t len;
	uint8_t buf[BUF_SIZE];
	uint8_t *dev_eui = config_get_dev_eui();
	uint8_t *app_eui = config_get_app_eui();
	uint8_t *app_key = config_get_app_key();
#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
	uint8_t *payload_key = config_get_payload_key();
#endif

	if (argc < 2) {
		if (!strncmp(argv[0], "dev_eui", strlen("dev_eui"))) {
			shell_lorawan_hexdump(shell, dev_eui,
					DEV_EUI_SIZE, "dev_eui ");
		}
		else if (!strncmp(argv[0], "app_eui", strlen("app_eui"))) {
			shell_lorawan_hexdump(shell, app_eui,
					APP_EUI_SIZE, "app_eui ");
		}
		else if (!strncmp(argv[0], "app_key", strlen("app_key"))) {
			shell_lorawan_hexdump(shell, app_key,
					APP_KEY_SIZE, "app_key ");
		}
#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
		else if (!strncmp(argv[0], "payload_key", strlen("payload_key"))) {
			shell_lorawan_hexdump(shell, payload_key,
					PAYLOAD_KEY_SIZE, "payload_key ");
		}
#endif
	} else {
		if (!strncmp(argv[0], "dev_eui", strlen("dev_eui"))) {
			len = lorawan_hex2bin(argv[1], strlen(argv[1]), buf, DEV_EUI_SIZE);
			if (len != DEV_EUI_SIZE) {
				LOG_WRN("Not enough or invalid characters, len: %d", len);
				return -EINVAL;
			}
			LOG_HEXDUMP_DBG(buf, len, "dev_eui: ");
			memcpy(dev_eui, buf, DEV_EUI_SIZE);
			save = true;
		}
		else if (!strncmp(argv[0], "app_eui", strlen("app_eui"))) {
			len = lorawan_hex2bin(argv[1], strlen(argv[1]), buf, APP_EUI_SIZE);
			if (len != APP_EUI_SIZE) {
				LOG_WRN("Not enough or invalid characters, len: %d", len);
				return -EINVAL;
			}
			LOG_HEXDUMP_DBG(buf, len, "app_eui: ");
			memcpy(app_eui, buf, APP_EUI_SIZE);
			save = true;
		}
		else if (!strncmp(argv[0], "app_key", strlen("app_key"))) {
			len = lorawan_hex2bin(argv[1], strlen(argv[1]), buf, APP_KEY_SIZE);
			if (len != APP_KEY_SIZE) {
				LOG_WRN("Not enough or invalid characters, len: %d", len);
				return -EINVAL;
			}
			LOG_HEXDUMP_DBG(buf, len, "app_key: ");
			memcpy(app_key, buf, APP_KEY_SIZE);
			save = true;
		}
#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
		else if (!strncmp(argv[0], "payload_key", strlen("payload_key"))) {
			len = lorawan_hex2bin(argv[1], strlen(argv[1]), buf, PAYLOAD_KEY_SIZE);
			if (len != 16) {
				LOG_WRN("Not enough or invalid characters, len: %d", len);
				return -EINVAL;
			}
			LOG_HEXDUMP_DBG(buf, len, "payload_key: ");
			memcpy(payload_key, buf, PAYLOAD_KEY_SIZE);
			save = true;
		}
#endif
	}

#if IS_ENABLED(CONFIG_SETTINGS)
	if (save) {
		hm_lorawan_nvm_save_settings(argv[0]);
	}
#endif

	return 0;
}

static int cmd_auto_join(const struct shell *shell, size_t argc, char **argv)
{
	bool save = false;

	if (argc < 2) {
		shell_print(shell, "%s", config_get_auto_join() ? "true" : "false");
	} else {
		if (!strncmp(argv[1], "true", strlen("true"))) {
			config_set_auto_join(true);
			save = true;
		}
		if (!strncmp(argv[1], "false", strlen("false"))) {
			config_set_auto_join(false);
			save = true;
		}

		if (save) {
#if IS_ENABLED(CONFIG_SETTINGS)
			hm_lorawan_nvm_save_settings("auto_join");
#endif
		} else {
			shell_print(shell, "Invalid input: valid are true/false");
		}
	}

	return 0;
}

static int cmd_confirmed_msg(const struct shell *shell, size_t argc, char **argv)
{
	bool save = false;

	if (argc < 2) {
		shell_print(shell, "%s", config_get_confirmed_msg() ? "true" : "false");
	} else {
		if (!strncmp(argv[1], "true", strlen("true"))) {
			config_set_confirmed_msg(LORAWAN_MSG_CONFIRMED);
			save = true;
		}
		if (!strncmp(argv[1], "false", strlen("false"))) {
			config_set_confirmed_msg(LORAWAN_MSG_UNCONFIRMED);
			save = true;
		}
#if IS_ENABLED(CONFIG_SETTINGS)
		if (save) {
			hm_lorawan_nvm_save_settings("confirmed_msg");
		}
#endif
	}

	return 0;
}

static int cmd_send_interval(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(shell, "%u sec", config_get_send_repeat_time());
		if (shell_ctx.shell_cb) {
			shell_ctx.shell_cb(SHELL_CMD_SEND_TIMER_GET, shell_ctx.data);
		}
	} else {
		config_set_send_repeat_time(atoi(argv[1]));
#if IS_ENABLED(CONFIG_SETTINGS)
		hm_lorawan_nvm_save_settings("send_repeat_time");
#endif
		if (shell_ctx.shell_cb) {
			shell_ctx.shell_cb(SHELL_CMD_SEND_TIMER_SET, shell_ctx.data);
		}
	}

	return 0;
}

static int cmd_min_delay(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(shell, "%u sec", config_get_send_min_delay());
	} else {
		config_set_send_min_delay(atoi(argv[1]));
#if IS_ENABLED(CONFIG_SETTINGS)
		hm_lorawan_nvm_save_settings("send_min_delay");
#endif
	}

	return 0;
}

static int cmd_max_gps_on(const struct shell *shell, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(shell, "%u sec", config_get_max_gps_on_time());
	} else {
		config_set_max_gps_on_time(atoi(argv[1]));
#if IS_ENABLED(CONFIG_SETTINGS)
		hm_lorawan_nvm_save_settings("max_gps_on_time");
#endif
	}

	return 0;
}

static int cmd_data_rate(const struct shell *shell, size_t argc, char **argv)
{
	int rate;

	if (argc < 2) {
		shell_print(shell, "%d", config_get_data_rate());
	} else {
		rate = atoi(argv[1]);
		if (rate > LORAWAN_DR_15 || rate < LORAWAN_DR_0) {
			return -EINVAL;
		}
		config_set_data_rate(rate);
#if IS_ENABLED(CONFIG_SETTINGS)
		hm_lorawan_nvm_save_settings("data_rate");
#endif
	}

	return 0;
}

#define HELP_DEV_EUI "Get/set dev_eui [0011223344556677]"
#define HELP_APP_EUI "Get/set app_eui [0011223344556677]"
#define HELP_APP_KEY "get/set app_key [00112233445566778899aabbccddeeff]"
#define HELP_AUTO_JOIN "Auto join true/false"
#define HELP_CONFIRMED_MSG "Confirmed messages true/false"
#define HELP_SEND_INTERVAL "Send interval in seconds"
#define HELP_MIN_DELAY "Min delay between 2 messages in ms"
#define HELP_MAX_GPS_ON "Max time GPS is ON if no one using it in seconds"
#define HELP_DATA_RATE "Get/set data rate 0-15"
#define HELP_PAYLOAD_KEY "get/set payload_key [00000000000000000000000000000000]"

SHELL_STATIC_SUBCMD_SET_CREATE(sub_lorawan,
	SHELL_CMD_ARG(dev_eui, NULL, HELP_DEV_EUI, cmd_lorawan_keys, 1, 1),
	SHELL_CMD_ARG(app_eui, NULL, HELP_APP_EUI, cmd_lorawan_keys, 1, 1),
	SHELL_CMD_ARG(app_key, NULL, HELP_APP_KEY, cmd_lorawan_keys, 1, 1),
	SHELL_CMD_ARG(auto_join, NULL, HELP_AUTO_JOIN, cmd_auto_join, 1, 1),
	SHELL_CMD_ARG(confirmed_msg, NULL, HELP_CONFIRMED_MSG, cmd_confirmed_msg, 1, 1),
	SHELL_CMD_ARG(send_interval, NULL, HELP_SEND_INTERVAL, cmd_send_interval, 1, 1),
	SHELL_CMD_ARG(min_delay, NULL, HELP_MIN_DELAY, cmd_min_delay, 1, 1),
	SHELL_CMD_ARG(max_gps_on_time, NULL, HELP_MAX_GPS_ON, cmd_max_gps_on, 1, 1),
	SHELL_CMD_ARG(data_rate, NULL, HELP_DATA_RATE, cmd_data_rate, 1, 1),
#if IS_ENABLED(CONFIG_PAYLOAD_ENCRYPTION)
	SHELL_CMD_ARG(payload_key, NULL, HELP_PAYLOAD_KEY, cmd_lorawan_keys, 1, 1),
#endif
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(lorawan, &sub_lorawan, "lorawan commands", NULL);

#if IS_ENABLED(CONFIG_SHELL_START_OBSCURED)
SHELL_CMD_ARG_REGISTER(login, NULL, "<password>", cmd_login, 2, 0);
SHELL_CMD_REGISTER(logout, NULL, "Log out.", cmd_logout);
SHELL_CMD_REGISTER(passwd, NULL, "<new password>", cmd_passwd);
#endif

void dl_shell_cmd_exec(uint8_t len, const uint8_t *data)
{
	char cmd_buff[DL_SHELL_CMD_BUF_SIZE];
	char *shell_cmd;
	size_t cmd_len;
	int err;

	/* at least single character cmd requred after "shell " prefix */
	if (len < (strlen(DL_SHELL_CMD_PREFIX) + 1)) {
		return;
	}

	cmd_len = len < DL_SHELL_CMD_BUF_SIZE ? len : DL_SHELL_CMD_BUF_SIZE;
	strncpy(cmd_buff, data, cmd_len);
	cmd_buff[cmd_len] = '\0';

	shell_cmd = strstr(cmd_buff, DL_SHELL_CMD_PREFIX);
	if (shell_cmd == NULL) {
		return;
	}

	shell_cmd += strlen(DL_SHELL_CMD_PREFIX);

	LOG_INF("shell: execute cmd: '%s'", shell_cmd);

	err = shell_execute_cmd(NULL, shell_cmd);
	if (err) {
		LOG_ERR("Can't execute shell cmd: '%s', err: %d", shell_cmd, err);
	}
}

int init_shell(void)
{
	const struct device *dev;

	dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart));
	if (!device_is_ready(dev)) {
		LOG_ERR("Can't find shell device");
		return -ENODEV;
	}

#if IS_ENABLED(CONFIG_SHELL_START_OBSCURED)
	login_init();
#endif

	return 0;
}
