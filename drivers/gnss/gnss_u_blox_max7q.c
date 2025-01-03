/*
 * Copyright (c) 2023 Trackunit Corporation
 * Copyright (c) 2023 Bjarki Arge Andreasen
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gnss/gnss_publish.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/backend/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device_runtime.h>
#include <string.h>

#include <app/drivers/gnss/gnss_nmea0183.h>
#include <app/drivers/gnss/gnss_nmea0183_match.h>
#include <app/drivers/gnss/gnss_parse.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nmea_max7q, CONFIG_GNSS_LOG_LEVEL);

#define DT_DRV_COMPAT u_blox_nmea_max7q

#ifdef CONFIG_MAX7Q_PM_TIMEOUT_MS
#define MAX7Q_PM_TIMEOUT_MS CONFIG_MAX7Q_PM_TIMEOUT_MS
#else
#define MAX7Q_PM_TIMEOUT_MS 0U
#endif

#define UART_RX_BUF_SZ (256 + IS_ENABLED(CONFIG_GNSS_SATELLITES) * 512)
#define UART_TX_BUF_SZ 64
#define CHAT_RECV_BUF_SZ 256
#define CHAT_ARGV_SZ 32

struct max7q_config {
	const struct device *uart;
	const struct modem_chat_script *const init_chat_script;
#ifdef CONFIG_MAX7Q_POWER_ENABLE
	/** enable gpio (optional). */
	struct gpio_dt_spec enable;
#endif
};

struct max7q_data {
	struct gnss_nmea0183_match_data match_data;
#if CONFIG_GNSS_SATELLITES
	struct gnss_satellite satellites[CONFIG_GNSS_NMEA_U_BLOX_MAX_7Q_SATELLITES_COUNT];
#endif

	/* UART backend */
	struct modem_pipe *uart_pipe;
	struct modem_backend_uart uart_backend;
	uint8_t uart_backend_receive_buf[UART_RX_BUF_SZ];
	uint8_t uart_backend_transmit_buf[UART_TX_BUF_SZ];

	/* Modem chat */
	struct modem_chat chat;
	uint8_t chat_receive_buf[CHAT_RECV_BUF_SZ];
	uint8_t *chat_argv[CHAT_ARGV_SZ];
	uint8_t chat_delimiter[2];

	struct k_sem lock;
	k_timeout_t pm_timeout;
};

MODEM_CHAT_MATCHES_DEFINE(unsol_matches,
	MODEM_CHAT_MATCH_WILDCARD("$??GGA,", ",*", gnss_nmea0183_match_gga_callback),
	MODEM_CHAT_MATCH_WILDCARD("$??RMC,", ",*", gnss_nmea0183_match_rmc_callback),
#if CONFIG_GNSS_SATELLITES
	MODEM_CHAT_MATCH_WILDCARD("$??GSV,", ",*", gnss_nmea0183_match_gsv_callback),
#endif
);

static void max7q_pm_changed(const struct device *dev)
{
	struct max7q_data *data = dev->data;
	uint32_t pm_ready_at_ms;

	pm_ready_at_ms = k_uptime_get() + MAX7Q_PM_TIMEOUT_MS;
	data->pm_timeout = K_TIMEOUT_ABS_MS(pm_ready_at_ms);
}

static void max7q_await_pm_ready(const struct device *dev)
{
	struct max7q_data *data = dev->data;

	LOG_INF("Waiting until PM ready");
	k_sleep(data->pm_timeout);
}

static int max7q_resume(const struct device *dev)
{
#if IS_ENABLED(CONFIG_GNSS_RUN_INIT_CHAT_SCRIPT) || IS_ENABLED(CONFIG_MAX7Q_POWER_ENABLE)
	const struct max7q_config *config = dev->config;
#endif
	struct max7q_data *data = dev->data;
	int ret;

	LOG_INF("Resuming");

#if IS_ENABLED(CONFIG_MAX7Q_POWER_ENABLE)
	gpio_pin_set_dt(&config->enable, 1);
#endif
	max7q_await_pm_ready(dev);

	ret = modem_pipe_open(data->uart_pipe, K_SECONDS(10));
	if (ret < 0) {
		LOG_ERR("Failed to open pipe");
		return ret;
	}

	ret = modem_chat_attach(&data->chat, data->uart_pipe);
	if (ret < 0) {
		LOG_ERR("Failed to attach chat");
		modem_pipe_close(data->uart_pipe, K_SECONDS(10));
		return ret;
	}

#if IS_ENABLED(CONFIG_GNSS_RUN_INIT_CHAT_SCRIPT)
	ret = modem_chat_run_script(&data->chat, config->init_chat_script);
	if (ret < 0) {
		LOG_ERR("Failed to initialize GNSS");
		modem_pipe_close(data->uart_pipe, K_SECONDS(10));
		return ret;
	}
#endif

	LOG_INF("Resumed");

	return ret;
}

#ifdef CONFIG_PM_DEVICE
static void max7q_lock(const struct device *dev)
{
	struct max7q_data *data = dev->data;

	(void)k_sem_take(&data->lock, K_FOREVER);
}

static void max7q_unlock(const struct device *dev)
{
	struct max7q_data *data = dev->data;

	k_sem_give(&data->lock);
}

static int max7q_suspend(const struct device *dev)
{
	struct max7q_data *data = dev->data;
#if IS_ENABLED(CONFIG_MAX7Q_POWER_ENABLE)
	const struct max7q_config *config = dev->config;

	gpio_pin_set_dt(&config->enable, 0);
#endif
	LOG_INF("Suspending");

	max7q_await_pm_ready(dev);

	LOG_INF("Suspended");

	modem_pipe_close(data->uart_pipe, K_SECONDS(10));

	return 0;
}

static int max7q_turn_on(const struct device *dev)
{
	LOG_INF("Powered on");

	return 0;
}

static int max7q_turn_off(const struct device *dev)
{
	struct max7q_data *data = dev->data;

	LOG_INF("Powered off");

	return modem_pipe_close(data->uart_pipe, K_SECONDS(10));
}

static int max7q_pm_action(const struct device *dev, enum pm_device_action action)
{
	int ret = -ENOTSUP;

	max7q_lock(dev);

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		ret = max7q_suspend(dev);
		break;

	case PM_DEVICE_ACTION_RESUME:
		ret = max7q_resume(dev);
		break;

	case PM_DEVICE_ACTION_TURN_ON:
		ret = max7q_turn_on(dev);
		break;

	case PM_DEVICE_ACTION_TURN_OFF:
		ret = max7q_turn_off(dev);
		break;

	default:
		break;
	}

	max7q_pm_changed(dev);

	max7q_unlock(dev);
	return ret;
}
#endif /* CONFIG_PM_DEVICE */

static DEVICE_API(gnss, gnss_api) = {
};

static int max7q_init_nmea0183_match(const struct device *dev)
{
	struct max7q_data *data = dev->data;

	const struct gnss_nmea0183_match_config config = {
		.gnss = dev,
#if CONFIG_GNSS_SATELLITES
		.satellites = data->satellites,
		.satellites_size = ARRAY_SIZE(data->satellites),
#endif
	};

	return gnss_nmea0183_match_init(&data->match_data, &config);
}

static void max7q_init_pipe(const struct device *dev)
{
	const struct max7q_config *config = dev->config;
	struct max7q_data *data = dev->data;

	const struct modem_backend_uart_config uart_backend_config = {
		.uart = config->uart,
		.receive_buf = data->uart_backend_receive_buf,
		.receive_buf_size = ARRAY_SIZE(data->uart_backend_receive_buf),
		.transmit_buf = data->uart_backend_transmit_buf,
		.transmit_buf_size = ARRAY_SIZE(data->uart_backend_transmit_buf),
	};

	data->uart_pipe = modem_backend_uart_init(&data->uart_backend, &uart_backend_config);
}

static int max7q_init_chat(const struct device *dev)
{
	struct max7q_data *data = dev->data;

	const struct modem_chat_config chat_config = {
		.user_data = data,
		.receive_buf = data->chat_receive_buf,
		.receive_buf_size = ARRAY_SIZE(data->chat_receive_buf),
		.delimiter = data->chat_delimiter,
		.delimiter_size = ARRAY_SIZE(data->chat_delimiter),
		.filter = NULL,
		.filter_size = 0,
		.argv = data->chat_argv,
		.argv_size = ARRAY_SIZE(data->chat_argv),
		.unsol_matches = unsol_matches,
		.unsol_matches_size = ARRAY_SIZE(unsol_matches),
	};

	return modem_chat_init(&data->chat, &chat_config);
}

static int max7q_init(const struct device *dev)
{
	struct max7q_data *data = dev->data;
	int ret;

	k_sem_init(&data->lock, 1, 1);

	ret = max7q_init_nmea0183_match(dev);
	if (ret < 0) {
		return ret;
	}

	max7q_init_pipe(dev);

	ret = max7q_init_chat(dev);
	if (ret < 0) {
		return ret;
	}

	max7q_pm_changed(dev);

	if (pm_device_is_powered(dev)) {
		ret = max7q_resume(dev);
		if (ret < 0) {
			return ret;
		}
		max7q_pm_changed(dev);
	} else {
		pm_device_init_off(dev);
	}

	return pm_device_runtime_enable(dev);
}

#define INIT_CHAT_SCRIPT _CONCAT(DT_DRV_COMPAT, _init_chat_script)

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
MODEM_CHAT_SCRIPT_EMPTY_DEFINE(INIT_CHAT_SCRIPT);
#endif

#define MAX7Q_INST_NAME(inst, name) \
	_CONCAT(_CONCAT(_CONCAT(name, _), DT_DRV_COMPAT), inst)

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define MAX7Q_DEVICE(inst)							\
	static const struct max7q_config MAX7Q_INST_NAME(inst, config) = {	\
		.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),			\
		.init_chat_script = &_CONCAT(DT_DRV_COMPAT, _init_chat_script),	\
		IF_ENABLED(CONFIG_MAX7Q_POWER_ENABLE,				\
		(.enable = GPIO_DT_SPEC_INST_GET_OR(inst, enable_gpios, {}), ))	\
	};									\
	static struct max7q_data MAX7Q_INST_NAME(inst, data) = {		\
		.chat_delimiter = {'\r', '\n'},					\
	};									\
	PM_DEVICE_DT_INST_DEFINE(inst, max7q_pm_action);			\
	DEVICE_DT_INST_DEFINE(inst,						\
			max7q_init,						\
			PM_DEVICE_DT_INST_GET(inst),				\
			&MAX7Q_INST_NAME(inst, data),				\
			&MAX7Q_INST_NAME(inst, config),				\
			POST_KERNEL,						\
			CONFIG_GNSS_INIT_PRIORITY, &gnss_api);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(MAX7Q_DEVICE)
