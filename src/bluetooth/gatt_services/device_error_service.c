#include "device_error_service.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#include "zbus_common.h"
#include "openearable_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(device_error_service, CONFIG_BLE_LOG_LEVEL);

ZBUS_CHAN_DECLARE(bt_mgmt_chan);

static bool device_error_notify_enabled;
static device_error_data_t device_error_data;

static void device_error_bt_evt_handler(const struct zbus_channel *chan);
ZBUS_LISTENER_DEFINE(device_error_bt_evt_listen, device_error_bt_evt_handler);

static void device_error_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	device_error_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Device error notifications %s", device_error_notify_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(device_error_service,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DEVICE_ERROR_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_DEVICE_ERROR_EVENT,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, &device_error_data),
	BT_GATT_CCC(device_error_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void device_error_bt_evt_handler(const struct zbus_channel *chan)
{
	const struct bt_mgmt_msg *msg = zbus_chan_const_msg(chan);

	if (msg->event == BT_MGMT_DISCONNECTED) {
		device_error_notify_enabled = false;
	}
}

static void copy_message(char *dest, size_t dest_len, const char *message)
{
	if (dest_len == 0) {
		return;
	}

	if (message == NULL) {
		message = "";
	}

	strncpy(dest, message, dest_len - 1);
	dest[dest_len - 1] = '\0';
}

static void format_message(char *dest, size_t dest_len, const char *fmt, va_list args)
{
	if (dest_len == 0) {
		return;
	}

	if (fmt == NULL) {
		dest[0] = '\0';
		return;
	}

	(void)vsnprintf(dest, dest_len, fmt, args);
	dest[dest_len - 1] = '\0';
}

int init_device_error_service(void)
{
	int ret = zbus_chan_add_obs(&bt_mgmt_chan, &device_error_bt_evt_listen, ZBUS_ADD_OBS_TIMEOUT_MS);

	if (ret) {
		LOG_ERR("Failed to add device error bt_mgmt listener: %d", ret);
		return ret;
	}

	return 0;
}

int send_device_error(enum device_error_level level, uint16_t error_code, uint8_t source_id,
		      const char *message)
{
	if (!device_error_notify_enabled) {
		return -ENOENT;
	}

	device_error_data.version = DEVICE_ERROR_PAYLOAD_VERSION;
	device_error_data.level = (uint8_t)level;
	device_error_data.error_code = error_code;
	device_error_data.source_id = source_id;
	device_error_data.timestamp_ms = k_uptime_get_32();
	copy_message(device_error_data.message, sizeof(device_error_data.message), message);

	int err = bt_gatt_notify(NULL, &device_error_service.attrs[2], &device_error_data,
				 sizeof(device_error_data));

	if (err) {
		LOG_WRN("Failed to send device error notification: %d", err);
	}

	return err;
}

int send_device_errorf(enum device_error_level level, uint16_t error_code, uint8_t source_id,
		       const char *fmt, ...)
{
	char message[DEVICE_ERROR_MESSAGE_MAX_LENGTH];
	va_list args;

	va_start(args, fmt);
	format_message(message, sizeof(message), fmt, args);
	va_end(args);

	return send_device_error(level, error_code, source_id, message);
}

void device_error_log_inf(uint16_t error_code, uint8_t source_id, const char *fmt, ...)
{
	char message[DEVICE_ERROR_MESSAGE_MAX_LENGTH];
	va_list args;

	va_start(args, fmt);
	format_message(message, sizeof(message), fmt, args);
	va_end(args);

	LOG_INF("%s", message);
	(void)send_device_error(DEVICE_ERROR_LEVEL_INFO, error_code, source_id, message);
}

void device_error_log_wrn(uint16_t error_code, uint8_t source_id, const char *fmt, ...)
{
	char message[DEVICE_ERROR_MESSAGE_MAX_LENGTH];
	va_list args;

	va_start(args, fmt);
	format_message(message, sizeof(message), fmt, args);
	va_end(args);

	LOG_WRN("%s", message);
	(void)send_device_error(DEVICE_ERROR_LEVEL_WARNING, error_code, source_id, message);
}

void device_error_log_err(uint16_t error_code, uint8_t source_id, const char *fmt, ...)
{
	char message[DEVICE_ERROR_MESSAGE_MAX_LENGTH];
	va_list args;

	va_start(args, fmt);
	format_message(message, sizeof(message), fmt, args);
	va_end(args);

	LOG_ERR("%s", message);
	(void)send_device_error(DEVICE_ERROR_LEVEL_ERROR, error_code, source_id, message);
}

void device_error_log_fatal(uint16_t error_code, uint8_t source_id, const char *fmt, ...)
{
	char message[DEVICE_ERROR_MESSAGE_MAX_LENGTH];
	va_list args;

	va_start(args, fmt);
	format_message(message, sizeof(message), fmt, args);
	va_end(args);

	LOG_ERR("%s", message);
	(void)send_device_error(DEVICE_ERROR_LEVEL_FATAL, error_code, source_id, message);
}
