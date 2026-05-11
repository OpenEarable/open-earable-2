#include "power_saving_service.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/util.h>

#include "AutoOffManager.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(power_saving_service, CONFIG_BLE_LOG_LEVEL);

#define POWER_SAVING_SUPPORTED_MODES_MAX_PAYLOAD_LEN 128

static uint8_t supported_modes_payload[POWER_SAVING_SUPPORTED_MODES_MAX_PAYLOAD_LEN];

static ssize_t encode_supported_modes(uint8_t *payload, size_t capacity)
{
	uint8_t mode_count = auto_off_get_supported_mode_count();
	size_t payload_len = 0;

	if (capacity < 1) {
		return -ENOMEM;
	}

	payload[payload_len++] = mode_count;

	for (uint8_t mode_id = 0; mode_id < mode_count; mode_id++) {
		const char *name = auto_off_get_mode_name((power_saving_level_t)mode_id);
		size_t name_len;

		if (name == NULL) {
			return -EINVAL;
		}

		name_len = strlen(name);
		if (name_len > UINT8_MAX ||
		    payload_len + 2 + name_len > capacity) {
			return -ENOMEM;
		}

		payload[payload_len++] = mode_id;
		payload[payload_len++] = (uint8_t)name_len;
		memcpy(&payload[payload_len], name, name_len);
		payload_len += name_len;
	}

	return payload_len;
}

static ssize_t read_power_saving_mode(struct bt_conn *conn,
				      const struct bt_gatt_attr *attr,
				      void *buf,
				      uint16_t len,
				      uint16_t offset)
{
	uint8_t mode = (uint8_t)auto_off_get_mode();

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &mode, sizeof(mode));
}

static ssize_t write_power_saving_mode(struct bt_conn *conn,
				       const struct bt_gatt_attr *attr,
				       const void *buf,
				       uint16_t len,
				       uint16_t offset,
				       uint8_t flags)
{
	const uint8_t *mode_buf = buf;
	power_saving_level_t mode;

	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(uint8_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	mode = (power_saving_level_t)mode_buf[0];
	if (!auto_off_mode_is_supported(mode)) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	auto_off_set_mode(mode);
	return len;
}

static ssize_t read_supported_power_saving_modes(struct bt_conn *conn,
						 const struct bt_gatt_attr *attr,
						 void *buf,
						 uint16_t len,
						 uint16_t offset)
{
	ssize_t payload_len = encode_supported_modes(
		supported_modes_payload,
		sizeof(supported_modes_payload));

	if (payload_len < 0) {
		LOG_ERR("Failed to encode supported power saving modes: %d", (int)payload_len);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, supported_modes_payload,
				 (uint16_t)payload_len);
}

BT_GATT_SERVICE_DEFINE(power_saving_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_POWER_SAVING_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_POWER_SAVING_MODE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_power_saving_mode, write_power_saving_mode, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_POWER_SAVING_SUPPORTED_MODES,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_supported_power_saving_modes, NULL, NULL),
);
