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

/**
 * @brief Encode the manager's supported modes using the shared protocol codec.
 *
 * Mode names remain owned by AutoOffManager; encoding reads them synchronously.
 * @return Encoded byte count, or negative errno on invalid metadata or capacity.
 */
static ssize_t encode_supported_modes(uint8_t *payload, size_t capacity)
{
	power_saving_mode_description_t modes[POWER_SAVING_LEVEL_COUNT] = {0};
	uint8_t count = auto_off_get_supported_mode_count();
	size_t encoded_size;

	if (count > ARRAY_SIZE(modes)) {
		return -EOVERFLOW;
	}
	for (uint8_t id = 0; id < count; id++) {
		const char *name = auto_off_get_mode_name((power_saving_level_t)id);
		if (name == NULL) {
			return -EINVAL;
		}
		size_t name_length = strlen(name);
		if (name_length > UINT8_MAX) {
			return -EOVERFLOW;
		}
		modes[id].id = id;
		modes[id].name_length = (uint8_t)name_length;
		/* Generated storage is mutable for decoding; encoding never modifies it. */
		modes[id].name = (uint8_t *)name;
	}
	power_saving_supported_modes_t message = {
		.count = count,
		.modes = modes,
	};
	protocol_status_t status = power_saving_supported_modes_encode(
		&message, payload, capacity, &encoded_size);
	if (status != PROTOCOL_OK) {
		return status == PROTOCOL_ERROR_BUFFER_TOO_SMALL ? -ENOMEM : -EINVAL;
	}
	return (ssize_t)encoded_size;
}

/** @brief Return the selected automatic power-off mode as a generated message. */
static ssize_t read_power_saving_mode(struct bt_conn *conn,
				      const struct bt_gatt_attr *attr,
				      void *buf,
				      uint16_t len,
				      uint16_t offset)
{
	power_saving_mode_t message = { .id = (uint8_t)auto_off_get_mode() };
	uint8_t payload[1];
	size_t size;
	if (power_saving_mode_encode(&message, payload, sizeof(payload), &size) != PROTOCOL_OK) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
	return bt_gatt_attr_read(conn, attr, buf, len, offset, payload, size);
}

/** @brief Decode and validate a supported mode before applying it. */
static ssize_t write_power_saving_mode(struct bt_conn *conn,
				       const struct bt_gatt_attr *attr,
				       const void *buf,
				       uint16_t len,
				       uint16_t offset,
				       uint8_t flags)
{
	power_saving_mode_t message;
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

	if (power_saving_mode_decode(&message, buf, len, NULL) != PROTOCOL_OK) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	mode = (power_saving_level_t)message.id;
	if (!auto_off_mode_is_supported(mode)) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	auto_off_set_mode(mode);
	return len;
}

/** @brief Return the supported mode IDs and display names. */
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
	BT_GATT_PRIMARY_SERVICE(POWER_SAVING_ZEPHYR_SERVICE_UUID),
	BT_GATT_CHARACTERISTIC(POWER_SAVING_ZEPHYR_MODE_CHARACTERISTIC_UUID,
                               POWER_SAVING_ZEPHYR_MODE_CHARACTERISTIC_PROPERTIES,
                               POWER_SAVING_ZEPHYR_MODE_CHARACTERISTIC_PERMISSIONS,
			       read_power_saving_mode, write_power_saving_mode, NULL),
	BT_GATT_CHARACTERISTIC(POWER_SAVING_ZEPHYR_SUPPORTED_MODES_CHARACTERISTIC_UUID,
                               POWER_SAVING_ZEPHYR_SUPPORTED_MODES_CHARACTERISTIC_PROPERTIES,
                               POWER_SAVING_ZEPHYR_SUPPORTED_MODES_CHARACTERISTIC_PERMISSIONS,
			       read_supported_power_saving_modes, NULL, NULL),
);
