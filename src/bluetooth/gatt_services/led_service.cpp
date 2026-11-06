#include "led_service.h"

#include "../../utils/StateIndicator.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(led_service, CONFIG_BLE_LOG_LEVEL);

/** @brief Encode the stored custom RGB color for a GATT read. */
static ssize_t read_led(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    RGBColor color;
    state_indicator.get_custom_color(color);
    led_rgb_t message = {color[0], color[1], color[2]};
    uint8_t payload[3];
    size_t size;
    if (led_rgb_encode(&message, payload, sizeof(payload), &size) != PROTOCOL_OK) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    return bt_gatt_attr_read(conn, attr, buf, len, offset, payload, size);
}

/** @brief Encode the current indication mode for a GATT read. */
static ssize_t read_state(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         void *buf, uint16_t len, uint16_t offset)
{
    led_state_t message = {static_cast<uint8_t>(state_indicator.get_indication_mode())};
    uint8_t payload[1];
    size_t size;
    if (led_state_encode(&message, payload, sizeof(payload), &size) != PROTOCOL_OK) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    return bt_gatt_attr_read(conn, attr, buf, len, offset, payload, size);
}

/** @brief Decode an RGB command, apply it, and notify subscribers. */
static ssize_t write_led(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(flags);
	if (len != 3U) {
		LOG_INF("Write led: Incorrect data length");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		LOG_INF("Write led: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	led_rgb_t message;
	if (led_rgb_decode(&message, static_cast<const uint8_t *>(buf), len, nullptr) != PROTOCOL_OK) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	RGBColor color = {message.red, message.green, message.blue};
	state_indicator.set_custom_color(color);
	(void)bt_gatt_notify(NULL, attr, buf, len);

	return len;
}

/** @brief Decode and validate the indication mode before notifying subscribers. */
static ssize_t write_state(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(flags);
	if (len != 1U) {
		LOG_INF("Write led: Incorrect data length");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		LOG_INF("Write led: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	led_state_t message;
	if (led_state_decode(&message, static_cast<const uint8_t *>(buf), len, nullptr) != PROTOCOL_OK) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	uint8_t mode = message.mode;
	if (mode != STATE_INDICATION && mode != CUSTOM) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
	state_indicator.set_indication_mode((led_mode)mode);
	(void)bt_gatt_notify(NULL, attr, buf, len);

	return len;
}

BT_GATT_SERVICE_DEFINE(rgb_led_svc,
BT_GATT_PRIMARY_SERVICE(LED_ZEPHYR_SERVICE_UUID),
    BT_GATT_CHARACTERISTIC(LED_ZEPHYR_RGB_CHARACTERISTIC_UUID,
                LED_ZEPHYR_RGB_CHARACTERISTIC_PROPERTIES,
                LED_ZEPHYR_RGB_CHARACTERISTIC_PERMISSIONS,
                read_led, write_led, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(LED_ZEPHYR_STATE_CHARACTERISTIC_UUID,
                LED_ZEPHYR_STATE_CHARACTERISTIC_PROPERTIES,
                LED_ZEPHYR_STATE_CHARACTERISTIC_PERMISSIONS,
                read_state, write_state, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/** @brief Initialize the LED controller used by the GATT service. */
int init_led_service() {
	led_controller.begin();
	return 0;
}
