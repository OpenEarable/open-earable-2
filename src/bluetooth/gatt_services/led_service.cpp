#include "led_service.h"

#include "../../utils/StateIndicator.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(led_service, CONFIG_BLE_LOG_LEVEL);

/** @brief Decode an RGB command and apply the custom indicator color. */
static ssize_t write_led(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
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

	return len;
}

/** @brief Decode and apply the indicator mode. */
static ssize_t write_state(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
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
	state_indicator.set_indication_mode(static_cast<led_mode>(message.mode));

	return len;
}

BT_GATT_SERVICE_DEFINE(rgb_led_svc,
BT_GATT_PRIMARY_SERVICE(LED_ZEPHYR_SERVICE_UUID),
    BT_GATT_CHARACTERISTIC(LED_ZEPHYR_RGB_CHARACTERISTIC_UUID,
                LED_ZEPHYR_RGB_CHARACTERISTIC_PROPERTIES,
                LED_ZEPHYR_RGB_CHARACTERISTIC_PERMISSIONS,
                NULL, write_led, NULL),
	BT_GATT_CHARACTERISTIC(LED_ZEPHYR_STATE_CHARACTERISTIC_UUID,
                LED_ZEPHYR_STATE_CHARACTERISTIC_PROPERTIES,
                LED_ZEPHYR_STATE_CHARACTERISTIC_PERMISSIONS,
                NULL, write_state, NULL),
);

/** @brief Initialize the LED controller used by the GATT service. */
int init_led_service() {
	led_controller.begin();
	return 0;
}
