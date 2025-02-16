#include "led_service.h"

#include "../../utils/StateIndicator.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(led_service, CONFIG_BLE_LOG_LEVEL);

static ssize_t write_led(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	//printk("Attribute write, handle: %u, conn: %p", attr->handle,
		//(void *)conn);

	if (len != 3U) {
		LOG_INF("Write led: Incorrect data length");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		LOG_INF("Write led: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

    //earable_led.set_color((uint8_t*)buf);
	led_controller.setColor((uint8_t*)buf);

	return len;
}

static ssize_t write_state(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	//printk("Attribute write, handle: %u, conn: %p", attr->handle,
		//(void *)conn);

	if (len != 1U) {
		LOG_INF("Write led: Incorrect data length");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		LOG_INF("Write led: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	state_indicator.set_led_state((led_state) *((uint8_t*)buf));

	return len;
}

BT_GATT_SERVICE_DEFINE(rgb_led_svc,
BT_GATT_PRIMARY_SERVICE(BT_UUID_LED),
    BT_GATT_CHARACTERISTIC(BT_UUID_LED_RGB,
                BT_GATT_CHRC_WRITE,
                BT_GATT_PERM_WRITE,
                NULL, write_led, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_LED_STATE,
                BT_GATT_CHRC_WRITE,
                BT_GATT_PERM_WRITE,
                NULL, write_state, NULL),
);

void LED_Service::begin() {
    //earable_led.init();
	led_controller.begin();
}

LED_Service led_service;
