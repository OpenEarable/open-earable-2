#include "button_service.h"

#include "macros_common.h"
#include "button_assignments.h"

#include <zephyr/zbus/zbus.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(button_manager, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);

static uint8_t button_state = BUTTON_RELEASED;

static bool notify_enabled;

static struct k_thread thread_data;
static k_tid_t thread_id;

ZBUS_SUBSCRIBER_DEFINE(button_gatt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);

ZBUS_CHAN_DECLARE(button_chan);

static K_THREAD_STACK_DEFINE(thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE);

/** @brief Track whether button notifications are enabled. */
static void button_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	ARG_UNUSED(attr);
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

/** @brief Encode the latest button action for a GATT read. */
static ssize_t read_button_state(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	button_state_t message = { .action = button_state };
	uint8_t payload[1];
	size_t size;
	if (button_state_encode(&message, payload, sizeof(payload), &size) != PROTOCOL_OK) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
	return bt_gatt_attr_read(conn, attr, buf, len, offset, payload, size);
}

BT_GATT_SERVICE_DEFINE(button_service,
BT_GATT_PRIMARY_SERVICE(BUTTON_ZEPHYR_SERVICE_UUID),
BT_GATT_CHARACTERISTIC(BUTTON_ZEPHYR_STATE_CHARACTERISTIC_UUID,
                BUTTON_ZEPHYR_STATE_CHARACTERISTIC_PROPERTIES,
                BUTTON_ZEPHYR_STATE_CHARACTERISTIC_PERMISSIONS,
            read_button_state, NULL, &button_state),
BT_GATT_CCC(button_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/** @brief Store the latest button action and notify subscribed peers. */
int bt_send_button_state(enum button_action _button_state)
{
	button_state = _button_state;

	if (!notify_enabled) {
		return -EACCES;
	}

	button_state_t message = { .action = button_state };
	uint8_t payload[1];
	size_t size;
	if (button_state_encode(&message, payload, sizeof(payload), &size) != PROTOCOL_OK) {
		return -EINVAL;
	}
	return bt_gatt_notify(NULL, &button_service.attrs[2], payload, size);
}

/** @brief Forward play/pause button events from zbus to GATT subscribers. */
static void write_button_gatt(void)
{
	int ret;
	const struct zbus_channel *chan;

	while (1) {
		ret = zbus_sub_wait(&button_gatt_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		struct button_msg msg;

		ret = zbus_chan_read(chan, &msg, ZBUS_READ_TIMEOUT_MS);
		ERR_CHK(ret);

		/*ret = zbus_sub_wait_msg(&button_gatt_sub, &chan, &msg, K_FOREVER);
		ERR_CHK(ret);*/

		if (msg.button_pin == BUTTON_PLAY_PAUSE) {
			bt_send_button_state(msg.button_action);
		}

		STACK_USAGE_PRINT("button_msg_thread", &thread_data);
	}
}

/** @brief Start forwarding button events to the GATT service. */
int init_button_service() {
    int ret;

	thread_id = k_thread_create(
		&thread_data, thread_stack,
		CONFIG_BUTTON_MSG_SUB_STACK_SIZE, (k_thread_entry_t)write_button_gatt, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_BUTTON_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);

	ret = k_thread_name_set(thread_id, "BUTTON_GATT_SUB");
	if (ret) {
		LOG_ERR("Failed to create button_msg thread");
		return ret;
	}

    ret = zbus_chan_add_obs(&button_chan, &button_gatt_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add button sub");
		return ret;
	}

    return 0;
}
