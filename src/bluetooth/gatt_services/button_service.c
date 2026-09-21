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

static void button_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t read_button_state(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &button_state,
					 sizeof(button_state));
}

BT_GATT_SERVICE_DEFINE(button_service,
BT_GATT_PRIMARY_SERVICE(BT_UUID_BUTTON),
BT_GATT_CHARACTERISTIC(BT_UUID_BUTTON_STATE,
            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
            BT_GATT_PERM_READ,
            read_button_state, NULL, &button_state),
BT_GATT_CCC(button_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

int bt_send_button_state(enum button_action _button_state)
{
	button_state = _button_state;

	if (!notify_enabled) {
		return -EACCES;
	}

	return bt_gatt_notify(NULL, &button_service.attrs[2], &button_state, sizeof(button_state));
}

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