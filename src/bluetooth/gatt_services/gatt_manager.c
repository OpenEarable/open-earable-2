#include "gatt_manager.h"

#include "macros_common.h"

#include "button_assignments.h"
#include "button_service.h"

#include <zephyr/zbus/zbus.h>
#include "nrf5340_audio_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gatt_manager, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);

static struct k_thread button_gatt_thread_data;

static k_tid_t button_gatt_thread_id;
ZBUS_SUBSCRIBER_DEFINE(button_gatt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);

ZBUS_CHAN_DECLARE(button_chan);

K_THREAD_STACK_DEFINE(button_gatt_thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE);

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

		if (msg.button_pin == BUTTON_PLAY_PAUSE) {
			bt_send_button_state(msg.button_action);
		}

		STACK_USAGE_PRINT("button_msg_thread", &button_gatt_thread_data);
	}
}

int gatt_init() {
    int ret;

	button_gatt_thread_id = k_thread_create(
		&button_gatt_thread_data, button_gatt_thread_stack,
		CONFIG_BUTTON_MSG_SUB_STACK_SIZE, (k_thread_entry_t)write_button_gatt, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_BUTTON_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(button_gatt_thread_id, "BUTTON_GATT_SUB");
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