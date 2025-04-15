#include "button_manager.h"

#include "openearable_common.h"
#include "zbus_common.h"

#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(button, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);

K_MSGQ_DEFINE(button_queue, sizeof(struct button_msg), 1, 4);

ZBUS_CHAN_DEFINE(button_chan, struct button_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

void button_pub_task() {
    int ret;
	struct button_msg msg;

	while (1) {
		k_msgq_get(&button_queue, &msg, K_FOREVER);

		ret = zbus_chan_pub(&button_chan, &msg, K_FOREVER); //K_NO_WAIT
		if (ret) {
			LOG_ERR("Failed to publish button msg, ret: %d", ret);
		}
	}
}

K_THREAD_DEFINE(button_publish, CONFIG_BUTTON_PUBLISH_STACK_SIZE, button_pub_task, NULL, NULL,
		NULL, K_PRIO_PREEMPT(CONFIG_BUTTON_PUBLISH_THREAD_PRIO), 0, 0);