//#include "SensorManager.h"
#include "macros_common.h"
#include "openearable_common.h"

#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor_pub, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);

K_MSGQ_DEFINE(sensor_queue, sizeof(struct sensor_data), 16, 4);

ZBUS_CHAN_DEFINE(sensor_chan, struct sensor_data, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

struct sensor_data msg;

void sensor_chan_update() {
    int ret;

	while (1) {
		k_msgq_get(&sensor_queue, &msg, K_FOREVER);

		ret = zbus_chan_pub(&sensor_chan, &msg, K_FOREVER); //K_NO_WAIT
		if (ret) {
			LOG_ERR("Failed to publish sensor msg, ret: %d", ret);
		}
	}
}

K_THREAD_DEFINE(sensor_publish, 1024, sensor_chan_update, NULL, NULL, //CONFIG_BUTTON_PUBLISH_STACK_SIZE
		NULL, K_PRIO_PREEMPT(7), 0, 0); //CONFIG_BUTTON_PUBLISH_THREAD_PRIO