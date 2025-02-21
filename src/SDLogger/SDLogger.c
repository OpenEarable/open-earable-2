#include "sensor_service.h"
#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>
#include "../SensorManager/SensorManager.h"

#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sd_logger, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);

static struct k_thread thread_data;
static k_tid_t thread_id;

static K_THREAD_STACK_DEFINE(thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE * 4);

ZBUS_SUBSCRIBER_DEFINE(sensor_sd_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);
ZBUS_CHAN_DECLARE(sensor_chan);

static struct sensor_msg msg;
static struct sensor_data * const data_buf = &(msg.data);

static void sd_work_handler(struct k_work * work);

K_WORK_DEFINE(sd_sensor_work, sd_work_handler);

int write_sensor_data() {
	const uint16_t size = sizeof(data_buf->id) + sizeof(data_buf->size) + sizeof(data_buf->time) + data_buf->size; //sizeof(float)*6;

	// TODO: write the data

	return 0;
}

static void sd_work_handler(struct k_work * work) {
	//send_sensor_data();
	//if (connection_complete && notify_enabled) {
	if (msg.sd) {
		int ret = write_sensor_data();
		if (ret != 0) LOG_WRN("Failed to write data to sd_card.\n");
	}
	//}
}

static void sensor_sd_task(void)
{
	int ret;
	const struct zbus_channel *chan;

	while (1) {
		ret = zbus_sub_wait(&sensor_sd_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		ret = zbus_chan_read(chan, &msg, ZBUS_READ_TIMEOUT_MS);
		ERR_CHK(ret);

		if (msg.sd) {
			k_work_submit(&sd_sensor_work);
		}

		STACK_USAGE_PRINT("sensor_msg_thread", &thread_data);
	}
}

int init_sd_logger() {
	int ret;

	thread_id = k_thread_create(
		&thread_data, thread_stack,
		CONFIG_BUTTON_MSG_SUB_STACK_SIZE, (k_thread_entry_t)sensor_sd_task, NULL,
		NULL, NULL, K_PRIO_PREEMPT(4), 0, K_NO_WAIT);
	
	ret = k_thread_name_set(thread_id, "SENSOR_SD_SUB");
	if (ret) {
		LOG_ERR("Failed to create sensor_msg thread");
		return ret;
	}

    ret = zbus_chan_add_obs(&sensor_chan, &sensor_sd_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add sensor sub");
		return ret;
	}

    return 0;
}