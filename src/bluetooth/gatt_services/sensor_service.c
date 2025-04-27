#include "sensor_service.h"
#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>
#include "../SensorManager/SensorManager.h"

#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor_manager, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);

static struct k_thread thread_data_notify;

static k_tid_t thread_id_notify;

ZBUS_SUBSCRIBER_DEFINE(sensor_gatt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);

ZBUS_CHAN_DECLARE(sensor_chan);
ZBUS_CHAN_DECLARE(bt_mgmt_chan);

static K_THREAD_STACK_DEFINE(thread_stack_notify, CONFIG_SENSOR_GATT_NOTIFY_STACK_SIZE);

K_MSGQ_DEFINE(gatt_queue, sizeof(struct sensor_data), CONFIG_SENSOR_GATT_SUB_QUEUE_SIZE, 4);

//static struct sensor_msg msg;
static struct sensor_data sensor_data;
static struct sensor_config config;

static bool notify_enabled = false;


static void connect_evt_handler(const struct zbus_channel *chan);
ZBUS_LISTENER_DEFINE(bt_mgmt_evt_listen2, connect_evt_handler); //static

void sensor_queue_listener_cb(const struct zbus_channel *chan);
ZBUS_LISTENER_DEFINE(sensor_queue_listener, sensor_queue_listener_cb);

static bool connection_complete = false;

static struct k_mutex notify_mutex;

static void connect_evt_handler(const struct zbus_channel *chan)
{
	const struct bt_mgmt_msg *msg;

	msg = zbus_chan_const_msg(chan);

	switch (msg->event) {
	case BT_MGMT_CONNECTED:
		connection_complete = true;
		break;

	case BT_MGMT_DISCONNECTED:
		connection_complete = false;
		break;
	}
}

static void sensor_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);

	k_msgq_purge(&gatt_queue);
}

static ssize_t read_sensor_value(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	const uint16_t size = sizeof(sensor_data.id) + sizeof(sensor_data.size) + sizeof(sensor_data.time) + sensor_data.size;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &sensor_data, size);
}

static ssize_t write_config(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != sizeof(struct sensor_config)) {
		LOG_WRN("Write sensor config: Incorrect data length: Expected %i but got %i", sizeof(struct sensor_config), len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		LOG_WRN("Write sensor config: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	LOG_INF("Setup sensor ID %i with samplerateIndex %i", ((struct sensor_config *)buf)->sensorId, ((struct sensor_config *)buf)->sampleRateIndex);

	//stop_sensor_manager();
	config_sensor((struct sensor_config *) buf);

	return len;
}

BT_GATT_SERVICE_DEFINE(sensor_service,
BT_GATT_PRIMARY_SERVICE(BT_UUID_SENSOR),
BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_CONFIG,
            BT_GATT_CHRC_WRITE,
            BT_GATT_PERM_WRITE,
            NULL, write_config, &config),
BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_DATA,
			BT_GATT_CHRC_NOTIFY,
			BT_GATT_PERM_NONE,
			NULL, NULL, &sensor_data),
BT_GATT_CCC(sensor_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void notify_complete() {
	k_mutex_unlock(&notify_mutex);
}

//static void gatt_work_handler(struct k_work * work) {
static void notification_task(void) {
	int ret;

	while (1) {
		ret = k_msgq_get(&gatt_queue, &sensor_data, K_FOREVER);

		if (ret != 0) {
			LOG_WRN("No data to process");
			continue;
		}

		if (connection_complete && notify_enabled) {
			const uint16_t size = sizeof(sensor_data.id) + sizeof(sensor_data.size) + sizeof(sensor_data.time) + sensor_data.size;

			static struct bt_gatt_notify_params params;
			params.attr = &sensor_service.attrs[4];
			params.data = &sensor_data;
			params.len = size;
			params.func = notify_complete;
			params.user_data = NULL;

			ret = k_mutex_lock(&notify_mutex, K_MSEC(100));
			if (ret != 0) {
				LOG_ERR("Unable to lock notify mutex.");
			}

			ret = bt_gatt_notify_cb(NULL, &params);
			if (ret != 0) {
				k_mutex_unlock(&notify_mutex);
				LOG_WRN("Failed to send data.\n");
			}
		}
	}
}

void sensor_queue_listener_cb(const struct zbus_channel *chan) {
	int ret;
	const struct sensor_msg * msg;
    
    msg = (struct sensor_msg *)zbus_chan_const_msg(&sensor_chan);

	if (msg->stream) {
		ret = k_msgq_put(&gatt_queue, &msg->data, K_NO_WAIT);

		if (ret) {
			LOG_WRN("ble sensor stream queue full");
		}
	}
}

int init_sensor_service() {
	int ret;

	thread_id_notify = k_thread_create(
		&thread_data_notify, thread_stack_notify,
		CONFIG_SENSOR_GATT_NOTIFY_STACK_SIZE, (k_thread_entry_t)notification_task, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_SENSOR_GATT_NOTIFY_THREAD_PRIO), 0, K_NO_WAIT);
	
	ret = k_thread_name_set(thread_id_notify, "SENSOR_GATT_NOTIFY");
	if (ret) {
		LOG_ERR("Failed to create sensor_msg thread");
		return ret;
	}

    ret = zbus_chan_add_obs(&sensor_chan, &sensor_queue_listener, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add sensor sub");
		return ret;
	}

	ret = zbus_chan_add_obs(&bt_mgmt_chan, &bt_mgmt_evt_listen2, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add bt_mgmt listener");
		return ret;
	}

	k_mutex_init(&notify_mutex);

    return 0;
}