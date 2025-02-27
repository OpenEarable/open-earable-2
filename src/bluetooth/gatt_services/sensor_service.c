#include "sensor_service.h"
#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>
#include "../SensorManager/SensorManager.h"

#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor_manager, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);

static struct k_thread thread_data;
static k_tid_t thread_id;

ZBUS_SUBSCRIBER_DEFINE(sensor_gatt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);
//ZBUS_LISTENER_DEFINE()

ZBUS_CHAN_DECLARE(sensor_chan);
ZBUS_CHAN_DECLARE(bt_mgmt_chan);

static K_THREAD_STACK_DEFINE(thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE * 4);

//extern const k_tid_t sensor_publish;

//static struct sensor_data data;
static struct sensor_msg msg;
static struct sensor_data * const data_buf = &(msg.data);
static struct sensor_config config;

static bool notify_enabled = false;

//k_work gatt_sensor_work;

static void gatt_work_handler(struct k_work * work);
int send_sensor_data();

K_WORK_DEFINE(gatt_sensor_work, gatt_work_handler);

static void connect_evt_handler(const struct zbus_channel *chan);

ZBUS_LISTENER_DEFINE(bt_mgmt_evt_listen2, connect_evt_handler); //static

static bool connection_complete = false;

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
}

static ssize_t read_sensor_value(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	const uint16_t size = sizeof(data_buf->id) + sizeof(data_buf->size) + sizeof(data_buf->time) + data_buf->size;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, data_buf, size);
}

static ssize_t write_config(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_INF("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

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
            BT_GATT_CHRC_WRITE,
            NULL, write_config, &config),
BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_DATA,
            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
            BT_GATT_PERM_READ,
            read_sensor_value, NULL, data_buf),
BT_GATT_CCC(sensor_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void gatt_work_handler(struct k_work * work) {
	//send_sensor_data();
	if (connection_complete && notify_enabled && msg.stream) {
		int ret = send_sensor_data();
		//if (ret == 0) printk("data send\n");
		if (ret != 0) printk("Failed to send data.\n");
		//ERR_CHK(ret);
	}
}

int send_sensor_data() {
	if (!notify_enabled) {
		return -EACCES;
	}

	const uint16_t size = sizeof(data_buf->id) + sizeof(data_buf->size) + sizeof(data_buf->time) + data_buf->size; //sizeof(float)*6;

	return bt_gatt_notify(NULL, &sensor_service.attrs[4], data_buf, size);
}

static void sensor_gatt_task(void)
{
	int ret;
	const struct zbus_channel *chan;

	while (1) {
		ret = zbus_sub_wait(&sensor_gatt_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		ret = zbus_chan_read(chan, &msg, ZBUS_READ_TIMEOUT_MS);
		ERR_CHK(ret);

		if (msg.stream) {
			k_work_submit(&gatt_sensor_work);
		}

		STACK_USAGE_PRINT("sensor_msg_thread", &thread_data);
	}
}

int init_sensor_service() {
	int ret;

	thread_id = k_thread_create(
		&thread_data, thread_stack,
		CONFIG_BUTTON_MSG_SUB_STACK_SIZE, (k_thread_entry_t)sensor_gatt_task, NULL,
		NULL, NULL, K_PRIO_PREEMPT(4), 0, K_NO_WAIT);
	
	ret = k_thread_name_set(thread_id, "SENSOR_GATT_SUB");
	if (ret) {
		LOG_ERR("Failed to create sensor_msg thread");
		return ret;
	}

    ret = zbus_chan_add_obs(&sensor_chan, &sensor_gatt_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add sensor sub");
		return ret;
	}

	ret = zbus_chan_add_obs(&bt_mgmt_chan, &bt_mgmt_evt_listen2, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add bt_mgmt listener");
		return ret;
	}

    return 0;
}