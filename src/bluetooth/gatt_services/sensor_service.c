#include "sensor_service.h"
#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>
//#include <

#include "macros_common.h"
#include "nrf5340_audio_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor_manager, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);

static struct k_thread sensor_gatt_thread_data;

static k_tid_t sensor_gatt_thread_id;
ZBUS_SUBSCRIBER_DEFINE(sensor_gatt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);
//ZBUS_LISTENER_DEFINE()

ZBUS_CHAN_DECLARE(sensor_chan);

K_THREAD_STACK_DEFINE(sensor_gatt_thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE * 4);

//extern const k_tid_t sensor_publish;

static struct sensor_data data;
static struct sensor_config config;

static bool notify_sensor_enabled = false;

//k_work gatt_sensor_work;

static void gatt_work_handler(struct k_work * work);
int send_sensor_data();

K_WORK_DEFINE(gatt_sensor_work, gatt_work_handler);

static void sensor_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	notify_sensor_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t read_sensor_value(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	const uint16_t size = sizeof(data.id) + sizeof(data.size) + sizeof(data.time) + data.size;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &data, size);
}

static ssize_t write_config(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	printk("Attribute write, handle: %u, conn: %p", attr->handle,
		(void *)conn);

	if (len != sizeof(struct sensor_config)) {
		printk("Write sensor config: Incorrect data length");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		printk("Write sensor config: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	//SensorManager::manager.stop();
	//SensorManager::manager.config();

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
            read_sensor_value, NULL, &data),
BT_GATT_CCC(sensor_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void gatt_work_handler(struct k_work * work) {
	//send_sensor_data();
	if (notify_sensor_enabled) {
		int ret = send_sensor_data();
		//ERR_CHK(ret);
	}
}

int send_sensor_data() { //struct sensor_data * _data

	if (!notify_sensor_enabled) {
		return -EACCES;
	}

	//const uint16_t size = sizeof(_data->id) + sizeof(_data->size) + sizeof(_data->time) + _data->size;
	const uint16_t size = sizeof(data.id) + sizeof(data.size) + sizeof(data.time) + data.size; //sizeof(float)*6; 

	return bt_gatt_notify(NULL, &sensor_service.attrs[4], &data, size);
}

static void sensor_gatt_task(void)
{
	int ret;
	const struct zbus_channel *chan;

	float t_imu = 1000/50;
	float t_baro = 1000/20;
	int count = 0;

	const float alpha = 0.01;

	uint32_t time_last_imu = k_cyc_to_ms_floor32(k_cycle_get_32());
	uint32_t time_last_baro = time_last_imu;

	while (1) {
		ret = zbus_sub_wait(&sensor_gatt_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		ret = zbus_chan_read(chan, &data, ZBUS_READ_TIMEOUT_MS);
		ERR_CHK(ret);

		//ret = zbus_sub_wait_msg(&sensor_gatt_sub, &chan, &data, K_FOREVER);
		//ERR_CHK(ret);

		

		//printk("rec: %i\n", msg.id);

		/*switch (data.id)
		{
		case ID_TEMP_BARO:
			printk("temperature: %.3fC, ", data.data[0]);
			printk("pressure: %.3fhPa\n", data.data[1]/100);
			break;

		case ID_IMU:
			printk("acc: %.3f, %.3f, %.3f", data.data[0], data.data[1], data.data[2]);
			printk(" | gyro: %.3f, %.3f, %.3f", data.data[3], data.data[4], data.data[5]);
			printk(" | mag: %.3f, %.3f, %.3f\n", data.data[6], data.data[7], data.data[8]);
			break;
		
		default:
			break;
		}*/

		switch (data.id)
		{
		case ID_TEMP_BARO:
			t_baro = (data.time - time_last_baro) * alpha + t_baro * (1 - alpha);
			time_last_baro = data.time;
			break;

		case ID_IMU:
			t_imu = (data.time - time_last_imu) * alpha + t_imu * (1 - alpha);
			time_last_imu = data.time;
			break;
		
		default:
			break;
		}

		count ++;

		if (count >= 100) {
			count = 0;
			printk("imu: %.3f, baro: %.3f, enabled: %i\n", 1000 / t_imu, 1000 / t_baro, notify_sensor_enabled);
		}

		//if (notify_sensor_enabled) ret = send_sensor_data();
		//ERR_CHK(ret);

		k_work_submit(&gatt_sensor_work);
		
		//send_sensor_data();

		STACK_USAGE_PRINT("sensor_msg_thread", &sensor_gatt_thread_data);
	}
}

int init_sensor_service() {
	int ret;

	sensor_gatt_thread_id = k_thread_create(
		&sensor_gatt_thread_data, sensor_gatt_thread_stack,
		CONFIG_BUTTON_MSG_SUB_STACK_SIZE, (k_thread_entry_t)sensor_gatt_task, NULL,
		NULL, NULL, K_PRIO_PREEMPT(4), 0, K_NO_WAIT);
	
	ret = k_thread_name_set(sensor_gatt_thread_id, "SENSOR_GATT_SUB");
	if (ret) {
		LOG_ERR("Failed to create sensor_msg thread");
		return ret;
	}

    ret = zbus_chan_add_obs(&sensor_chan, &sensor_gatt_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add sensor sub");
		return ret;
	}

    return 0;
}