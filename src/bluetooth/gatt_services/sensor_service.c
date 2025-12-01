#include "sensor_service.h"
#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>
#include "../SensorManager/SensorManager.h"
#include "../ParseInfo/SensorScheme.h"

#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor_manager, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);

#define MAX_SENSOR_REC_NAME_LENGTH 64

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
static bool sensor_config_status_ntfy_enabled = false;

void set_sensor_recording_name(const char *name);
static char sensor_recording_name[MAX_SENSOR_REC_NAME_LENGTH] = "sensor_log_";

static struct sensor_config *active_sensor_configs;
static size_t active_sensor_configs_size = 0;

static void connect_evt_handler(const struct zbus_channel *chan);
ZBUS_LISTENER_DEFINE(bt_mgmt_evt_listen2, connect_evt_handler); //static

void sensor_queue_listener_cb(const struct zbus_channel *chan);
ZBUS_LISTENER_DEFINE(sensor_queue_listener, sensor_queue_listener_cb);

static bool connection_complete = false;

int notify_count = 0;

int MAX_NOTIFIES_IN_FLIGHT = 4;

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
		notify_enabled = false;
		k_msgq_purge(&gatt_queue);
		break;
	}
}

static void sensor_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);

	LOG_INF("Sensor data notifications %s", notify_enabled ? "enabled" : "disabled");

	k_msgq_purge(&gatt_queue);
}

static void sensor_config_status_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	sensor_config_status_ntfy_enabled = (value == BT_GATT_CCC_NOTIFY);
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

	struct sensor_config * config = (struct sensor_config *)buf;

	if (config->storageOptions == 0) {
		LOG_INF("Setup sensor ID %i (turned off)", config->sensorId);
	} else {
		LOG_INF("Setup sensor ID %i with samplerateIndex %i", config->sensorId, config->sampleRateIndex);
	}

	//stop_sensor_manager();
	config_sensor((struct sensor_config *) buf);

	return len;
}

static ssize_t read_sensor_rec_name(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	const char *name = get_sensor_recording_name();
	size_t name_len = strlen(name);

	if (offset > name_len) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, name, name_len);
}

static ssize_t write_sensor_rec_name(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, len: %u, handle: %u, conn: %p", len, attr->handle, (void *)conn);
	if (len > MAX_SENSOR_REC_NAME_LENGTH - 1) {
		LOG_WRN("Write sensor recording name: Data length exceeds maximum allowed length of %i", MAX_SENSOR_REC_NAME_LENGTH - 1);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	// Print buffer contents as a string (ensure null-termination for safety)
	char temp_buf[MAX_SENSOR_REC_NAME_LENGTH];
	strncpy(temp_buf, (const char *)buf, len);
	temp_buf[len] = '\0';

	LOG_DBG("Write sensor recording name: %s", temp_buf);

	set_sensor_recording_name(temp_buf);

	return len;
}

static ssize_t read_sensor_config_status(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	const uint16_t size = sizeof(struct sensor_config) * active_sensor_configs_size;
	LOG_DBG("Reading sensor config status");

	if (len < size) {
		LOG_WRN("Read sensor config status: Buffer too small: %u < %u", len, size);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, active_sensor_configs, size);
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
BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_CONFIG_STATUS,
			BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			BT_GATT_PERM_READ,
			read_sensor_config_status, NULL, &active_sensor_configs),
BT_GATT_CCC(sensor_config_status_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_RECORDING_NAME,
			BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			read_sensor_rec_name, write_sensor_rec_name, NULL),
);

static void notify_complete() {
	notify_count--;

	if (notify_count < 0) {
		notify_count = 0;
		LOG_WRN("Notify count went below zero!");
	}
}

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

			while(notify_count >= MAX_NOTIFIES_IN_FLIGHT) {
				k_yield(); // maybe replace with k_sleep?
			}

			notify_count++;

			ret = bt_gatt_notify_cb(NULL, &params);
			if (ret != 0) {
				LOG_WRN("Failed to send data: %d.\n", ret);
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

int init_sensor_config_status() {
	struct ParseInfoScheme *parse_info_scheme = getParseInfoScheme();

	// Initialize the active sensor configs list
	active_sensor_configs_size = parse_info_scheme->sensorCount;
	active_sensor_configs = k_malloc(sizeof(struct sensor_config) * active_sensor_configs_size);
	if (active_sensor_configs == NULL) {
		LOG_ERR("Failed to allocate memory for active sensor configs");
		return -1;
	}

	for (size_t i = 0; i < active_sensor_configs_size; i++) {
		struct SensorScheme *sensor_scheme = getSensorSchemeForId(parse_info_scheme->sensorIds[i]);
		LOG_DBG("Initializing sensor config state for sensor with id %d", sensor_scheme->id);

		active_sensor_configs[i].sensorId = sensor_scheme->id;
		if (sensor_scheme->configOptions.availableOptions & FREQUENCIES_DEFINED) {
			active_sensor_configs[i].sampleRateIndex = sensor_scheme->configOptions.frequencyOptions.defaultFrequencyIndex;
		} else {
			active_sensor_configs[i].sampleRateIndex = 0; // Default to 0 if frequencies are not defined
		}
		active_sensor_configs[i].storageOptions = 0; // Default storage options
	}

	LOG_DBG("Sensor config status initialized");
	return 0;
}

int set_sensor_config_status(struct sensor_config config) {
	LOG_DBG("Setting sensor config status for sensorId: %i", config.sensorId);

	ssize_t sensor_config_index = -1;
	for (size_t i = 0; i < active_sensor_configs_size; i++) {
		if (active_sensor_configs[i].sensorId == config.sensorId) {
			sensor_config_index = i;
			break;
		}
	}

	if (sensor_config_index >= 0) {
		active_sensor_configs[sensor_config_index] = config;
		LOG_DBG("Found sensor config");
	} else {
		LOG_DBG("Sensor config not found, adding new sensor config");
		// allocate more space for the new sensor config list
		active_sensor_configs_size++;
		struct sensor_config *new_active_sensor_configs = k_realloc(active_sensor_configs, active_sensor_configs_size);
		if (new_active_sensor_configs == NULL) {
			LOG_ERR("Failed to allocate memory for new sensor config");
			return -1;
		}
		active_sensor_configs = new_active_sensor_configs;
		active_sensor_configs[active_sensor_configs_size - 1] = config;
	}

	if (sensor_config_status_ntfy_enabled) {
		LOG_DBG("Sensor config status notification, notifying %zu active sensor configs", active_sensor_configs_size);
		struct bt_gatt_notify_params params = {
            .attr   = &sensor_service.attrs[7],
            .data   = active_sensor_configs,
            .len    = sizeof(struct sensor_config) * active_sensor_configs_size,
        };
        int ret = bt_gatt_notify_cb(NULL, &params);

		if (ret) {
			LOG_ERR("Failed to notify sensor config status, error code: %d", ret);
			return ret;
		}
	}

	return 0;
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

	init_sensor_config_status();

    return 0;
}

const char *get_sensor_recording_name() {
	return sensor_recording_name;
}

/**
 * @brief Set the sensor recording name object.
 * 
 * @param name A pointer to the name string.
 * Has to be a valid string with a length greater than 0
 * and 0 terminated.
 */
void set_sensor_recording_name(const char *name) {
	if (name == NULL || strlen(name) == 0) {
		LOG_WRN("Invalid sensor recording name");
		return;
	}

	strncpy(sensor_recording_name, name, sizeof(sensor_recording_name) - 1);
	sensor_recording_name[sizeof(sensor_recording_name) - 1] = '\0';
}
