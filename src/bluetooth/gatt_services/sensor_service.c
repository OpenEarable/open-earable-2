#include "sensor_service.h"
#include "sensor_transport.h"
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>
#include "../SensorManager/SensorManager.h"
#include "../ParseInfo/SensorScheme.h"

#include "macros_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor_manager, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);

#define MAX_SENSOR_REC_NAME_LENGTH 64
#define MAX_NOTIFIES_IN_FLIGHT 4
#define BATCH_LATENCY_MS 20
BUILD_ASSERT(MAX_NOTIFIES_IN_FLIGHT <= CONFIG_BT_ATT_TX_COUNT);
static atomic_t stream_epoch;
static struct bt_conn *sensor_conn;
struct queued_sensor { struct sensor_data data; uint32_t epoch; };

static struct k_thread thread_data_notify;

static k_tid_t thread_id_notify;

ZBUS_SUBSCRIBER_DEFINE(sensor_gatt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);

ZBUS_CHAN_DECLARE(sensor_chan);
ZBUS_CHAN_DECLARE(bt_mgmt_chan);

static K_THREAD_STACK_DEFINE(thread_stack_notify, CONFIG_SENSOR_GATT_NOTIFY_STACK_SIZE);

K_MSGQ_DEFINE(gatt_queue, sizeof(struct queued_sensor), CONFIG_SENSOR_GATT_SUB_QUEUE_SIZE, 4);

static struct sensor_data sensor_data_value;
static struct sensor_config config;

static bool notify_enabled = false;
static bool sensor_config_status_ntfy_enabled = false;
static struct k_spinlock notify_state_lock;

void set_sensor_recording_name(const char *name);
static char sensor_recording_name[MAX_SENSOR_REC_NAME_LENGTH] = "sensor_log_";

static struct sensor_config *active_sensor_configs;
static size_t active_sensor_configs_size = 0;
static void notify_complete(struct bt_conn *conn, void *user_data);

static void connect_evt_handler(const struct zbus_channel *chan);
ZBUS_LISTENER_DEFINE(bt_mgmt_evt_listen2, connect_evt_handler); //static

void sensor_queue_listener_cb(const struct zbus_channel *chan);
ZBUS_LISTENER_DEFINE(sensor_queue_listener, sensor_queue_listener_cb);

static bool connection_complete = false;

/**
 * @brief Tracks one in-flight GATT notification and its dedicated payload copy.
 *
 * @details The Bluetooth stack completes notifications asynchronously, so each
 * pending notification needs stable storage until the completion callback runs.
 */
enum sensor_notify_context_state {
	SENSOR_NOTIFY_CONTEXT_FREE = 0,
	SENSOR_NOTIFY_CONTEXT_RESERVED,
	SENSOR_NOTIFY_CONTEXT_IN_FLIGHT,
};

struct sensor_notify_context {
	struct bt_gatt_notify_params params;
	struct oe_sensor_batch payload;
	enum sensor_notify_context_state state;
	uint32_t generation;
};

static int notify_count = 0;
static struct sensor_notify_context notify_contexts[MAX_NOTIFIES_IN_FLIGHT];

/**
 * @brief Disable new sensor notifications and drop queued payloads.
 *
 * @details Reserved or already queued notifications are left to retire through
 * the normal send-failure or completion paths so that slot ownership remains
 * well defined across disconnect races.
 */
static void reset_sensor_notification_state(void)
{
	k_spinlock_key_t key = k_spin_lock(&notify_state_lock);

	notify_enabled = false;
	sensor_config_status_ntfy_enabled = false;
	connection_complete = false;
	struct bt_conn *old = sensor_conn;
	sensor_conn = NULL;
	atomic_inc(&stream_epoch);
	k_spin_unlock(&notify_state_lock, key);

	if (old) bt_conn_unref(old);
	k_msgq_purge(&gatt_queue);
}

/**
 * @brief Reserve a context for the next asynchronous notification.
 *
 * @param[in] data Payload to copy into the reserved notification context.
 * @retval Pointer to a free notification context.
 * @retval NULL No context is currently available.
 */
static struct sensor_notify_context *acquire_notify_context(const struct oe_sensor_batch *data,
							    uint32_t *generation, uint32_t epoch);

/**
 * @brief Mark a reserved context as queued in the Bluetooth stack.
 *
 * @param[in] context Context to update.
 * @param[in] generation Reservation generation owned by the caller.
 */
static void mark_notify_context_in_flight(struct sensor_notify_context *context, uint32_t generation)
{
	k_spinlock_key_t key;

	if (context == NULL) {
		return;
	}

	key = k_spin_lock(&notify_state_lock);
	if (context->generation == generation &&
	    context->state == SENSOR_NOTIFY_CONTEXT_RESERVED) {
		context->state = SENSOR_NOTIFY_CONTEXT_IN_FLIGHT;
	}
	k_spin_unlock(&notify_state_lock, key);
}

/**
 * @brief Release a reserved or completed notification context.
 *
 * @param[in] context Context to release. May be NULL.
 * @param[in] generation Reservation generation owned by the caller.
 */
static void release_notify_context(struct sensor_notify_context *context, uint32_t generation)
{
	k_spinlock_key_t key;

	if (context == NULL) {
		return;
	}

	key = k_spin_lock(&notify_state_lock);
	if (context->generation != generation ||
	    context->state == SENSOR_NOTIFY_CONTEXT_FREE) {
		k_spin_unlock(&notify_state_lock, key);
		return;
	}

	context->state = SENSOR_NOTIFY_CONTEXT_FREE;

	if (notify_count > 0) {
		notify_count--;
	} else {
		LOG_WRN("Notify count went below zero!");
		notify_count = 0;
	}
	k_spin_unlock(&notify_state_lock, key);
}

static void connect_evt_handler(const struct zbus_channel *chan)
{
	const struct bt_mgmt_msg *msg;

	msg = zbus_chan_const_msg(chan);

	switch (msg->event) {
	case BT_MGMT_CONNECTED:
	{
		k_spinlock_key_t key = k_spin_lock(&notify_state_lock);
		sensor_conn = bt_conn_ref(msg->conn);
		connection_complete = true;
		k_spin_unlock(&notify_state_lock, key);
		break;
	}

	case BT_MGMT_DISCONNECTED:
		reset_sensor_notification_state();
		break;

	default:
		/* Other events do not affect sensor notification state. */
		break;
	}
}

static void sensor_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	ARG_UNUSED(attr);
	k_spinlock_key_t key = k_spin_lock(&notify_state_lock);

	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	atomic_inc(&stream_epoch);
	k_spin_unlock(&notify_state_lock, key);

	LOG_INF("Sensor data notifications %s", notify_enabled ? "enabled" : "disabled");

	k_msgq_purge(&gatt_queue);
}

static void sensor_config_status_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	ARG_UNUSED(attr);
	k_spinlock_key_t key = k_spin_lock(&notify_state_lock);
	sensor_config_status_ntfy_enabled = (value == BT_GATT_CCC_NOTIFY);
	k_spin_unlock(&notify_state_lock, key);
}

static ssize_t write_config(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(flags);
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != sizeof(struct sensor_config)) {
		LOG_WRN("Write sensor config: Incorrect data length: Expected %i but got %i", sizeof(struct sensor_config), len);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0) {
		LOG_WRN("Write sensor config: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

struct sensor_config * sensor_configuration = (struct sensor_config *)buf;

	if (sensor_configuration->storageOptions == 0) {
		LOG_INF("Setup sensor ID %i (turned off)", sensor_configuration->sensorId);
	} else {
		LOG_INF("Setup sensor ID %i with samplerateIndex %i", sensor_configuration->sensorId, sensor_configuration->sampleRateIndex);
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
	ARG_UNUSED(offset);
	ARG_UNUSED(flags);
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
			NULL, NULL, &sensor_data_value),
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

static struct sensor_notify_context *acquire_notify_context(const struct oe_sensor_batch *data,
							    uint32_t *generation, uint32_t epoch)
{
	k_spinlock_key_t key = k_spin_lock(&notify_state_lock);

	if (!connection_complete || !notify_enabled || epoch != (uint32_t)atomic_get(&stream_epoch) || notify_count >= MAX_NOTIFIES_IN_FLIGHT) {
		k_spin_unlock(&notify_state_lock, key);
		return NULL;
	}

	for (size_t i = 0; i < ARRAY_SIZE(notify_contexts); i++) {
		if (notify_contexts[i].state == SENSOR_NOTIFY_CONTEXT_FREE) {
			notify_contexts[i].state = SENSOR_NOTIFY_CONTEXT_RESERVED;
			notify_contexts[i].generation++;
			notify_contexts[i].payload = *data;
			notify_contexts[i].params.attr = &sensor_service.attrs[4];
			notify_contexts[i].params.data = notify_contexts[i].payload.data;
			notify_contexts[i].params.func = notify_complete;
			notify_contexts[i].params.user_data = &notify_contexts[i];
			notify_count++;
			*generation = notify_contexts[i].generation;
			k_spin_unlock(&notify_state_lock, key);
			return &notify_contexts[i];
		}
	}

	k_spin_unlock(&notify_state_lock, key);
	return NULL;
}

/**
 * @brief Release notification resources after the Bluetooth stack completes a send.
 */
static void notify_complete(struct bt_conn *conn, void *user_data)
{
	struct sensor_notify_context *context = (struct sensor_notify_context *)user_data;
	uint32_t generation;

	ARG_UNUSED(conn);

	if (context == NULL) {
		return;
	}

	generation = context->generation;
	release_notify_context(context, generation);
}

/* A connection reference and epoch keep queued data out of a later session. */
static struct bt_conn *stream_connection(uint32_t epoch)
{
    k_spinlock_key_t key = k_spin_lock(&notify_state_lock);
    struct bt_conn *conn = NULL;
    if (connection_complete && notify_enabled && sensor_conn &&
        epoch == (uint32_t)atomic_get(&stream_epoch)) {
        conn = bt_conn_ref(sensor_conn);
    }
    k_spin_unlock(&notify_state_lock, key);
    return conn;
}

static void send_batch(struct oe_sensor_batch *batch, uint32_t epoch)
{
    if (!batch->count) return;
    while (1) {
        struct bt_conn *conn = stream_connection(epoch);
        if (!conn) break;
        uint32_t generation;
        struct sensor_notify_context *context = acquire_notify_context(batch, &generation, epoch);
        if (!context) {
            bt_conn_unref(conn);
            k_sleep(K_MSEC(1));
            continue;
        }
        context->params.len = batch->len;
        int ret = bt_gatt_notify_cb(conn, &context->params);
        bt_conn_unref(conn);
        if (ret) {
            LOG_WRN("Failed to send data: %d.\n", ret);
            release_notify_context(context, generation);
        } else {
            mark_notify_context_in_flight(context, generation);
        }
        break;
    }
    batch->count = 0;
    batch->len = 0;
}

static void notification_task(void)
{
    static struct oe_sensor_batch batches[8];
    int64_t started[8] = {0};
    uint32_t epoch = atomic_get(&stream_epoch);
    unsigned next = 0;
    while (1) {
        if (epoch != (uint32_t)atomic_get(&stream_epoch)) {
            for (unsigned i = 0; i < 8; ++i) send_batch(&batches[i], epoch);
            epoch = atomic_get(&stream_epoch);
        }
        int64_t now = k_uptime_get();
        int64_t wait_ms = BATCH_LATENCY_MS;
        /* Rotate the first sensor serviced so a busy FIFO cannot monopolize TX. */
        for (unsigned n = 0; n < 8; ++n) {
            unsigned i = (next + n) % 8;
            if (!batches[i].count) continue;
            if (now - started[i] >= BATCH_LATENCY_MS) send_batch(&batches[i], epoch);
            else wait_ms = MIN(wait_ms, BATCH_LATENCY_MS - (now - started[i]));
        }
        next = (next + 1) % 8;
        struct queued_sensor item;
        if (k_msgq_get(&gatt_queue, &item, K_MSEC(wait_ms))) continue;
        const struct sensor_data *data = &item.data;
        unsigned id = data->id;
        if (id >= 8) continue;
        unsigned count = oe_sensor_sample_count(id, data->size);
        if (data->size > sizeof(data->data) || !count) {

            continue;
        }
        struct bt_conn *conn = stream_connection(item.epoch);
        if (!conn) {

            continue;
        }
        if (item.epoch != epoch) {
            for (unsigned n = 0; n < 8; ++n) send_batch(&batches[n], epoch);
            epoch = item.epoch;
        }
        unsigned limit = MIN(OE_SENSOR_PACKET_MAX, bt_gatt_get_mtu(conn) - 3);
        bt_conn_unref(conn);
        unsigned width = oe_sensor_sample_size(id);
        unsigned period = count > 1 ? sys_get_le16(data->data + data->size - 2) : 0;
        for (unsigned i = 0; i < count; ++i) {
            const uint8_t *sample = data->data + i * width;
            uint64_t time = data->time + (uint64_t)i * period;
            struct oe_sensor_batch *batch = &batches[id];
            if (!oe_sensor_batch_append(batch, id, sample, time, limit)) {
                send_batch(batch, epoch);
                if (!oe_sensor_batch_append(batch, id, sample, time, limit)) {

                    continue;
                }
            }
            if (batch->count == 1) started[id] = k_uptime_get();
        }
    }
}

void sensor_queue_listener_cb(const struct zbus_channel *chan)
{
    const struct sensor_msg *msg = zbus_chan_const_msg(chan);
    if (!msg->stream || msg->data.id >= 8) return;
    struct queued_sensor item = { .data = msg->data, .epoch = atomic_get(&stream_epoch) };
    struct bt_conn *conn = stream_connection(item.epoch);
    if (!conn) return;
    bt_conn_unref(conn);
    if (k_msgq_put(&gatt_queue, &item, K_NO_WAIT)) {
        /* Keep live sensor data fresh when the link cannot drain the queue. */
        struct queued_sensor discarded;
        (void)k_msgq_get(&gatt_queue, &discarded, K_NO_WAIT);
        (void)k_msgq_put(&gatt_queue, &item, K_NO_WAIT);
        LOG_WRN("ble sensor stream queue full");
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

int set_sensor_config_status(struct sensor_config sensor_configuration) {
	LOG_DBG("Setting sensor config status for sensorId: %i", sensor_configuration.sensorId);

	ssize_t sensor_config_index = -1;
	for (size_t i = 0; i < active_sensor_configs_size; i++) {
		if (active_sensor_configs[i].sensorId == sensor_configuration.sensorId) {
			sensor_config_index = i;
			break;
		}
	}

	if (sensor_config_index >= 0) {
		active_sensor_configs[sensor_config_index] = sensor_configuration;
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
		active_sensor_configs[active_sensor_configs_size - 1] = sensor_configuration;
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
