#include "SensorManager.h"

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>

#include "macros_common.h"
#include "openearable_common.h"

#include <zephyr/zbus/zbus.h>

#include "IMU.h"
#include "Baro.h"
#include "PPG.h"
#include "Temp.h"
#include "BoneConduction.h"
#include "Microphone.h"

#include "openearable_common.h"
#include "StateIndicator.h"
#include "AutoOffManager.h"

#include <SensorScheme.h>
#include "../SD_Card/SDLogger/SDLogger.h"
#include <string>
#include <set>

#include "audio_datapath.h"

#include <sensor_service.h>

#include <zephyr/logging/log.h>
#include <sensor_service.h>
LOG_MODULE_DECLARE(sensor_manager);

std::set<int> ble_sensors = {};
std::set<int> sd_sensors = {};

//extern struct k_msgq sensor_queue;

EdgeMlSensor * get_sensor(enum sensor_id id);

static sensor_manager_state _state;

// Larger records batch more FIFO samples. Keep the queue's RAM budget bounded
// so the audio runtime heap retains its required headroom.
K_MSGQ_DEFINE(sensor_queue, sizeof(struct sensor_msg), 128, 4);
K_MSGQ_DEFINE(config_queue, sizeof(struct sensor_config), 16, 4);

K_THREAD_STACK_DEFINE(sensor_work_q_stack, CONFIG_SENSOR_WORK_QUEUE_STACK_SIZE);

ZBUS_CHAN_DEFINE(sensor_chan, struct sensor_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

// Internal queue marker; consumed before publication, never written to BLE/SD.
static constexpr uint8_t SENSOR_QUEUE_BARRIER = UINT8_MAX;

struct sensor_msg msg;

struct k_thread sensor_publish;

static k_tid_t sensor_pub_id;

static struct k_work config_work;
static struct k_work_delayable sd_config_work;
// Profile writes arrived at most 92 ms apart in paired phone tests with music.
// Wait for a quiet window, leaving the old recording untouched in the meantime.
static constexpr int SD_CONFIG_QUIET_MS = 300;
static constexpr size_t SENSOR_CONFIG_SLOTS = 8;
static sensor_config pending_sd_configs[SENSOR_CONFIG_SLOTS];
static bool pending_sd_valid[SENSOR_CONFIG_SLOTS];
static atomic_t config_stopping;
static bool config_ready;
static struct k_work_q config_work_q;
K_THREAD_STACK_DEFINE(config_work_q_stack, CONFIG_SENSOR_CONFIG_STACK_SIZE);

struct k_work_q sensor_work_q;
// PPG uses I2C2; the other polled sensors share I2C3 and one worker.
struct k_work_q sensor_ppg_work_q;
K_THREAD_STACK_DEFINE(sensor_ppg_work_q_stack, CONFIG_SENSOR_WORK_QUEUE_STACK_SIZE);

K_THREAD_STACK_DEFINE(sensor_publish_thread_stack, CONFIG_SENSOR_PUB_STACK_SIZE);

int active_sensors = 0;
static const char sensor_manager_auto_off_token[] = "SensorManager";

static void config_work_handler(struct k_work *work);
static void sd_config_work_handler(struct k_work *work);

static void drain_sensor_queue() {
	if (_state == INIT) return;

	struct k_sem drained;
	k_sem_init(&drained, 0, 1);
	struct sensor_msg barrier = {};
	barrier.data.id = SENSOR_QUEUE_BARRIER;
	struct k_sem *completion = &drained;
	memcpy(barrier.data.data, &completion, sizeof(completion));
	// Each caller waits for its own marker, including an overlapping shutdown.
	k_msgq_put(&sensor_queue, &barrier, K_FOREVER);
	k_sem_take(&drained, K_FOREVER);
}

void sensor_chan_update(void *p1, void *p2, void *p3) {
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
    int ret;

	while (1) {
		ret = k_msgq_get(&sensor_queue, &msg, K_FOREVER);
		if (ret) {
			continue;
		}
		if (msg.data.id == SENSOR_QUEUE_BARRIER) {
			struct k_sem *completion;
			memcpy(&completion, msg.data.data, sizeof(completion));
			k_sem_give(completion);
			continue;
		}

		ret = zbus_chan_pub(&sensor_chan, &msg, K_FOREVER); //K_NO_WAIT
		if (ret) {
			LOG_ERR("Failed to publish sensor msg, ret: %d", ret);
		}
	}
}

void init_sensor_manager() {
	_state = INIT;

	active_sensors = 0;

	k_work_queue_init(&sensor_work_q);
	k_work_queue_init(&sensor_ppg_work_q);
	k_work_queue_start(&sensor_ppg_work_q, sensor_ppg_work_q_stack,
        K_THREAD_STACK_SIZEOF(sensor_ppg_work_q_stack),
        K_PRIO_PREEMPT(CONFIG_SENSOR_WORK_QUEUE_PRIO), NULL);
	k_thread_name_set(&sensor_ppg_work_q.thread, "sensor_ppg");

	k_work_queue_start(&sensor_work_q, sensor_work_q_stack,
                   K_THREAD_STACK_SIZEOF(sensor_work_q_stack), K_PRIO_PREEMPT(CONFIG_SENSOR_WORK_QUEUE_PRIO),
                   NULL);

	sensor_pub_id = k_thread_create(&sensor_publish, sensor_publish_thread_stack, CONFIG_SENSOR_PUB_STACK_SIZE,
		sensor_chan_update, NULL, NULL, NULL,
			K_PRIO_PREEMPT(CONFIG_SENSOR_PUB_THREAD_PRIO), 0, K_FOREVER);  // Thread ist initial suspendiert

	k_work_init(&config_work, config_work_handler);
	k_work_init_delayable(&sd_config_work, sd_config_work_handler);
	/* Driver initialization can take hundreds of milliseconds. Keep it
	 * preemptible by audio decoding instead of using the cooperative system
	 * queue. It must also be separate from the polling queue, which sensor
	 * shutdown drains synchronously.
	 */
	k_work_queue_init(&config_work_q);
	k_work_queue_start(&config_work_q, config_work_q_stack,
		K_THREAD_STACK_SIZEOF(config_work_q_stack),
		K_PRIO_PREEMPT(CONFIG_SENSOR_WORK_QUEUE_PRIO), NULL);
	k_thread_name_set(&config_work_q.thread, "sensor_config");
	config_ready = true;

	sdlogger.init();

	int ret = auto_off_manager.register_participant(
		sensor_manager_auto_off_token,
		(power_saving_level_t)CONFIG_POWER_SAVING_LEVEL_SENSOR_MANAGER);
	if (ret && ret != -EALREADY) {
		LOG_WRN("Failed to register SensorManager with auto-off: %d", ret);
	} else {
		auto_off_manager.allow(sensor_manager_auto_off_token);
	}
}

void start_sensor_manager() {
	if (_state == RUNNING) return;

	LOG_DBG("Starting sensor manager");

	k_work_queue_unplug(&sensor_work_q);
	k_work_queue_unplug(&sensor_ppg_work_q);

	ble_sensors.clear();
	sd_sensors.clear();

	if (_state == INIT) {
		k_thread_start(sensor_pub_id);
	}

	_state = RUNNING;
}

void stop_sensor_manager() {
	if (config_ready && k_current_get() != &config_work_q.thread) {
		// Power-down must not leave a delayed profile that restarts acquisition.
		atomic_set(&config_stopping, 1);
		struct k_work_sync sync;
		k_work_cancel_delayable_sync(&sd_config_work, &sync);
		k_work_flush(&config_work, &sync);
		k_work_cancel_delayable_sync(&sd_config_work, &sync);
		memset(pending_sd_valid, 0, sizeof(pending_sd_valid));
	}
	if (_state != RUNNING) return;

	LOG_DBG("Stopping sensor manager");

	// Stop audio recording/processing first to prevent race condition
	//extern "C" void audio_datapath_stop_recording(void);
	audio_datapath_stop_recording();

    Baro::sensor.stop();
	IMU::sensor.stop();
	PPG::sensor.stop();
	Temp::sensor.stop();
	BoneConduction::sensor.stop();
	Microphone::sensor.stop();

	active_sensors = 0;
	auto_off_manager.allow(sensor_manager_auto_off_token);

	k_work_queue_drain(&sensor_work_q, true);
	k_work_queue_drain(&sensor_ppg_work_q, true);

	// Producers are stopped; let every accepted message reach the SD listener.
	drain_sensor_queue();

	_state = SUSPENDED;

	// End SDLogger and close current log file
	sdlogger.end();

	//k_msgq_purge(&config_queue);
}

EdgeMlSensor * get_sensor(enum sensor_id id) {
	switch (id) {
	case ID_IMU:
		return &(IMU::sensor);
	case ID_TEMP_BARO:
		return &(Baro::sensor);
	case ID_PPG:
		return &(PPG::sensor);
	case ID_OPTTEMP:
		return &(Temp::sensor);
	case ID_BONE_CONDUCTION:
		return &(BoneConduction::sensor);
	case ID_MICRO:
		return &(Microphone::sensor);
	default:
		return NULL;
	}
}

// Apply one request; the worker below drains all requests, since k_work
// submissions coalesce while an earlier sensor reconfiguration is running.
static void apply_sensor_config(const struct sensor_config &config, bool stop_idle = true) {
    float sampleRate = getSampleRateForSensorId(config.sensorId, config.sampleRateIndex);
	if (sampleRate <= 0) {
		LOG_ERR("Invalid sample rate %f for sensor %i", (double)sampleRate, config.sensorId);
		return;
	}

	EdgeMlSensor * sensor = get_sensor((enum sensor_id) config.sensorId);

	if (sensor == NULL) {
		LOG_ERR("Sensor not found for ID %i", config.sensorId);
		return;
	}

	struct sensor_config previous;
	if (sensor->is_running() &&
		get_sensor_config_status(config.sensorId, &previous) == 0 &&
		previous.sampleRateIndex == config.sampleRateIndex &&
		(previous.storageOptions & DATA_STORAGE) == (config.storageOptions & DATA_STORAGE) &&
		(config.storageOptions & (DATA_STORAGE | DATA_STREAMING))) {
		// Preserve the FIFO, sample clock and in-flight SD data for BLE-only
		// routing changes or repeated configurations. SD changes still drain
		// the stopped producer through the normal path below.
		sensor->ble_stream(config.storageOptions & DATA_STREAMING);
		if (config.storageOptions & DATA_STREAMING) ble_sensors.insert(config.sensorId);
		else ble_sensors.erase(config.sensorId);
		set_sensor_config_status(config);
		return;
	}

	if (sensor->is_running()) {
		sensor->stop();
		active_sensors--;

		if (active_sensors < 0) {
			LOG_WRN("Active sensors is already 0");
			active_sensors = 0;
		}
	}

	sensor->sd_logging(config.storageOptions & DATA_STORAGE);
	sensor->ble_stream(config.storageOptions & DATA_STREAMING);

	if (config.storageOptions & (DATA_STORAGE | DATA_STREAMING)) {
		if (sensor->init(&sensor_queue)) {
			if (active_sensors == 0) start_sensor_manager();
			sensor->start(config.sampleRateIndex);
			if (sensor->is_running()) {
				active_sensors++;
				auto_off_manager.prohibit(sensor_manager_auto_off_token);
			}
		}
	}

	if (config.storageOptions & DATA_STORAGE) {
		sd_sensors.insert(config.sensorId);
	} else if (sd_sensors.find(config.sensorId) != sd_sensors.end()) {
		sd_sensors.erase(config.sensorId);
	}

	if (config.storageOptions & DATA_STREAMING) ble_sensors.insert(config.sensorId);
	else if (ble_sensors.find(config.sensorId) != ble_sensors.end()) {
		ble_sensors.erase(config.sensorId);

		// TODO: if (ble_sensors.empty()) ...
	}

	set_sensor_config_status(config);

	if (stop_idle && active_sensors == 0) stop_sensor_manager();
}

static bool sd_configuration_changed(const sensor_config &before, const sensor_config &after) {
	return ((before.storageOptions ^ after.storageOptions) & DATA_STORAGE) ||
		((after.storageOptions & DATA_STORAGE) && before.sampleRateIndex != after.sampleRateIndex);
}

static void sd_config_work_handler(struct k_work *work) {
	ARG_UNUSED(work);
	if (atomic_get(&config_stopping)) return;
	// A received write can still be queued behind this delayed work item.
	if (k_msgq_num_used_get(&config_queue)) {
		k_work_reschedule_for_queue(&config_work_q, &sd_config_work, K_MSEC(SD_CONFIG_QUIET_MS));
		return;
	}

	sensor_config previous[SENSOR_CONFIG_SLOTS] = {};
	sensor_config desired[SENSOR_CONFIG_SLOTS] = {};
	bool apply[SENSOR_CONFIG_SLOTS] = {};
	const ParseInfoScheme *scheme = getParseInfoScheme();
	if (!scheme || scheme->sensorCount > SENSOR_CONFIG_SLOTS) return;
	bool rotate = false;
	bool record = false;
	for (size_t i = 0; i < scheme->sensorCount; i++) {
		const uint8_t id = scheme->sensorIds[i];
		if (id >= SENSOR_CONFIG_SLOTS || get_sensor_config_status(id, &previous[i])) return;
		desired[i] = pending_sd_valid[id] ? pending_sd_configs[id] : previous[i];
		apply[i] = pending_sd_valid[id];
		rotate |= sd_configuration_changed(previous[i], desired[i]);
		record |= (desired[i].storageOptions & DATA_STORAGE) != 0;
	}
	memset(pending_sd_valid, 0, sizeof(pending_sd_valid));
	const bool was_recording = sdlogger.is_active();
	rotate |= record && !was_recording;
	int ret = 0;
	if (rotate && was_recording) {
		// Stop all SD producers before crossing the file boundary. BLE-only
		// producers and playback continue; accepted samples are never purged.
		for (size_t i = 0; i < scheme->sensorCount; i++) {
			if (!(previous[i].storageOptions & DATA_STORAGE)) continue;
			EdgeMlSensor *sensor = get_sensor((sensor_id)previous[i].sensorId);
			if (sensor->is_running()) {
				sensor->stop();
				active_sensors--;
			}
			apply[i] = true;
		}
		drain_sensor_queue();
		ret = sdlogger.end();
		sd_sensors.clear();
	}
	if (rotate && record && ret == 0) {
		const std::string filename = get_sensor_recording_name() + std::to_string(micros());
		ret = sdlogger.begin(filename, desired, scheme->sensorCount);
	}
	if (ret != 0) {
		LOG_ERR("Failed to prepare SD configuration: %d", ret);
		// A rejected initial start leaves the working configuration unchanged.
		// If an existing file was closed, resume its BLE streams with SD off.
		if (was_recording) {
			for (size_t i = 0; i < scheme->sensorCount; i++) {
				if (!(previous[i].storageOptions & DATA_STORAGE)) continue;
				previous[i].storageOptions &= ~DATA_STORAGE;
				apply_sensor_config(previous[i], false);
			}
			if (active_sensors == 0) stop_sensor_manager();
		}
		state_indicator.set_sd_state(SD_FAULT);
		notify_sensor_config_status();
		return;
	}
	for (size_t i = 0; i < scheme->sensorCount; i++) {
		if (apply[i]) apply_sensor_config(desired[i], false);
	}
	if (rotate) state_indicator.set_sd_state(record ? SD_RECORDING : SD_IDLE);
	if (active_sensors == 0) stop_sensor_manager();
}

static void config_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    struct sensor_config config;
    while (k_msgq_get(&config_queue, &config, K_NO_WAIT) == 0) {
        if (atomic_get(&config_stopping)) continue;
        sensor_config previous;
        if (config.sensorId >= SENSOR_CONFIG_SLOTS ||
            get_sensor_config_status(config.sensorId, &previous) ||
            getSampleRateForSensorId(config.sensorId, config.sampleRateIndex) <= 0) {
            notify_sensor_config_status();
            continue;
        }
        if (pending_sd_valid[config.sensorId] || sd_configuration_changed(previous, config) ||
            ((config.storageOptions & DATA_STORAGE) && !sdlogger.is_active())) {
            pending_sd_configs[config.sensorId] = config;
            pending_sd_valid[config.sensorId] = true;
            k_work_reschedule_for_queue(&config_work_q, &sd_config_work, K_MSEC(SD_CONFIG_QUIET_MS));
        } else {
            apply_sensor_config(config);
        }
    }
}

void config_sensor(struct sensor_config * config) {
	if (atomic_get(&config_stopping)) return;
	int ret = k_msgq_put(&config_queue, config, K_NO_WAIT);
	if (ret) {
		LOG_ERR("Failed to put config in queue, ret: %d", ret);
		return;
	}

	k_work_submit_to_queue(&config_work_q, &config_work);
}
