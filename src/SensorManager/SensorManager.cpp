#include "SensorManager.h"

#include <zephyr/kernel.h>

#include "macros_common.h"
#include "openearable_common.h"

#include "IMU.h"
#include "Baro.h"
#include "PPG.h"
#include "Temp.h"
#include "BoneConduction.h"
#include "Microphone.h"

#include "openearable_common.h"
#include "StateIndicator.h"

#include <SensorScheme.h>
#include "../SD_Card/SDLogger/SDLogger.h"
#include <string>
#include <set>

#include <sensor_service.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(sensor_manager);

std::set<int> ble_sensors = {};
std::set<int> sd_sensors = {};

EdgeMlSensor * get_sensor(enum sensor_id id);

static sensor_manager_state _state;

K_MSGQ_DEFINE(config_queue, sizeof(struct sensor_config), 16, 4);

K_THREAD_STACK_DEFINE(sensor_work_q_stack, CONFIG_SENSOR_WORK_QUEUE_STACK_SIZE);

static struct k_poll_signal sensor_manager_sig;
static struct k_poll_event sensor_manager_evt =
		 K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &sensor_manager_sig);

static struct k_work config_work;

struct k_work_q sensor_work_q;

int active_sensors = 0;

static void config_work_handler(struct k_work *work);

void init_sensor_manager() {
	_state = INIT;

	active_sensors = 0;

	k_work_queue_init(&sensor_work_q);
	k_work_queue_start(&sensor_work_q, sensor_work_q_stack,
                   K_THREAD_STACK_SIZEOF(sensor_work_q_stack), K_PRIO_PREEMPT(CONFIG_SENSOR_WORK_QUEUE_PRIO),
                   NULL);

	k_work_init(&config_work, config_work_handler);

	k_poll_signal_init(&sensor_manager_sig);

	sdlogger.init();
}

void start_sensor_manager() {
	if (_state == RUNNING) return;

	LOG_DBG("Starting sensor manager");

	k_work_queue_unplug(&sensor_work_q);

	ble_sensors.clear();
	sd_sensors.clear();

	k_poll_signal_raise(&sensor_manager_sig, 0);

	_state = RUNNING;
}

void stop_sensor_manager() {
	if (_state != RUNNING) return;

	LOG_DBG("Stopping sensor manager");

    Baro::sensor.stop();
	IMU::sensor.stop();
	PPG::sensor.stop();
	Temp::sensor.stop();
	BoneConduction::sensor.stop();
	Microphone::sensor.stop();

	active_sensors = 0;

	k_work_queue_drain(&sensor_work_q, true);

	k_poll_signal_reset(&sensor_manager_sig);

	_state = SUSPENDED;

	sdlogger.end();
	sd_sensors.clear();
	state_indicator.set_sd_state(SD_IDLE);
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

static void process_sensor_config(struct sensor_config &config) {
	float sampleRate = getSampleRateForSensorId(config.sensorId, config.sampleRateIndex);
	if (sampleRate <= 0) {
		LOG_ERR("Invalid sample rate %f for sensor %i", sampleRate, config.sensorId);
		return;
	}

	EdgeMlSensor * sensor = get_sensor((enum sensor_id) config.sensorId);

	if (sensor == NULL) {
		LOG_ERR("Sensor not found for ID %i", config.sensorId);
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

	if (config.storageOptions & DATA_STORAGE) {
		sd_sensors.insert(config.sensorId);

		if (!sdlogger.is_active()) {
			const char *recording_name_prefix = get_sensor_recording_name();
			LOG_INF("Starting SDLogger with recording name prefix: %s", recording_name_prefix);
			std::string filename = recording_name_prefix + std::to_string(micros());
			int ret = sdlogger.begin(filename);
			if (ret == 0) state_indicator.set_sd_state(SD_RECORDING);
		}
	} else if (sd_sensors.find(config.sensorId) != sd_sensors.end()) {
		sd_sensors.erase(config.sensorId);

		if (sd_sensors.empty()) {
			sdlogger.end();
			state_indicator.set_sd_state(SD_IDLE);
		}
	}

	if (config.storageOptions & (DATA_STORAGE | DATA_STREAMING)) {
		if (sensor->init()) {
			if (active_sensors == 0) start_sensor_manager();
			sensor->start(config.sampleRateIndex);
			if (sensor->is_running()) {
				active_sensors++;
			}
		}
	}

	if (config.storageOptions & DATA_STREAMING) ble_sensors.insert(config.sensorId);
	else if (ble_sensors.find(config.sensorId) != ble_sensors.end()) {
		ble_sensors.erase(config.sensorId);
	}

	set_sensor_config_status(config);

	if (active_sensors == 0) stop_sensor_manager();
}

static void config_work_handler(struct k_work *work) {
	struct sensor_config config;

	while (k_msgq_get(&config_queue, &config, K_NO_WAIT) == 0) {
		process_sensor_config(config);
	}
}

void config_sensor(struct sensor_config * config) {
	int ret = k_msgq_put(&config_queue, config, K_NO_WAIT);
	if (ret) {
		LOG_ERR("Failed to put config in queue, ret: %d", ret);
		return;
	}

	k_work_submit(&config_work);
}
