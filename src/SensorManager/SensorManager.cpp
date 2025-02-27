#include "SensorManager.h"

#include <zephyr/kernel.h>

#include "macros_common.h"
#include "openearable_common.h"

#include <zephyr/zbus/zbus.h>

#include "IMU.h"
#include "Baro.h"
#include "PPG.h"
#include "Temp.h"
#include "BoneConduction.h"
#include <SensorScheme.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(sensor_manager);
#include "../SD_Card/SDLogger/SDLogger.hpp"
#include <string>

extern struct k_msgq sensor_queue;
// extern k_tid_t sensor_publish;

EdgeMlSensor * get_sensor(enum sensor_id id);

// Track number of active sensors
static int active_sensor_count = 0;

static sensor_manager_state _state;

K_MSGQ_DEFINE(sensor_queue, sizeof(struct sensor_msg), 16, 4);

K_MSGQ_DEFINE(config_queue, sizeof(struct sensor_config), 16, 4);

ZBUS_CHAN_DEFINE(sensor_chan, struct sensor_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

struct sensor_msg msg;

struct k_thread sensor_publish;

static k_tid_t sensor_pub_id;

static struct k_work config_work;

int active_sensors = 0;

static void config_work_handler(struct k_work *work);

void sensor_chan_update(void *p1, void *p2, void *p3) {
    int ret;

	while (1) {
		k_msgq_get(&sensor_queue, &msg, K_FOREVER);

		ret = zbus_chan_pub(&sensor_chan, &msg, K_FOREVER); //K_NO_WAIT
		if (ret) {
			LOG_ERR("Failed to publish sensor msg, ret: %d", ret);
		}
	}
}

K_THREAD_STACK_DEFINE(sensor_publish_thread_stack, 1024);

void init_sensor_manager() {
	_state = INIT;

	active_sensors = 0;

	sensor_pub_id = k_thread_create(&sensor_publish, sensor_publish_thread_stack, 1024,
		sensor_chan_update, NULL, NULL, NULL,
			K_PRIO_PREEMPT(CONFIG_SENSOR_THREAD_PRIO), 0, K_FOREVER);  // Thread ist initial suspendiert

	k_work_init(&config_work, config_work_handler);
}

void start_sensor_manager() {
	if (_state == RUNNING) return;

	//empty message queue
	k_msgq_purge(&sensor_queue);
	k_msgq_purge(&config_queue);

	if (_state == INIT) {
		k_thread_start(sensor_pub_id);
	} else {
		k_thread_resume(sensor_pub_id);
	}

	_state = RUNNING;

	// Start SDLogger with timestamp-based filename
	std::string filename = "sensor_log_" + std::to_string(k_uptime_get());
	//SDLogger::get_instance().begin(filename);
}

void stop_sensor_manager() {
	if (_state != RUNNING) return;

    Baro::sensor.stop();
	IMU::sensor.stop();
	PPG::sensor.stop();
	Temp::sensor.stop();
	BoneConduction::sensor.stop();

	active_sensor_count = 0;

	k_thread_suspend(sensor_pub_id);

	_state = SUSPENDED;

	// End SDLogger and close current log file
	//SDLogger::get_instance().end();
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
	default:
		return NULL;
	}
}

// Worker-Funktion für die Sensor-Konfiguration
static void config_work_handler(struct k_work *work) {
	struct sensor_config config;

	int ret = k_msgq_get(&config_queue, &config, K_NO_WAIT);

    float sampleRate = getSampleRateForSensor(config.sensorId, config.sampleRateIndex);
	if (sampleRate <= 0) {
		LOG_ERR("Invalid sample rate %f for sensor %i", sampleRate, config.sensorId);
		return;
	}

    k_timeout_t t = K_USEC(1e6 / sampleRate);

	EdgeMlSensor * sensor = get_sensor((enum sensor_id) config.sensorId);

	if (config.storageOptions == 0 || !(config.storageOptions & (DATA_STREAMING | DATA_STORAGE))) {
		if (sensor->is_running()) {
			active_sensors--;
			if (active_sensors < 0) {
				LOG_WRN("Active sensors is already 0");
				active_sensors = 0;
				stop_sensor_manager();
			}
			sensor->stop();
		}
		return;
	}

	if (sensor->is_running()) {
		sensor->stop();
		active_sensors--;
	}

	sensor->sd_logging(config.storageOptions & DATA_STORAGE);
	sensor->ble_stream(config.storageOptions & DATA_STREAMING);

	if (sensor->init(&sensor_queue)) {
		if (active_sensors == 0) start_sensor_manager();
		sensor->start(config.sampleRateIndex);
		if (sensor->is_running()) {
			active_sensors++;
		}
	}
}

void config_sensor(struct sensor_config * config) {
	int ret = k_msgq_put(&config_queue, config, K_NO_WAIT);
	k_work_submit(&config_work);
}