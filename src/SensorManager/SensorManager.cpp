#include "SensorManager.h"

#include <zephyr/kernel.h>

#include "IMU.h"
#include "Baro.h"
#include "PPG.h"
#include "Temp.h"
#include "BoneConduction.h"
#include "SensorScheme.h"

extern struct k_msgq sensor_queue;
extern k_tid_t sensor_publish;

EdgeMlSensor * get_sensor(enum sensor_id id);

//SensorManager SensorManager::manager = SensorManager();

void start_sensor_manager() {
	//empty message queue
	k_msgq_purge(&sensor_queue);

	//k_thread_start(sensor_publish);
	k_thread_resume(sensor_publish);
}

void stop_sensor_manager() {
    Baro::sensor.stop();
	IMU::sensor.stop();
	PPG::sensor.stop();
	Temp::sensor.stop();
	BoneConduction::sensor.stop();

	k_thread_suspend(sensor_publish);
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

void config_sensor(struct sensor_config * config) {
	EdgeMlSensor * sensor = get_sensor((enum sensor_id) config->sensorId);

	if (config->storageOptions == 0) {
		sensor->stop();
		return;
	}

	sensor->sd_logging(config->storageOptions & DATA_STORAGE);
	sensor->ble_stream(config->storageOptions & DATA_STREAMING);

	if (sensor->init(&sensor_queue)) sensor->start(config->sampleRateIndex);
}