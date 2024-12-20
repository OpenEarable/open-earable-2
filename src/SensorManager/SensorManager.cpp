#include "SensorManager.h"
#include <zephyr/kernel.h>

#include "IMU.h"
#include "Baro.h"
#include "PPG.h"
#include "Temp.h"
#include "BoneConduction.h"

extern struct k_msgq sensor_queue;
extern k_tid_t sensor_publish;

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

void config_sensor(struct sensor_config * config) {
    k_timeout_t t = K_USEC(1e6 / config->sampleRate);

	EdgeMlSensor * sensor;

	switch (config->sensorId) {
	case ID_IMU:
		sensor = &(IMU::sensor);
		break;
	case ID_TEMP_BARO:
		sensor = &(Baro::sensor);
		break;
	case ID_PPG:
		sensor = &(PPG::sensor);
		break;
	case ID_OPTTEMP:
		sensor = &(Temp::sensor);
		break;
	case ID_BONE_CONDUCTION:
		sensor = &(BoneConduction::sensor);
		break;
	default:
		return;
	}

	if (config->sampleRate == 0) sensor->stop();
	else if (sensor->init(&sensor_queue)) sensor->start(t);
}