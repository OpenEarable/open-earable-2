#include "SensorManager.h"
#include <zephyr/kernel.h>
//#include <zephyr/zbus/zbus.h>

extern struct k_msgq sensor_queue;

SensorManager SensorManager::manager = SensorManager();

void SensorManager::start() {
    baro.init(&sensor_queue);
	imu.init(&sensor_queue);
}

void SensorManager::stop() {
    baro.stop();
	imu.stop();
}

void SensorManager::config(sensor_config * config) {
    k_timeout_t t = K_MSEC(1000 / config->sampleRate);

	switch (config->sensorId)
	{
	case ID_IMU:
		imu.start(t);
		break;
	case ID_TEMP_BARO:
		baro.start(t);
		break;
	
	default:
		break;
	}
}