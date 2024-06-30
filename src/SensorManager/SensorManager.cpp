#include "SensorManager.h"
#include <zephyr/kernel.h>

#include "IMU.h"
#include "Baro.h"
#include "PPG.h"
#include "Temp.h"

extern struct k_msgq sensor_queue;

//SensorManager SensorManager::manager = SensorManager();

void start_sensor_manager() {
    //baro.init(&sensor_queue);
	//imu.init(&sensor_queue);
}

void stop_sensor_manager() {
    Baro::sensor.stop();
	IMU::sensor.stop();
	PPG::sensor.stop();
	Temp::sensor.stop();
}

void config_sensor(struct sensor_config * config) {
    k_timeout_t t = K_USEC(1e6 / config->sampleRate);

	EdgeMlSensor * sensor;

	switch (config->sensorId)
	{
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
	default:
		return;
	}

	if (config->sampleRate == 0) sensor->stop();
	else if (sensor->init(&sensor_queue)) sensor->start(t);
}