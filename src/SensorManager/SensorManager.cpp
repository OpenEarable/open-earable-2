#include "SensorManager.h"
#include <zephyr/kernel.h>

#include "IMU.h"
#include "Baro.h"
#include "PPG.h"

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
}

void config_sensor(struct sensor_config * config) {
    k_timeout_t t = K_USEC(1e6 / config->sampleRate);

	/*switch (config->sensorId)
	{
	case ID_IMU:
		if (IMU::sensor.init(&sensor_queue)) IMU::sensor.start(t);
		break;
	case ID_TEMP_BARO:
		if (Baro::sensor.init(&sensor_queue)) Baro::sensor.start(t);
		break;
	case ID_PPG:
		if (PPG::sensor.init(&sensor_queue)) PPG::sensor.start(t);
		break;
	default:
		break;
	}*/
}