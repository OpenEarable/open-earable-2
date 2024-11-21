#include "Baro.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(BMP388);

static struct sensor_data msg_baro;

Adafruit_BMP3XX Baro::bmp;

Baro Baro::sensor;

void Baro::update_sensor(struct k_work *work) {
	int ret;

	bmp.performReading();

	msg_baro.id = ID_TEMP_BARO;
	msg_baro.size = 2 * sizeof(float);
	msg_baro.time = millis();
	msg_baro.data[0] = bmp.temperature;
	msg_baro.data[1] = bmp.pressure;

	ret = k_msgq_put(sensor_queue, &msg_baro, K_NO_WAIT);
	if (ret == -EAGAIN) {
		//LOG_WRN("sensor msg queue full");
		LOG_WRN("sensor msg queue full");
	}
}

/*void test(Baro * b, k_timer * t) {
	b->sensor_timer_handler(t);
}*/

/**
* @brief Submit a k_work on timer expiry.
*/
void Baro::sensor_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&sensor.sensor_work);
};

bool Baro::init(struct k_msgq * queue) {
	if (!_active) {
		pm_device_runtime_get(ls_1_8);
    	_active = true;
	}

    if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
		LOG_WRN("Could not find a valid BMP388 sensor, check wiring!");
		pm_device_runtime_put(ls_1_8);
		return false;
    }

	sensor_queue = queue;
	
	k_work_init(&sensor.sensor_work, update_sensor);
	k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

	return true;
}

void Baro::start(k_timeout_t t) {
	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);
}

void Baro::stop() {
	if (!_active) return;
    _active = false;

	k_timer_stop(&sensor.sensor_timer);

    pm_device_runtime_put(ls_1_8);
}