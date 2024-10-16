#include "Temp.h"

#include "math.h"
#include "stdlib.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(MLX90632);

Temp Temp::sensor;

MLX90632 Temp::temp;

static struct sensor_data msg_temp;

bool Temp::init(struct k_msgq * queue) {
    if (!_active) {
        pm_device_runtime_get(ls_1_8);
        pm_device_runtime_get(ls_3_3);
    	_active = true;
	}

    if (!temp.begin()) {   // hardware I2C mode, can pass in address & alt Wire
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);

		printk("Could not find a valid Optical Temperature sensor, check wiring!");
		return false;
    }

	sensor_queue = queue;
	
	k_work_init(&sensor.sensor_work, update_sensor);
	k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

    _active = true;

	return true;
}

void Temp::reset() {
    // Reset pulse oximeter state
}

void Temp::update_sensor(struct k_work *work) {
    float temperature = temp.getObjectTempF();
    msg_temp.id = ID_OPTTEMP;
    msg_temp.size = sizeof(float);
    msg_temp.time = millis();
    msg_temp.data[0]=temperature;

    int ret = k_msgq_put(sensor_queue, &msg_temp, K_NO_WAIT);
    if (ret == -EAGAIN) {
        LOG_WRN("sensor msg queue full");
    }
}

/**
* @brief Submit a k_work on timer expiry.
*/
void Temp::sensor_timer_handler(struct k_timer *dummy) {
	k_work_submit(&sensor.sensor_work);
}

void Temp::start(k_timeout_t t) {
    if (!_active) return;

	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);
}

void Temp::stop() {
    if (!_active) return;
    _active = false;

	k_timer_stop(&sensor.sensor_timer);

    pm_device_runtime_put(ls_1_8);
    pm_device_runtime_put(ls_3_3);
}