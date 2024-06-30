#include "Temp.h"

#include "math.h"
#include "stdlib.h"

Temp Temp::sensor;

MLX90632 Temp::temp;

static struct sensor_data msg_temp;

bool Temp::init(struct k_msgq * queue) {
    if (!temp.begin()) {   // hardware I2C mode, can pass in address & alt Wire
		printk("Could not find a valid Optical Temperature sensor, check wiring!");
		return false;
    }

	sensor_queue = queue;
	
	k_work_init(&sensor.sensor_work, update_sensor);
	k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

	return true;
}

void Temp::reset() {
    // Reset pulse oximeter state
}

void Temp::update_sensor(struct k_work *work) {
    //ppg.check();
    float temperature = temp.getObjectTempF();
    msg_temp.id = ID_OPTTEMP;
    msg_temp.size = sizeof(float);
    msg_temp.time = k_cyc_to_ms_floor32(k_cycle_get_32());
    msg_temp.data[0]=temperature;

    int ret = k_msgq_put(sensor_queue, &msg_temp, K_NO_WAIT);
    if (ret == -EAGAIN) {
        //LOG_WRN("sensor msg queue full");
        printk("sensor msg queue full");
    }
}

/**
* @brief Submit a k_work on timer expiry.
*/
void Temp::sensor_timer_handler(struct k_timer *dummy) {
	k_work_submit(&sensor.sensor_work);
}

void Temp::start(k_timeout_t t) {
	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);
}

void Temp::stop() {
	k_timer_stop(&sensor.sensor_timer);
}