#include "BoneConduction.h"

#include "math.h"
#include "stdlib.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(BMA580);

BoneConduction BoneConduction::sensor;

static struct sensor_data msg_bc;

bool BoneConduction::init(struct k_msgq * queue) {
    if (!_active) {
        pm_device_runtime_get(ls_1_8);
    	_active = true;
	}
    
    if (bma580.init() != 0) {   // hardware I2C mode, can pass in address & alt Wire
		LOG_WRN("Could not find a valid bone conduction sensor, check wiring!");
        _active = false;
        pm_device_runtime_put(ls_1_8);
		return false;
    }

	sensor_queue = queue;
	
	k_work_init(&sensor.sensor_work, update_sensor);
	k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

	return true;
}

void BoneConduction::reset() {
    // Reset pulse oximeter state
}

void BoneConduction::update_sensor(struct k_work *work) {
    int num_samples = sensor.bma580.read(sensor.fifo_acc_data);

    for (int i = 0; i < num_samples; i++) {
        msg_bc.id = ID_BONE_CONDUCTION;
        msg_bc.size = 3 * sizeof(uint32_t);
        msg_bc.time = millis();
        msg_bc.data[0]=sensor.fifo_acc_data[i].x;
        msg_bc.data[1]=sensor.fifo_acc_data[i].y;
        msg_bc.data[2]=sensor.fifo_acc_data[i].z;

        int ret = k_msgq_put(sensor_queue, &msg_bc, K_NO_WAIT);
        if (ret == -EAGAIN) {
            LOG_WRN("sensor msg queue full");
        }
    }
}

/**
* @brief Submit a k_work on timer expiry.
*/
void BoneConduction::sensor_timer_handler(struct k_timer *dummy) {
	k_work_submit(&sensor.sensor_work);
}

void BoneConduction::start(k_timeout_t t) {
    if (!_active) return;
    
    bma580.init();
    bma580.start();
	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);
}

void BoneConduction::stop() {
    if (!_active) return;
    _active = false;

	k_timer_stop(&sensor.sensor_timer);

    bma580.stop();

    pm_device_runtime_put(ls_1_8);
}