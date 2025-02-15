#include "BoneConduction.h"

#include "math.h"
#include "stdlib.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(BMA580);

BoneConduction BoneConduction::sensor;

static struct sensor_data msg_bc;

uint64_t BoneConduction::system_time_us_ref = 0;
uint64_t BoneConduction::fifo_time_us_ref = 0;
uint64_t BoneConduction::last_fifo_time_us = 0;

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
        uint64_t fifo_time_us = (uint64_t)sensor.fifo_acc_data[i].sensor_time * 312.5;

        // Handle FIFO rollover (24-bit counter overflow)
        if (fifo_time_us < last_fifo_time_us) {  
            fifo_time_us += (uint64_t)(1 << 24) * 312.5;  // Correct for rollover
        }

        // Compute synchronized timestamp
        uint64_t timestamp = system_time_us_ref + (fifo_time_us - fifo_time_us_ref);

        // Store and update last known timestamp
        last_fifo_time_us = fifo_time_us;

        msg_bc.id = ID_BONE_CONDUCTION;
        msg_bc.size = 3 * sizeof(uint32_t);
        msg_bc.time = timestamp;
        msg_bc.data[0]=sensor.fifo_acc_data[i].x;
        msg_bc.data[1]=sensor.fifo_acc_data[i].y;
        msg_bc.data[2]=sensor.fifo_acc_data[i].z;

        int ret = k_msgq_put(sensor_queue, &msg_bc, K_NO_WAIT);
        if (ret == -EAGAIN) {
            LOG_WRN("sensor msg queue full");
        }
    }

    // Call the clock resynchronization method after reading all frames
    sync_fifo_time();
}

void BoneConduction::sync_fifo_time() {
    // Set the reference timestamp
    system_time_us_ref = micros(); // Get system time in microseconds

    uint8_t sensor_time_raw[3];
    bma580.read(BMA5_REG_SENSOR_TIME_0, sensor_time_raw, 3);

    // Convert FIFO 24-bit timestamp to uint64_t (safe for multiplication)
    uint64_t fifo_time = ((uint64_t)sensor_time_raw[2] << 16) | 
                          ((uint64_t)sensor_time_raw[1] << 8)  | 
                          (uint64_t)sensor_time_raw[0];

    // Multiply safely using uint64_t
    fifo_time_us_ref = fifo_time * 312.5;
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

    // Initial clock synchronization
    sync_fifo_time();
}

void BoneConduction::stop() {
    if (!_active) return;
    _active = false;

	k_timer_stop(&sensor.sensor_timer);

    bma580.stop();

    pm_device_runtime_put(ls_1_8);
}