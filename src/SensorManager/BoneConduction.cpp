#include "BoneConduction.h"

#include "math.h"
#include "stdlib.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(BMA580);

BoneConduction BoneConduction::sensor;

static struct sensor_msg msg_bc;

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

        if (fifo_time_us < last_fifo_time_us) {  
            BoneConduction::sync_fifo_time(true); // resync timer if rollover happened, happens approx. after 1.5h
        }
        // Compute synchronized timestamp
        uint64_t timestamp = system_time_us_ref + (fifo_time_us - fifo_time_us_ref); // TODO: what to do with quantization error?

        // Store and update last known timestamp
        last_fifo_time_us = fifo_time_us;

        msg_bc.sd = sensor._sd_logging;
	    msg_bc.stream = sensor._ble_stream;

        msg_bc.data.id = ID_BONE_CONDUCTION;
        msg_bc.data.size = 3 * sizeof(int16_t);
        msg_bc.data.time = timestamp;
        memcpy(msg_bc.data.data, &sensor.fifo_acc_data[i], 3 * sizeof(int16_t));


        int ret = k_msgq_put(sensor_queue, &msg_bc, K_NO_WAIT);
        if (ret == -EAGAIN) {
            LOG_WRN("sensor msg queue full");
        }
    }

    // Call the clock resynchronization method after reading all frames
    BoneConduction::sync_fifo_time(false);
}

void BoneConduction::sync_fifo_time(bool force) {
    uint64_t current_time = micros();  // Get the current system time

    // Only sync if at least 5 seconds (1,000,000 µs = 1 s) have passed
    if (!force && current_time < system_time_us_ref + 1000000) {  // TODO: define constant somewhere
        return;  // Skip synchronization if the time condition is not met
    }

    // Update the reference timestamp
    system_time_us_ref = micros(); // fetch again for max accuracy
    uint64_t fifo_time = sensor.bma580.read_time();

    fifo_time_us_ref = fifo_time * 312.5; // TODO: define constant somewhere
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
    sync_fifo_time(false);
}

void BoneConduction::stop() {
    if (!_active) return;
    _active = false;

	k_timer_stop(&sensor.sensor_timer);

    bma580.stop();

    pm_device_runtime_put(ls_1_8);
}