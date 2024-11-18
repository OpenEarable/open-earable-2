#include "BoneConduction.h"

#include "math.h"
#include "stdlib.h"

#include "nrf5340_audio_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(BMA580);

BoneConduction BoneConduction::sensor;

//BMA580 BoneConduction::bone_conduction(Wire1);

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
    //int int_status;
    //int status = bma5880.read(fifo_acc_data);
    //if((status == 0) && (int_status & MAXM86161_INT_DATA_RDY)) {
        int num_samples = sensor.bma580.read(sensor.fifo_acc_data);

        for (int i = 0; i < num_samples; i++) {
            msg_bc.id = ID_PPG;
            msg_bc.size = 4 * sizeof(uint32_t);
            msg_bc.time = millis();
            msg_bc.data[0]=sensor.fifo_acc_data[i].x;
            msg_bc.data[1]=sensor.fifo_acc_data[i].y;
            msg_bc.data[2]=sensor.fifo_acc_data[i].z;
        }
    //}
}

/**
* @brief Submit a k_work on timer expiry.
*/
void BoneConduction::sensor_timer_handler(struct k_timer *dummy) {
	k_work_submit(&sensor.sensor_work);
}

void BoneConduction::start(k_timeout_t t) {
    if (!_active) return;
    //ppg.setup(0x64, 1, 2, 400, 215, 8192);
    bma580.init();
    bma580.start();
	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);
}

void BoneConduction::stop() {
    if (!_active) return;
    _active = false;

	k_timer_stop(&sensor.sensor_timer);

    //ppg.stop();

    bma580.stop();

    pm_device_runtime_put(ls_1_8);
}