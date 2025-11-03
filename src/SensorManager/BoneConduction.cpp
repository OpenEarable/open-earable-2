#include "BoneConduction.h"

#include "SensorManager.h"

#include "math.h"
#include "stdlib.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(BMA580);

BoneConduction BoneConduction::sensor;

static struct sensor_msg msg_bc;

const SampleRateSetting<10> BoneConduction::sample_rates = {
    { BMA5_ACC_ODR_HZ_12P5, BMA5_ACC_ODR_HZ_25, BMA5_ACC_ODR_HZ_50, BMA5_ACC_ODR_HZ_100, 
      BMA5_ACC_ODR_HZ_200, BMA5_ACC_ODR_HZ_400, BMA5_ACC_ODR_HZ_800, BMA5_ACC_ODR_HZ_1K6, 
      BMA5_ACC_ODR_HZ_3K2, BMA5_ACC_ODR_HZ_6K4 },   // reg_vals

    { 12.5, 25.0, 50.0, 100.0, 200.0, 400.0, 800.0, 1600.0, 3200.0, 6400.0 },  // sample_rates

    { 12.5, 25.0, 50.0, 100.0, 200.0, 400.0, 800.0, 1600.0, 3200.0, 6400.0 }   // true_sample_rates
};

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
    uint64_t _time_stamp = micros();

    BoneConduction::sensor._sample_count += (_time_stamp - BoneConduction::sensor._last_time_stamp) / BoneConduction::sensor.t_sample_us;
    BoneConduction::sensor._last_time_stamp = _time_stamp;
    
    if (BoneConduction::sensor._sample_count < BoneConduction::sensor._num_samples_buffered * (1.f - CONFIG_SENSOR_CLOCK_ACCURACY / 100.f)) {
        return;
    }

    int num_samples = sensor.bma580.read(sensor.fifo_acc_data);

    if (num_samples > 0) {
        BoneConduction::sensor._sample_count = MAX(0, BoneConduction::sensor._num_samples_buffered - num_samples);
    }

    int written = 0;

    const int _size = 3 * sizeof(int16_t);

    while (written < num_samples) {
        int to_write = MIN((SENSOR_DATA_FIXED_LENGTH - sizeof(uint16_t)) / _size, num_samples - written);
        if (to_write <= 0) break;

        msg_bc.sd = sensor._sd_logging;
        msg_bc.stream = sensor._ble_stream;

        msg_bc.data.id = ID_BONE_CONDUCTION;
        msg_bc.data.size = to_write * _size + sizeof(uint16_t);
        msg_bc.data.time = _time_stamp - (num_samples - written) * BoneConduction::sensor.t_sample_us;

        if (to_write > 1) {
            uint16_t t_diff = BoneConduction::sensor.t_sample_us;
            for (int i = 0; i < to_write; i++) {
                memcpy(&msg_bc.data.data[i * _size], &sensor.fifo_acc_data[written + i], _size);
            }
            memcpy(&msg_bc.data.data[msg_bc.data.size - sizeof(uint16_t)], &t_diff, sizeof(uint16_t));
        } else {
            memcpy(&msg_bc.data.data, &sensor.fifo_acc_data[written], _size);
        }

        int ret = k_msgq_put(sensor_queue, &msg_bc, K_NO_WAIT);
        if (ret) {
            LOG_WRN("sensor msg queue full");
        }

        written += to_write;
    }
}

/**
* @brief Submit a k_work on timer expiry.
*/
void BoneConduction::sensor_timer_handler(struct k_timer *dummy) {
	k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
}

void BoneConduction::start(int sample_rate_idx) {
    if (!_active) return;

    t_sample_us = 1e6 / sample_rates.true_sample_rates[sample_rate_idx];

    k_timeout_t t = K_USEC(t_sample_us);

    int word_size = 3 * sizeof(int16_t) + 1;
    _num_samples_buffered = MIN(MAX(1, (int) (CONFIG_SENSOR_LATENCY_MS * 1e3 / t_sample_us)), 512 / word_size - 8); // Buffer size is 512 bytes
    
    bma580.init(sample_rates.reg_vals[sample_rate_idx], _num_samples_buffered * word_size);
    bma580.start();

	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);

    _running = true;
    _sample_count = 0;
    _last_time_stamp = micros();
}

void BoneConduction::stop() {
    if (!_active) return;
    _active = false;

    _running = false;

	k_timer_stop(&sensor.sensor_timer);

    bma580.stop();

    pm_device_runtime_put(ls_1_8);
}