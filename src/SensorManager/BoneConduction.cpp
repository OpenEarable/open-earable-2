#include "BoneConduction.h"

#include "SensorManager.h"
#include "sensor_sink.h"

#include "math.h"
#include "stdlib.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(BMA580);

BoneConduction BoneConduction::sensor;

static struct sensor_msg msg_bc;
static uint32_t bc_queue_full_count;

static struct {
    uint32_t timer_fires;
    uint32_t early_returns;
    uint32_t reads;
    uint32_t read_errors;
    uint32_t total_samples;
    uint32_t max_samples;
    uint32_t exec_max_us;
    uint64_t exec_total_us;
} bc_stats;

const SampleRateSetting<10> BoneConduction::sample_rates = {
    { BMA5_ACC_ODR_HZ_12P5, BMA5_ACC_ODR_HZ_25, BMA5_ACC_ODR_HZ_50, BMA5_ACC_ODR_HZ_100, 
      BMA5_ACC_ODR_HZ_200, BMA5_ACC_ODR_HZ_400, BMA5_ACC_ODR_HZ_800, BMA5_ACC_ODR_HZ_1K6, 
      BMA5_ACC_ODR_HZ_3K2, BMA5_ACC_ODR_HZ_6K4 },   // reg_vals

    { 12.5, 25.0, 50.0, 100.0, 200.0, 400.0, 800.0, 1600.0, 3200.0, 6400.0 },  // sample_rates

    { 12.5, 25.0, 50.0, 100.0, 200.0, 400.0, 800.0, 1600.0, 3200.0, 6400.0 }   // true_sample_rates
};

bool BoneConduction::init() {
    if (!_active) {
        pm_device_runtime_get(ls_1_8);
        pm_device_runtime_get(ls_3_3);
        k_msleep(50);
    	_active = true;
	}

    if (bma580.init() != 0) {
		LOG_WRN("Could not find a valid bone conduction sensor, check wiring!");
        _active = false;
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);
		return false;
    }

	k_work_init(&sensor.sensor_work, update_sensor);
	k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

	return true;
}

void BoneConduction::update_sensor(struct k_work *work) {
    if (!sensor._running) return;
    uint64_t t0 = micros();

    bc_stats.timer_fires++;

    BoneConduction::sensor._sample_count += (t0 - BoneConduction::sensor._last_time_stamp) / BoneConduction::sensor.t_sample_us;
    BoneConduction::sensor._last_time_stamp = t0;

    if (BoneConduction::sensor._sample_count < BoneConduction::sensor._num_samples_buffered * (1.f - CONFIG_SENSOR_CLOCK_ACCURACY / 100.f)) {
        bc_stats.early_returns++;
        return;
    }

    int num_samples = sensor.bma580.read(sensor.fifo_acc_data);

    if (num_samples < 0) {
        bc_stats.read_errors++;
        LOG_WRN("BMA580 read failed: %d", num_samples);
        return;
    }

    bc_stats.reads++;
    bc_stats.total_samples += num_samples;
    if ((uint32_t)num_samples > bc_stats.max_samples) {
        bc_stats.max_samples = num_samples;
    }

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

        uint64_t dt_us = (uint64_t)((double)(num_samples - written) * (double)BoneConduction::sensor.t_sample_us);
        msg_bc.data.time = t0 - dt_us;

        if (to_write > 1) {
            uint16_t t_diff = BoneConduction::sensor.t_sample_us;
            for (int i = 0; i < to_write; i++) {
                memcpy(&msg_bc.data.data[i * _size], &sensor.fifo_acc_data[written + i], _size);
            }
            memcpy(&msg_bc.data.data[msg_bc.data.size - sizeof(uint16_t)], &t_diff, sizeof(uint16_t));
        } else {
            memcpy(&msg_bc.data.data, &sensor.fifo_acc_data[written], _size);
        }

        int ret = sensor_sink_put(&msg_bc);
        if (ret) {
            bc_queue_full_count++;
        }

        written += to_write;
    }

    uint32_t elapsed = (uint32_t)(micros() - t0);
    bc_stats.exec_total_us += elapsed;
    if (elapsed > bc_stats.exec_max_us) {
        bc_stats.exec_max_us = elapsed;
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

    int word_size = 3 * sizeof(int16_t) + 1;
    int fifo_capacity = 1024 / word_size;
    int latency_samples = (int)(CONFIG_SENSOR_LATENCY_MS * 1e3 / t_sample_us);
    _num_samples_buffered = MIN(MAX(1, latency_samples), fifo_capacity / 2);

    bma580.init(sample_rates.reg_vals[sample_rate_idx], _num_samples_buffered * word_size);
    bma580.start();

    k_timeout_t t = K_USEC(t_sample_us * _num_samples_buffered);
	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);

    _running = true;
    _sample_count = 0;
    _last_time_stamp = micros();
}

void BoneConduction::stop() {
    if (!_active) return;
    _active = false;

    _running = false;

    uint32_t exec_avg = bc_stats.reads ? (uint32_t)(bc_stats.exec_total_us / bc_stats.reads) : 0;
    uint32_t samples_avg = bc_stats.reads ? bc_stats.total_samples / bc_stats.reads : 0;
    LOG_INF("bone_acc: timer=%u early_ret=%u reads=%u errors=%u queue_drops=%u",
            bc_stats.timer_fires, bc_stats.early_returns, bc_stats.reads,
            bc_stats.read_errors, bc_queue_full_count);
    LOG_INF("bone_acc: total_samples=%u avg=%u max=%u",
            bc_stats.total_samples, samples_avg, bc_stats.max_samples);
    LOG_INF("bone_acc: exec_avg=%u us exec_max=%u us",
            exec_avg, bc_stats.exec_max_us);
    memset(&bc_stats, 0, sizeof(bc_stats));
    bc_queue_full_count = 0;

	k_timer_stop(&sensor.sensor_timer);
	k_work_cancel(&sensor.sensor_work);

    bma580.stop();

    pm_device_runtime_put(ls_1_8);
    pm_device_runtime_put(ls_3_3);
}