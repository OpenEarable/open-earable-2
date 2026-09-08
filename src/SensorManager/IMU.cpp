#include "IMU.h"

#include "SensorManager.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(BMX160);

static struct sensor_msg msg_imu;

BMX160 IMU::imu(&I2C3);

IMU IMU::sensor;

const SampleRateSetting<6> IMU::sample_rates = {
    { BMI160_GYRO_ODR_25HZ, BMI160_GYRO_ODR_50HZ, BMI160_GYRO_ODR_100HZ,
	BMI160_GYRO_ODR_200HZ, BMI160_GYRO_ODR_400HZ, BMI160_GYRO_ODR_800HZ },

	{ 25, 50, 100, 200, 400, 800 },

	{ 25.0, 50.0, 100.0, 200.0, 400.0, 800.0 }
};

void IMU::update_sensor(struct k_work *work) {
	ARG_UNUSED(work);
	const int num_samples = imu.read(sensor.sample_buffer, sensor.MAX_BUFFERED_SAMPLES);
	const uint64_t read_finished_us = micros();

	if (num_samples < 0) {
		LOG_WRN("BMX160 FIFO read failed: %d", num_samples);
		return;
	}

	for (int i = 0; i < num_samples; ++i) {
		msg_imu.sd = sensor._sd_logging;
		msg_imu.stream = sensor._ble_stream;
		msg_imu.data.id = ID_IMU;
		msg_imu.data.size = 9 * sizeof(float);
		msg_imu.data.time = read_finished_us -
			(uint64_t)((num_samples - 1 - i) * sensor.t_sample_us);

		memcpy(msg_imu.data.data, sensor.sample_buffer[i].accel, 3 * sizeof(float));
		memcpy(msg_imu.data.data + 3 * sizeof(float), sensor.sample_buffer[i].gyro, 3 * sizeof(float));
		memcpy(msg_imu.data.data + 6 * sizeof(float), sensor.sample_buffer[i].mag, 3 * sizeof(float));

		const int ret = k_msgq_put(sensor_queue, &msg_imu, K_NO_WAIT);
		if (ret) {
			LOG_WRN("sensor msg queue full");
		}
	}

}

/**
* @brief Submit a k_work on timer expiry.
*/
void IMU::sensor_timer_handler(struct k_timer *dummy)
{
	ARG_UNUSED(dummy);
	k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
};

bool IMU::init(struct k_msgq * queue) {
	if (!_active) {
		pm_device_runtime_get(ls_1_8);
    	_active = true;
	}

	if (!imu.init()) {
		LOG_ERR("Could not find a valid BMX160 sensor, check wiring!");
		pm_device_runtime_put(ls_1_8);
    	_active = false;
		return false;
    }

	sensor_queue = queue;
	
	k_work_init(&sensor.sensor_work, update_sensor);
	k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

	return true;
}

void IMU::start(int sample_rate_idx) {
	if (!_active) return;

	t_sample_us = 1e6f / sample_rates.true_sample_rates[sample_rate_idx];
	const uint8_t num_samples_buffered = MIN(MAX(1, (int)(CONFIG_SENSOR_LATENCY_MS * 1e3f / t_sample_us)),
		(int)MAX_BUFFERED_SAMPLES);

	const int result = imu.start(sample_rates.reg_vals[sample_rate_idx],
		sample_rates.true_sample_rates[sample_rate_idx], num_samples_buffered);
	if (result != 0) {
		LOG_ERR("BMX160 FIFO start failed: %d", result);
		return;
	}

	const uint32_t poll_period_us = (uint32_t)(num_samples_buffered * t_sample_us);
	const k_timeout_t t = K_USEC(poll_period_us);

	_running = true;

	// Let the FIFO collect the first complete batch before the first burst read.
	k_timer_start(&sensor.sensor_timer, t, t);
	LOG_INF("BMX160 FIFO started: rate=%dHz buffered=%u poll=%uus",
		(int)sample_rates.true_sample_rates[sample_rate_idx], num_samples_buffered, poll_period_us);
}

void IMU::stop() {
    if (!_active) return;
    _active = false;

	_running = false;

	k_timer_stop(&sensor.sensor_timer);

	imu.stop();

    pm_device_runtime_put(ls_1_8);
}
