#include "IMU.h"

#include "SensorManager.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(BMX160);

static struct sensor_msg msg_imu;

DFRobot_BMX160 IMU::imu(&I2C3);

IMU IMU::sensor;

const SampleRateSetting<6> IMU::sample_rates = {
    { BMX160_GYRO_ODR_25HZ, BMX160_GYRO_ODR_50HZ, BMX160_GYRO_ODR_100HZ,
	BMX160_GYRO_ODR_200HZ, BMX160_GYRO_ODR_400HZ, BMX160_GYRO_ODR_800HZ },

	{ 25, 50, 100, 200, 400, 800 },

	{ 25.0, 50.0, 100.0, 200.0, 400.0, 800.0 }
};

void IMU::update_sensor(struct k_work *work) {
	int ret;

	sBmx160SensorData_t magno_data;
	sBmx160SensorData_t gyro_data;
	sBmx160SensorData_t accel_data;

	imu.getAllData(&magno_data, &gyro_data, &accel_data);

	size_t size =  3 * sizeof(float);
	
	msg_imu.sd = sensor._sd_logging;
	msg_imu.stream = sensor._ble_stream;

	msg_imu.data.id = ID_IMU;
	msg_imu.data.size = 3 * size;
	msg_imu.data.time = micros();

	memcpy(msg_imu.data.data, &accel_data,size);
	memcpy(msg_imu.data.data + size, &gyro_data, size);
	memcpy(msg_imu.data.data + 2 * size, &magno_data, size);

	ret = k_msgq_put(sensor_queue, &msg_imu, K_NO_WAIT);
	if (ret) {
		LOG_WRN("sensor msg queue full");
	}
}

/**
* @brief Submit a k_work on timer expiry.
*/
void IMU::sensor_timer_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
};

bool IMU::init(struct k_msgq * queue) {
	if (!_active) {
		pm_device_runtime_get(ls_1_8);
    	_active = true;
	}

    if (!imu.begin()) {   // hardware I2C mode, can pass in address & alt Wire
		LOG_ERR("Could not find a valid BMX160 sensor, check wiring!");
		pm_device_runtime_put(ls_1_8);
    	_active = false;
		return false;
    }

	imu.setAccelRange(eAccelRange_2G);

	sensor_queue = queue;
	
	k_work_init(&sensor.sensor_work, update_sensor);
	k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

	return true;
}

void IMU::start(int sample_rate_idx) {
	if (!_active) return;

    k_timeout_t t = K_USEC(1e6 / sample_rates.true_sample_rates[sample_rate_idx]);

	imu.setAccelODR(sample_rates.reg_vals[sample_rate_idx]);

	_running = true;

	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);
}

void IMU::stop() {
    if (!_active) return;
    _active = false;

	_running = false;

	k_timer_stop(&sensor.sensor_timer);

	// turn off imu (?)
	imu.softReset();

    pm_device_runtime_put(ls_1_8);
}