#include "IMU.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/device.h>

//extern struct k_msgq sensor_queue;

static struct sensor_msg msg_imu;

/*k_work IMU::sensor_work;
k_msgq * IMU::sensor_queue;*/
DFRobot_BMX160 IMU::imu(&Wire1);

IMU IMU::sensor;

void IMU::update_sensor(struct k_work *work) {
	int ret;

	msg_imu.sd = sensor._sd_logging;
	msg_imu.stream = sensor._ble_stream;

	msg_imu.data.id = ID_IMU;
	msg_imu.data.size = 9 * sizeof(float);
	msg_imu.data.time = millis();

	sBmx160SensorData_t magno_data;
	sBmx160SensorData_t gyro_data;
	sBmx160SensorData_t accel_data;

	imu.getAllData(&magno_data, &gyro_data, &accel_data);

	msg_imu.data.data[0] = accel_data.x;
	msg_imu.data.data[1] = accel_data.y;
	msg_imu.data.data[2] = accel_data.z;

	msg_imu.data.data[3] = gyro_data.x;
	msg_imu.data.data[4] = gyro_data.y;
	msg_imu.data.data[5] = gyro_data.z;

	msg_imu.data.data[6] = magno_data.x;
	msg_imu.data.data[7] = magno_data.y;
	msg_imu.data.data[8] = magno_data.z;

	//imu.getAllData((sBmx160SensorData_t*) &msg_imu.data[6], (sBmx160SensorData_t*) &msg_imu.data[3], (sBmx160SensorData_t*) &msg_imu);

	ret = k_msgq_put(sensor_queue, &msg_imu, K_NO_WAIT);
	if (ret == -EAGAIN) {
		//LOG_WRN("sensor msg queue full");
		printk("sensor msg queue full");
	}
}

/**
* @brief Submit a k_work on timer expiry.
*/
void IMU::sensor_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&sensor.sensor_work);
};

bool IMU::init(struct k_msgq * queue) {
	if (!_active) {
		pm_device_runtime_get(ls_1_8);
    	_active = true;
	}

    if (!imu.begin()) {   // hardware I2C mode, can pass in address & alt Wire
		printk("Could not find a valid BMX160 sensor, check wiring!");
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

void IMU::start(k_timeout_t t) {
	if (!_active) return;
	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);
}

void IMU::stop() {
    if (!_active) return;
    _active = false;

	k_timer_stop(&sensor.sensor_timer);

	// turn off imu (?)
	imu.softReset();

    pm_device_runtime_put(ls_1_8);
}