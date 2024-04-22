#include "IMU.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/device.h>

//extern struct k_msgq sensor_queue;

static struct sensor_data msg_imu;

/*k_work IMU::sensor_work;
k_msgq * IMU::sensor_queue;*/
DFRobot_BMX160 IMU::imu;

IMU IMU::sensor;

void IMU::update_sensor(struct k_work *work) {
	int ret;

	/*sBmx160SensorData_t accel_data;
	sBmx160SensorData_t gyro_data;
	sBmx160SensorData_t magno_data;*/

	msg_imu.id = ID_IMU;
	msg_imu.size = 9 * sizeof(float);
	msg_imu.time = k_cyc_to_ms_floor32(k_cycle_get_32());

	//imu.getAllData(&magno_data, &gyro_data, &accel_data);
	imu.getAllData((sBmx160SensorData_t*) &msg_imu.data[6], (sBmx160SensorData_t*) &msg_imu.data[3], (sBmx160SensorData_t*) &msg_imu.data[0]);

	/*msg_imu.data[0] = magno_data.x;
	msg_imu.data[1] = magno_data.y;
	msg_imu.data[2] = magno_data.z;
	msg_imu.data[3] = gyro_data.x;
	msg_imu.data[4] = gyro_data.y;
	msg_imu.data[5] = gyro_data.z;
	msg_imu.data[6] = accel_data.x;
	msg_imu.data[7] = accel_data.y;
	msg_imu.data[8] = accel_data.z;*/

	ret = k_msgq_put(sensor_queue, (void *)&msg_imu, K_NO_WAIT);
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
    if (!imu.begin()) {   // hardware I2C mode, can pass in address & alt Wire
		printk("Could not find a valid BMX160 sensor, check wiring!");
		return false;
    }

	sensor_queue = queue;
	
	k_work_init(&sensor.sensor_work, update_sensor);
	k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

	return true;
}

void IMU::start(k_timeout_t t) {
	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);
}

void IMU::stop() {
	k_timer_stop(&sensor.sensor_timer);
}