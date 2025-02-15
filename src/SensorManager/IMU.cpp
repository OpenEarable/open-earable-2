#include "IMU.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/device.h>

//extern struct k_msgq sensor_queue;

static struct sensor_data msg_imu;

/*k_work IMU::sensor_work;
k_msgq * IMU::sensor_queue;*/
DFRobot_BMX160 IMU::imu(&Wire1);

IMU IMU::sensor;

uint64_t IMU::system_time_us_ref = 0;
uint64_t IMU::fifo_time_us_ref = 0;
uint64_t IMU::last_fifo_time_us = 0;

void IMU::update_sensor(struct k_work *work) {
	int ret;

	msg_imu.id = ID_IMU;
	msg_imu.size = 9 * sizeof(float);


	sBmx160SensorData_t magno_data;
	sBmx160SensorData_t gyro_data;
	sBmx160SensorData_t accel_data;

	imu.getAllData(&magno_data, &gyro_data, &accel_data);

	uint64_t fifo_time_us = (uint64_t) accel_data.sensor_time * 39;

	// Handle FIFO rollover (24-bit counter overflow)
	if (fifo_time_us < last_fifo_time_us) {  
		fifo_time_us += (uint64_t)(1 << 24) * 39; // TODO: this only works for one rollover, maybe sync time here?
	}

	// Compute synchronized timestamp
	uint64_t timestamp = system_time_us_ref + (fifo_time_us - fifo_time_us_ref);

	// Store and update last known timestamp
	last_fifo_time_us = fifo_time_us;
	
	msg_imu.time = timestamp();

	msg_imu.data[0] = accel_data.x;
	msg_imu.data[1] = accel_data.y;
	msg_imu.data[2] = accel_data.z;

	msg_imu.data[3] = gyro_data.x;
	msg_imu.data[4] = gyro_data.y;
	msg_imu.data[5] = gyro_data.z;

	msg_imu.data[6] = magno_data.x;
	msg_imu.data[7] = magno_data.y;
	msg_imu.data[8] = magno_data.z;

	//imu.getAllData((sBmx160SensorData_t*) &msg_imu.data[6], (sBmx160SensorData_t*) &msg_imu.data[3], (sBmx160SensorData_t*) &msg_imu);

	ret = k_msgq_put(sensor_queue, &msg_imu, K_NO_WAIT);
	if (ret == -EAGAIN) {
		//LOG_WRN("sensor msg queue full");
		printk("sensor msg queue full");
	}

	sync_fifo_time();
}

void IMU::sync_fifo_time() {
	uint64_t current_time = micros();  // Get the current system time

    // Only sync if at least 5 seconds (5,000,000 µs = 5 s) have passed
    if (current_time < system_time_us_ref + 5000000) {
        return;  // Skip synchronization if the time condition is not met
    }

    system_time_us_ref = micros(); // Get system time in microseconds, fetch again for max accuracy
    uint8_t sensor_time_raw[3];
    imu.readReg(BMX160_SENSOR_TIME_ADDR, sensor_time_raw, 3); // TODO: there is a tiny risk that we read the time during overflow ...

    uint64_t fifo_time = ((uint64_t)sensor_time_raw[2] << 16) | 
                          ((uint64_t)sensor_time_raw[1] << 8)  | 
                          (uint64_t)sensor_time_raw[0];

    fifo_time_us_ref = fifo_time * 39; // BMX160 time conversion factor
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
	sync_fifo_time(); // Initial synchronization
}

void IMU::stop() {
    if (!_active) return;
    _active = false;

	k_timer_stop(&sensor.sensor_timer);

	// turn off imu (?)
	imu.softReset();

    pm_device_runtime_put(ls_1_8);
}