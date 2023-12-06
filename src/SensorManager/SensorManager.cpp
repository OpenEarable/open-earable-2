#include "SensorManager.h"

SensorManager SensorManager::manager = SensorManager();

void SensorManager::update_sensors(struct k_work *work) {
	sBmx160SensorData_t accel_data;
    sBmx160SensorData_t gyro_data;
    sBmx160SensorData_t magno_data;

	manager.imu.getAllData(&magno_data, &gyro_data, &accel_data);
	manager.bmp.performReading();

	printk("acc: %.3f, %.3f, %.3f", accel_data.x, accel_data.y, accel_data.z);
	printk(" | gyro: %.3f, %.3f, %.3f", gyro_data.x, gyro_data.y, gyro_data.z);
	printk(" | mag: %.3f, %.3f, %.3f\n", magno_data.x, magno_data.y, magno_data.z);

	printk("temperature: %.3fC, ", manager.bmp.temperature);
	printk("pressure: %.3fhPa\n", manager.bmp.pressure/100);
}

/**
 * @brief Submit a k_work on timer expiry.
 */
void SensorManager::sensor_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&manager.sensor_work);
}

void SensorManager::start() {
    if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
            printk("Could not find a valid BMP3 sensor, check wiring!");
            return;
    }

    if (!imu.begin()) {   // hardware I2C mode, can pass in address & alt Wire
            printk("Could not find a valid BMX160 sensor, check wiring!");
            return;
    }

	//k_timer_init (struct k_timer *timer, k_timer_expiry_t expiry_fn, k_timer_stop_t stop_fn)
	k_timer_start(&sensor_timer, K_MSEC(100), K_MSEC(100));
}

/*static int create_sensor_thread(void)
{
	int ret;

	sensor_thread_id = k_thread_create(
		&sensor_thread_data, sensor_thread_stack,
		CONFIG_BUTTON_MSG_SUB_STACK_SIZE, (k_thread_entry_t)update_sensors_task, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_BUTTON_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(sensor_thread_id, "SENSOR_THRD");
	if (ret) {
		LOG_ERR("Failed to create sensor thread");
		return ret;
	}
    return 0;
}*/