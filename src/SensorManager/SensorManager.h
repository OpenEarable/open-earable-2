#ifndef _SENSOR_MANAGER_H
#define _SENSOR_MANAGER_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include "BMP388/Adafruit_BMP3XX.h"
#include "BMX160/DFRobot_BMX160.h"

//#include "nrfx"

#define I2C_DEV_LABEL "I2C_1" // Change this to your I2C device label

class SensorManager {
public:
    static SensorManager manager;

    void start();
    void stop();
private:
    static struct k_thread sensor_thread_data;
    static k_tid_t sensor_thread_id;
    //K_THREAD_STACK_DEFINE(sensor_thread_stack, 1024);
    //K_WORK_DEFINE(sensor_work, update_sensors);
    k_work sensor_work = Z_WORK_INITIALIZER(update_sensors);
    //K_TIMER_DEFINE(sensor_timer, sensor_timer_handler, NULL);
    k_timer sensor_timer = Z_TIMER_INITIALIZER(sensor_timer, sensor_timer_handler, NULL);

    Adafruit_BMP3XX bmp;
    DFRobot_BMX160 imu;

    static void update_sensors(struct k_work *work);
    static void sensor_timer_handler(struct k_timer *dummy);

};

#endif