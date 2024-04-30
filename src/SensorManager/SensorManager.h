#ifndef _SENSOR_MANAGER_H
#define _SENSOR_MANAGER_H

#include "nrf5340_audio_common.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

//#include "BMP388/Adafruit_BMP3XX.h"
//#include "BMX160/DFRobot_BMX160.h"

#include "IMU.h"
#include "Baro.h"
#include "PPG.h"

#define I2C_DEV_LABEL "I2C_1" // Change this to your I2C device label

class SensorManager {
public:
    static SensorManager manager;

    void start();
    void stop();

    void config(sensor_config * config);
private:
    Baro baro = Baro::sensor;
    IMU imu = IMU::sensor;
    PPG ppg = PPG::sensor;
};

#endif