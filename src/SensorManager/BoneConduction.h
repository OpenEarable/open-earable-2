#ifndef _BONE_H
#define _BONE_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

//#include "MAX30102/MAX30102.h"
#include "BMA580/BMA580_Sensor.h"
#include "EdgeMLSensor.h"

#include "nrf5340_audio_common.h"

#define ACCEL_FRAME_LEN                   UINT8_C(255)

class BoneConduction : public EdgeMlSensor {
public:
    static BoneConduction sensor;

    bool init(struct k_msgq * queue) override;
    void start(k_timeout_t t) override;
    void stop() override;

    void reset();

private:
    BMA580 bma580;

    /*! Number of accel frames to be extracted from FIFO */
    uint8_t fifo_accel_frame_length = ACCEL_FRAME_LEN;

    /*! Array of accelerometer and sensortime frames
    * Array size same as fifo_accel_frame_length */
    bma5_sens_fifo_axes_data_16_bit fifo_acc_data[ACCEL_FRAME_LEN];

    static void sensor_timer_handler(struct k_timer *dummy);

    static void update_sensor(struct k_work *work);

    bool _active = false;
};

#endif