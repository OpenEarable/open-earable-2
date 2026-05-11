#ifndef IMU_H
#define IMU_H

#include "EdgeMLSensor.h"

#include "openearable_common.h"
#include "BMX160/BMX160_Sensor.h"

class IMU : public EdgeMlSensor {
public:
    static IMU sensor;

    bool init() override;
    void start(int sample_rate_idx) override;
    void stop() override;

    const static SampleRateSetting<6> sample_rates;
private:
    static BMX160 imu;

    static void sensor_timer_handler(struct k_timer *dummy);
    static void update_sensor(struct k_work *work);

    bool _active = false;
    uint32_t t_sample_us = 0;
    int _num_samples_buffered = 1;

    static BMX160Sample _batch[64];
};

#endif
