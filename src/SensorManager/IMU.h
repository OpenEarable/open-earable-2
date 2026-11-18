#ifndef IMU_H
#define IMU_H

#include "EdgeMLSensor.h"

#include "openearable_common.h"
#include "BMX160/BMX160.h"
#include "FifoSampleClock.h"

class IMU : public EdgeMlSensor {
public:
    static IMU sensor;

    bool init(struct k_msgq * queue) override;
    void start(int sample_rate_idx) override;
    void stop() override;
    /** Probe with the sensor rail powered; return 0x68/0x69, or zero on failure. */
    static uint8_t detectHardwareRevision();

    const static SampleRateSetting<6> sample_rates;
private:
    static BMX160 imu;

    static constexpr uint8_t MAX_BUFFERED_SAMPLES = BMX160::MAX_FIFO_SAMPLES;
    BMX160Sample sample_buffer[MAX_BUFFERED_SAMPLES] = {};
    float t_sample_us = 10000.0f;
    FifoSampleClock sample_clock;

    static void sensor_timer_handler(struct k_timer *dummy);

    static void update_sensor(struct k_work *work);
    bool _active = false;
};

#endif
