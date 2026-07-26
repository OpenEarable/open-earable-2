#ifndef IMU_H
#define IMU_H

#include "EdgeMLSensor.h"

#include "openearable_common.h"
#include "BMX160/BMX160_Bosch.h"

class IMU : public EdgeMlSensor {
public:
    static IMU sensor;

    bool init(struct k_msgq * queue) override;
    void start(int sample_rate_idx) override;
    void stop() override;
    static void setBenchmarkEnabled(bool enabled);

    const static SampleRateSetting<6> sample_rates;
private:
    static BMX160Bosch imu;

    static constexpr uint8_t MAX_BUFFERED_SAMPLES = BMX160Bosch::MAX_FIFO_SAMPLES;
    BMX160Sample sample_buffer[MAX_BUFFERED_SAMPLES] = {};
    float t_sample_us = 10000.0f;
    uint8_t _num_samples_buffered = 1;
    uint64_t _benchmark_started_us = 0;
    uint64_t _processing_time_us = 0;
    uint32_t _samples_read = 0;
    uint32_t _batches_read = 0;
    bool _benchmark_enabled = false;

    //const static int num_sample_rates = 6;
    //const static sample_rate_setting sample_rates[num_sample_rates];

    static void sensor_timer_handler(struct k_timer *dummy);

    static void update_sensor(struct k_work *work);
    void logBenchmark(uint64_t now_us);
    bool _active = false;
};

#endif
