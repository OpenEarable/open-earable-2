#ifndef EDGE_ML_SENSOR_H
#define EDGE_ML_SENSOR_H

#include <zephyr/kernel.h>

template <size_t N>
struct SampleRateSetting {
    uint8_t reg_vals[N];
    float sample_rates[N];
    float true_sample_rates[N];
};

class EdgeMlSensor {
public:
    virtual bool init() = 0;
    virtual void start(int sample_rate_idx) = 0;
    virtual void stop() = 0;

    bool is_running() {
        return _running;
    }

    void sd_logging(bool enable) {
        _sd_logging = enable;
    }

    void ble_stream(bool enable) {
        _ble_stream = enable;
    }

protected:
    k_work sensor_work;
    k_timer sensor_timer;

    bool _sd_logging = false;
    bool _ble_stream = true;
    bool _running = false;
};

#endif
