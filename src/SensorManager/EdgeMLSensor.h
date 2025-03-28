#ifndef EDGE_ML_SENSOR_H
#define EDGE_ML_SENSOR_H

#include <zephyr/kernel.h>

/*
#define _EXTRACT_SAMPLE_RATES(samplerates, i, num_samplerates) \
#if ((i) < num_samplerates) \
    "samplerates[" ## i ## "].sample_rate" ## "," ## _EXTRACT_SAMPLE_RATES(samplerates, (i+1), num_samplerates) \
#else \
    "}" \
#endif

#define EXTRACT_SAMPLE_RATES(samplerates, num_samplerates) "{" ## _EXTRACT_SAMPLE_RATES(samplerates, 0, num_samplerates)*/

/*struct sample_rate_setting {
    int reg_val;
    float sample_rate;
};*/

template <size_t N>
struct SampleRateSetting {
    uint8_t reg_vals[N];
    float sample_rates[N];
    float true_sample_rates[N];
};

class EdgeMlSensor {
public:
    //virtual static EdgeMlSensor sensor;
    virtual bool init(struct k_msgq * queue) = 0;
    virtual void start(int sample_rate_idx) = 0;
    virtual void stop() = 0;
    //virtual void end() = 0;
    //virtual void update_sensor(struct k_work * work) = 0;

    bool is_running() {
        return _running;
    }

    void sd_logging(bool enable) {
        _sd_logging = enable;
    }

    void ble_stream(bool enable) {
        _ble_stream = enable;
    }

    /**
    * @brief Submit a k_work on timer expiry.
    */
    /*void sensor_timer_handler(struct k_timer *dummy)
    {
        k_work_submit(&sensor_work);
    };*/

protected:
    k_work sensor_work;
    k_timer sensor_timer;
    static k_msgq * sensor_queue;

    bool _sd_logging = false;
    bool _ble_stream = true;
    bool _running = false;
};

#endif