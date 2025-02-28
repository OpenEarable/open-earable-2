#ifndef EDGE_ML_SENSOR_H
#define EDGE_ML_SENSOR_H

#include <zephyr/kernel.h>

struct sample_rate_setting {
    int reg_val;
    float sample_rate; 
};

class EdgeMlSensor {
public:
    virtual bool init(struct k_msgq * queue) = 0;
    // TODO: set the sample rate: 
    virtual void start(int sample_rate_idx) = 0;
    virtual void stop() = 0;
    //virtual void end() = 0;
    //virtual void update_sensor(struct k_work * work) = 0;

    void sd_logging(bool enable) {
        _sd_logging = enable;
    }

    void ble_stream(bool enable) {
        _ble_stream = enable;
    }

protected:
    k_work sensor_work;
    k_timer sensor_timer;
    
    static k_msgq * sensor_queue;

    bool _sd_logging = false;
    bool _ble_stream = true;
};

#endif