#ifndef EDGE_ML_SENSOR_H
#define EDGE_ML_SENSOR_H

#include <zephyr/kernel.h>

class EdgeMlSensor {
public:
    //virtual static EdgeMlSensor sensor;
    virtual bool init(struct k_msgq * queue) = 0;
    virtual void start(k_timeout_t t) = 0;
    virtual void stop() = 0;
    //virtual void update_sensor(struct k_work * work) = 0;

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
};

#endif