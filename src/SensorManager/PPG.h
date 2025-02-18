#ifndef _PPG_H
#define _PPG_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

//#include "MAX30102/MAX30102.h"
#include "MAXM86161/MAXM86161.h"
#include "EdgeMLSensor.h"

#include "openearable_common.h"
#include "zbus_common.h"

enum led_order {
    red, green, ir, ambient
};

class PPG : public EdgeMlSensor {
public:
    //PulseOximeter(int _samplerate);

    static PPG sensor;

    bool init(struct k_msgq * queue) override;
    void start(k_timeout_t t) override;
    void stop() override;

    void reset();
private:
    static MAXM86161 ppg;

    static void sensor_timer_handler(struct k_timer *dummy);

    static void update_sensor(struct k_work *work);

    ppg_sample data_buffer[64];

    bool _active = false;
};

#endif