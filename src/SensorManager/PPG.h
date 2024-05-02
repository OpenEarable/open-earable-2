#ifndef _PPG_H
#define _PPG_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "MAX30102/MAX30102.h"
#include "EdgeMlSensor.h"

#include "nrf5340_audio_common.h"

class PPG : public EdgeMlSensor {
public:
    //PulseOximeter(int _samplerate);

    static PPG sensor;

    bool init(struct k_msgq * queue) override;
    void start(k_timeout_t t) override;
    void stop() override;

    void reset();
private:
    static MAX30105 ppg;

    static void sensor_timer_handler(struct k_timer *dummy);

    static void update_sensor(struct k_work *work);
};

#endif