#ifndef _EXG_H
#define _EXG_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <array>
#include <utility>

#include "AD7124/AD7124.h"
#include "EdgeMLSensor.h"

#include "openearable_common.h"
#include "zbus_common.h"

class ExG : public EdgeMlSensor {
public:
    static ExG sensor;

    bool init(struct k_msgq * queue) override;
    void start(int sample_rate_idx) override;
    void stop() override;

    const static SampleRateSetting<8> sample_rates;

private:
    static AD7124 *adc;

    static void sensor_timer_handler(struct k_timer *dummy);
    static void update_sensor(struct k_work *work);

    bool _active = false;
};

#endif // _EXG_H
