#ifndef _TEMP_H
#define _TEMP_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "MLX90632/MLX90632.h"
#include "EdgeMLSensor.h"

#include "openearable_common.h"
#include "zbus_common.h"

class Temp : public EdgeMlSensor {
public:
    static Temp sensor;

    bool init(struct k_msgq * queue) override;
    void start(int sample_rate_idx) override;
    void stop() override;

    const static SampleRateSetting<8> sample_rates;
private:
    static MLX90632 temp;

    static void sensor_timer_handler(struct k_timer *dummy);

    static void update_sensor(struct k_work *work);

    bool _active = false;
};

#endif