#ifndef _TEMP_LEFT_I2C1_H
#define _TEMP_LEFT_I2C1_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "MLX90632/MLX90632.h"
#include "EdgeMLSensor.h"

#include "openearable_common.h"
#include "zbus_common.h"

class Temp_left_I2C1 : public EdgeMlSensor {
public:
    static Temp_left_I2C1 sensor;

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
