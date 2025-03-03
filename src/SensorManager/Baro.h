#ifndef BARO_H
#define BARO_H

#include "EdgeMLSensor.h"

#include "openearable_common.h"
#include "zbus_common.h"

#include "BMP388/Adafruit_BMP3XX.h"

class Baro : public EdgeMlSensor {
public:
    static Baro sensor;

    bool init(struct k_msgq * queue) override;
    void start(int sample_rate_idx) override;
    void stop() override;

    const static SampleRateSetting<18> sample_rates;
private:
    static Adafruit_BMP3XX bmp;

    bool _active = false;

    static void update_sensor(struct k_work *work);

    static void sensor_timer_handler(struct k_timer *dummy);
};

#endif