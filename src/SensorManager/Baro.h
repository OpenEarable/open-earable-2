#ifndef BARO_H
#define BARO_H

#include "EdgeMlSensor.h"

#include "nrf5340_audio_common.h"
#include "BMP388/Adafruit_BMP3XX.h"

class Baro : public EdgeMlSensor {
public:
    static Baro sensor;

    bool init(struct k_msgq * queue) override;
    void start(k_timeout_t t) override;
    void stop() override;
private:
    /*static k_work sensor_work;
    static k_msgq * sensor_queue;*/

    static void sensor_timer_handler(struct k_timer *dummy);

    static Adafruit_BMP3XX bmp;

    static void update_sensor(struct k_work *work);
};

#endif