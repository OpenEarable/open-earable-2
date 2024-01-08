#ifndef IMU_H
#define IMU_H

#include "EdgeMlSensor.h"

#include "nrf5340_audio_common.h"
#include "BMX160/DFRobot_BMX160.h"

class IMU : public EdgeMlSensor {
public:
    static IMU sensor;

    bool init(struct k_msgq * queue) override;
    void start(k_timeout_t t) override;
    void stop() override;
private:
    //static k_work sensor_work;
    //static k_msgq * sensor_queue;

    static DFRobot_BMX160 imu;

    static void sensor_timer_handler(struct k_timer *dummy);

    static void update_sensor(struct k_work *work);
    //static void sensor_timer_handler(struct k_timer *dummy);
};

#endif