#ifndef IMU_H
#define IMU_H

#include "EdgeMlSensor.h"

#include "openearable_common.h"
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

    static uint64_t system_time_us_ref;  // System micros() reference
    static uint64_t fifo_time_us_ref;    // FIFO timestamp reference
    static uint64_t last_fifo_time_us;   // Last known FIFO timestamp

    static void sensor_timer_handler(struct k_timer *dummy);

    static void update_sensor(struct k_work *work);
    
    static void sync_fifo_time(bool force = false);

    bool _active = false;
};

#endif