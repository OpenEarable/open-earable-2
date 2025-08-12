#ifndef ANC_DAMPING_H
#define ANC_DAMPING_H

#include "EdgeMLSensor.h"
#include "openearable_common.h"

class ANCDamping : public EdgeMlSensor {
public:
    static ANCDamping sensor;

    bool init(struct k_msgq * queue) override;
    void start(int sample_rate_idx) override;
    void stop() override;

    const static SampleRateSetting<1> sample_rates;
    
    static void send_damping_data(float damping_value);

private:
    bool _active = false;
    static struct k_msgq * sensor_queue;
};

#endif
