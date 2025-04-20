#ifndef MICRO_H
#define MICRO_H

#include "EdgeMLSensor.h"

#include "openearable_common.h"

class Microphone : public EdgeMlSensor {
public:
    static Microphone sensor;

    bool init(struct k_msgq * queue) override;
    void start(int sample_rate_idx) override;
    void stop() override;

    const static SampleRateSetting<1> sample_rates;
private:
    bool _active = false;
};

#endif