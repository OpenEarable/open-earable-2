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

    void record(bool active);

    const static SampleRateSetting<9> sample_rates;
private:
    bool _active = false;
};

#ifdef __cplusplus
extern "C" {
#endif

// C wrapper functions for C code to call C++ methods
void microphone_start(int sample_rate_idx);
void microphone_stop(void);

#ifdef __cplusplus
}
#endif

#endif