#ifndef _PULSE_OXIMETER_H
#define _PULSE_OXIMETER_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "MAX30102.h"

#include "SOSFilter.h"

#define NUM_RR 4

struct CircularBuffer {
    const int size = NUM_RR;
    int index = 0;
    float data[NUM_RR] = {0};

    void write(float value) {
        index++;
        if (index >= size) index -= size;
        data[index] = value;
    };

    void adjust(float value) {
        data[index] = value;
    };

    float read(int offset = 0) {
        int _index = index + offset;
        if (_index < 0) _index += size;
        return data[_index];
    };
};

struct ppg_state {
    bool init = false;
    bool adjust = true;
    int last_det = -1;
    int last_cross = 0;
    int last_R = 0;
    float int_x = 0;
};

class PulseOximeter {
public:
    PulseOximeter(int _samplerate);

    bool init();
    void reset();
    void processPPG(int32_t red, int32_t ir);

    bool hasSkinContact();

    float heartRate();
    float SpO2Level();
    float lastRR();
private:
    static MAX30105 ppg;

    const int samplerate;
    const float a_m = 1.f;
    // data
    SOSFilter lpf_red;
    SOSFilter lpf_ir;

    CircularBuffer RR;
    CircularBuffer peaks;
    CircularBuffer bases;
    CircularBuffer x_peaks;
    CircularBuffer x_bases;
    CircularBuffer x_m;
    CircularBuffer x_diff;
    CircularBuffer red_vals;
    CircularBuffer ir_vals;

    long t = 0;

    float avg_RR = 0;
    float avg_rise = 0;
    float avg_amp = 500;

    float var_rise = 0;
    float var_amp = 10000.f;
    float var_base = 0;

    const float a_avg = 0.1;

    struct ppg_state state;
};

#endif