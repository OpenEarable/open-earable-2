#include "PulseOximeter.h"

#include "math.h"
#include "stdlib.h"

PulseOximeter::PulseOximeter(int _samplerate) : samplerate(_samplerate) { //, a_m(4.f / _samplerate) {
    avg_RR = _samplerate;
}

bool PulseOximeter::init() {
    return true;
}

void PulseOximeter::reset() {
    // Reset pulse oximeter state
}

void PulseOximeter::processPPG(int32_t red, int32_t ir) {
    // Process incoming PPG signal (red and infrared)
    //if adjustement

    t++;

    lpf_red.update(&red, 1);

    //printk(" %i\n", red);

    if (t <= 400) {
        x_m.write(red - avg_amp);
        return;
    }

    x_diff.write(red - red_vals.read());
    red_vals.write(red);

    //printk(" %f\n", red_vals.read());

    state.int_x += (red - x_m.read()) / samplerate; // avg_RR;
    state.int_x *= (1.f-1e-3);

    float a_m_RR = a_m / samplerate;

    float d_base;
    if (bases.read() > peaks.read())
        d_base = x_bases.read() - avg_amp / 2 - x_m.read();
    else
        d_base = x_peaks.read() + avg_amp / 2 - x_m.read();
    if (d_base * d_base < var_amp) x_m.adjust(x_m.read() + d_base * 0.004);

    x_m.write((1 - a_m_RR) * x_m.read() + a_m_RR * red + 0.01 * state.int_x); // + 1.f * x_diff.read() / samplerate); //avg_RR

    if (abs(x_m.read() - red) > 4 * avg_amp) {
        printk("Baseline jump\n");
        x_m.adjust(red);
        state.int_x = 0;
        //state.last_det = -1;
        //state.adjust = true;
        return;
    }

    if (t - state.last_R > 2 * avg_RR) {
        printk("No beat\n");
        x_m.adjust(red);
        state.int_x = 0;
        state.last_R = t;
        //state.last_det = -1;
        //state.adjust = true;
        //state.init = false;
        var_rise *= 1.1f;
        var_amp *= 1.1f;
        var_base *= 1.1f;
        avg_RR = 0.95 * avg_RR + 0.05 * samplerate;
        //var_rise = 0.95 * var_rise + 0.05 * (avg_rise / 2);
        avg_rise = 0.95 * avg_rise + 0.05 * (avg_RR * 0.2);
        return;
    }

    //printk(" %f, %f, %f\n", red_vals.read(), x_m.read(), avg_RR);
    //printk(" %f, %f\n", red_vals.read(), x_m.read());
    //printk(" %f, %f, %i, %i, %f, %f\n", red_vals.read(), x_m.read(), state.last_det, state.adjust, avg_amp, avg_RR);

    if (state.adjust) {
        // check end of adjustment phase
        if (state.last_det * red >= state.last_det * x_m.read() && t - state.last_cross > 0.3 * avg_RR) {
            state.adjust = false;
            state.last_cross = t;
            // end of peak adjustment
            if (state.last_det == 1) {
                float _RR = RR.read();
                float amp = x_bases.read() - x_peaks.read();
                float rise = peaks.read() - state.last_R;
                float RR_b = bases.read() - bases.read(-1);
                float RR_p = peaks.read() - peaks.read(-1);
                float base_drift = bases.read() - bases.read(-1);
                if (state.init) {
                    //if (amp > avg_amp / 4 && amp < 4 * avg_amp) {
                    if (_RR < 2 * avg_RR && _RR > avg_RR / 2
                    && (RR_b -_RR) * (RR_b -_RR) < _RR * _RR / 32 && (RR_p -_RR) * (RR_p -_RR) < _RR * _RR / 64
                    && (amp - avg_amp) * (amp - avg_amp) < 9 * var_amp
                    && (rise - avg_rise) * (rise - avg_rise) < 9 * var_rise
                    && base_drift * base_drift < 9 * var_base) {
                        var_rise = (1 - a_avg) * var_rise + a_avg * (rise - avg_rise) * (rise - avg_rise);
                        avg_rise = (1 - a_avg) * avg_rise + a_avg * rise;
                        var_amp = (1 - a_avg) * var_amp + a_avg * (amp - avg_amp) *  (amp - avg_amp);
                        avg_amp = (1 - a_avg) * avg_amp + a_avg * amp;
                        //printk("valid update\n");
                    } else {
                        var_rise *= 1.05f;
                        //var_rise = 0.95 * var_rise + 0.05 * (avg_rise / 2);
                        var_amp *= 1.05f;
                        avg_rise = 0.95 * avg_rise + 0.05 * (avg_RR * 0.2);
                    }
                } else if (peaks.index > 1 && t > peaks.read() && peaks.read(-1) > 0) {
                    avg_RR = peaks.read() - peaks.read(-1);
                    avg_rise = rise;
                    avg_amp = amp;
                    var_rise = (avg_rise / 2) * (avg_rise / 2);
                    var_amp = avg_amp * avg_amp; //(avg_amp / 3) * (avg_amp / 3);
                    var_base = (avg_amp / 2) * (avg_amp / 2);
                    state.init = true;
                }
            }
            // end of base adjustment ==> crossing of x_m
            else {
                if (state.init) {
                    int _RR = t - state.last_R;
                    //if (_RR < 2 * avg_RR && _RR > avg_RR / 2) 
                    //RR.write((float)_RR / samplerate * 1000);
                    RR.write(_RR);
                    // / 2 ?
                    //printk("Heart rate: %f, RR: %f\n", (float) samplerate / _RR * 60, (float)_RR / samplerate * 1000);
                    float amp = x_bases.read() - x_peaks.read();
                    float rise = peaks.read() - state.last_R;
                    float RR_b = bases.read() - bases.read(-1);
                    float RR_p = peaks.read() - peaks.read(-1);
                    float base_drift = bases.read() - bases.read(-1);
                    if (_RR < 2 * avg_RR && _RR > avg_RR / 2
                    && (RR_b -_RR) * (RR_b -_RR) < _RR * _RR / 32 && (RR_p -_RR) * (RR_p -_RR) < _RR * _RR / 64
                    && (amp - avg_amp) * (amp - avg_amp) < 9 * var_amp
                    && (rise - avg_rise) * (rise - avg_rise) < 9 * var_rise
                    && base_drift * base_drift < 9 * var_base
                    ) {
                        printk("Heart rate: %f, RR: %f, avgRR: %f, amp: %f\n",
                        (float) samplerate / _RR * 60, (float)_RR / samplerate * 1000, (float)avg_RR / samplerate * 1000, avg_amp);
                        var_base = (1 - a_avg) * var_base + a_avg * base_drift * base_drift;
                        avg_RR = (1 - a_avg) * avg_RR + a_avg * _RR;
                    } else {
                        var_base *= 1.05f;
                        avg_RR = 0.95 * avg_RR + 0.05 * samplerate;
                        //printk()
                    }
                }
                
                state.last_R = t;
            }
        } else {
            // adjust base
            if (state.last_det == -1 && red_vals.read() > x_bases.read() - avg_amp * 0.05 && x_diff.read() >= 0) {
                bases.adjust(t);
                x_bases.adjust(red);
            }
            // adjust peak
            if (state.last_det == 1 && red_vals.read() < x_peaks.read() && (!state.init || t - state.last_cross < avg_rise + 2 * sqrtf(var_rise))) { //peaks.read()
                peaks.adjust(t);
                x_peaks.adjust(red);
            }
        }
    } else {
        // detect peak
        if (state.last_det == -1 && x_diff.read() >= 0 && x_diff.read(-1) < 0 && x_bases.read() - red_vals.read(-1) > avg_amp - 2 * sqrtf(var_amp) - 1 * sqrtf(var_base)) {
            state.last_det = 1;
            state.adjust = true;
            peaks.write(t-1);
            x_peaks.write(red);
        }
        //detect base
        else if (state.last_det == 1 && x_diff.read() <= 0 && x_diff.read(-1) > 0 && red_vals.read(-1) - x_peaks.read() > avg_amp - 2 * sqrtf(var_amp) - 2 * sqrtf(var_base)) {
            state.last_det = -1;
            state.adjust = true;
            bases.write(t-1);
            x_bases.write(red);
        }
    }
}

bool PulseOximeter::hasSkinContact() {
    // Check if the pulse oximeter has skin contact
    return true; // Implement according to your hardware and requirements
}

float PulseOximeter::heartRate() {
    // Calculate and return heart rate
    return 0.0; // Implement according to your algorithm
}

float PulseOximeter::SpO2Level() {
    // Calculate and return SpO2 level
    return 0.0; // Implement according to your algorithm
}

float PulseOximeter::lastRR() {
    // Return last RR interval
    return 0.0; // Implement according to your algorithm
}