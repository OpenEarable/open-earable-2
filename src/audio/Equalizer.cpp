#include "Equalizer.h"

#include <utility>

#include "math.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(equalizer, CONFIG_AUDIO_DATAPATH_LOG_LEVEL);

#define Q27_MULT(a, b) (((int64_t)(a) * (int64_t)(b)) >> 27)

static const int32_t c[EQ_ORDER][5] = {
    {(int32_t) 0x0803F78C, (int32_t) 0xF00D95E5, (int32_t) 0x07EE8F4C, (int32_t) 0xF00D8D3F, (int32_t) 0x07F27E32},
    {(int32_t) 0x07F496D7, (int32_t) 0xF04EEA81, (int32_t) 0x07BD44EC, (int32_t) 0xF04EEA81, (int32_t) 0x07B1DBC3},
    {(int32_t) 0x08094BEA, (int32_t) 0xF0A0D2AC, (int32_t) 0x075E52F8, (int32_t) 0xF0A0D2AC, (int32_t) 0x07679EE3},
    {(int32_t) 0x0768EE95, (int32_t) 0xF3A4F7B5, (int32_t) 0x067505B4, (int32_t) 0xF3A4F7B5, (int32_t) 0x05DDF449},
    {(int32_t) 0x0738B712, (int32_t) 0xF707D78E, (int32_t) 0x04158757, (int32_t) 0xF707D78E, (int32_t) 0x034E3E69},
    {(int32_t) 0x0717E3F7, (int32_t) 0xF7998E0C, (int32_t) 0x0641335E, (int32_t) 0xF7998E0C, (int32_t) 0x05591755},
    {(int32_t) 0x06470CEE, (int32_t) 0xFEC7033D, (int32_t) 0x056B3A6B, (int32_t) 0xFEC7033D, (int32_t) 0x03B24759},
    {(int32_t) 0x055F2368, (int32_t) 0xFF6BB374, (int32_t) 0x00EEC3C0, (int32_t) 0xFBFEC63F, (int32_t) 0x01BAD45E}
};

int32_t amp = (int32_t) 0x032F52D0; //-8dB

int64_t eq_buffer[EQ_ORDER][2] = {0};
int64_t y[EQ_ORDER+1] = {0};

void reset_eq() {
    #pragma unroll
    for(int i = 0; i < EQ_ORDER; i++) {
        eq_buffer[i][0] = 0;
        eq_buffer[i][1] = 0;
    }
}

void equalize(int16_t * data, int length) {
    for (int n = 0; n < length; n+=2) {
        y[0] = (int64_t)(data[n]) << 16;

        #pragma unroll
        for (int k = 0; k < EQ_ORDER; k++) {
            y[k+1] = Q27_MULT(c[k][0], y[k]) + eq_buffer[k][0];
            eq_buffer[k][0] = Q27_MULT(c[k][1], y[k]) - Q27_MULT(c[k][3], y[k+1]) + eq_buffer[k][1];
            eq_buffer[k][1] = Q27_MULT(c[k][2], y[k]) - Q27_MULT(c[k][4], y[k+1]);
        }

        data[n] = (int16_t) CLAMP(Q27_MULT(amp, y[EQ_ORDER]) >> 16,-1*(1<<15),1*(1<<15)-1);
    }
}