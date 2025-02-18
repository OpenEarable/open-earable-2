#include "Equalizer.h"

#include <utility>

#include "math.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(equalizer, CONFIG_AUDIO_DATAPATH_LOG_LEVEL);

// Fix point arithmetic 5.27
#define Q27_MULT(a, b) (((int64_t)(a) * (int64_t)(b)) >> 27)

static const int32_t c[EQ_ORDER][5] = {
    {(int32_t) 0x07F46AB0, (int32_t) 0xF05C87C0, (int32_t) 0x07AFA1BC, (int32_t) 0xF05CC168, (int32_t) 0x07A44615}, // f:   150 G:  -5.0dB Q: 0.5 (shelving)
    {(int32_t) 0x07ED1CFE, (int32_t) 0xF03F890A, (int32_t) 0x07D420FF, (int32_t) 0xF03F890A, (int32_t) 0x07C13DFD}, // f:   150 G:  -8.0dB Q: 1.0
    {(int32_t) 0x081B095D, (int32_t) 0xF08F4C0C, (int32_t) 0x0761E390, (int32_t) 0xF08F4C0C, (int32_t) 0x077CECED}, // f:   600 G:   3.0dB Q: 1.0 
    {(int32_t) 0x0768EE95, (int32_t) 0xF3A4F7B5, (int32_t) 0x067505B4, (int32_t) 0xF3A4F7B5, (int32_t) 0x05DDF449}, // f:  3600 G:  -7.0dB Q: 2.3 
    {(int32_t) 0x076FC198, (int32_t) 0xF6E18719, (int32_t) 0x040EC81C, (int32_t) 0xF6E18719, (int32_t) 0x037E89B4}, // f:  5000 G:  -2.5dB Q: 1.0 
    {(int32_t) 0x073E5C38, (int32_t) 0xF5A24BD8, (int32_t) 0x068B40C5, (int32_t) 0xF5A24BD8, (int32_t) 0x05C99CFD}, // f:  5500 G: -10.0dB Q: 4.0 
    {(int32_t) 0x074F07EC, (int32_t) 0xFAF010E0, (int32_t) 0x05EB6872, (int32_t) 0xFAF010E0, (int32_t) 0x053A705E}, // f:  9000 G:  -6.0dB Q: 4.0 
    {(int32_t) 0x071131FF, (int32_t) 0x0199A84F, (int32_t) 0x05314FF6, (int32_t) 0x0199A84F, (int32_t) 0x044281F5}, // f: 13000 G:  -6.0dB Q: 4.0 
    {(int32_t) 0x06067A87, (int32_t) 0xFDFCCCAC, (int32_t) 0x01272C9B, (int32_t) 0xFB4CB72E, (int32_t) 0x01DDBCA0}  // f:  9000 G:  -4.0dB Q: 1.0 (shelving) 
};

int32_t amp = (int32_t) 0x05A9DF7B; //-3dB

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

        y[0] = Q27_MULT(amp, y[0]);

        #pragma unroll
        for (int k = 0; k < EQ_ORDER; k++) {
            y[k+1] = Q27_MULT(c[k][0], y[k]) + eq_buffer[k][0];
            eq_buffer[k][0] = Q27_MULT(c[k][1], y[k]) - Q27_MULT(c[k][3], y[k+1]) + eq_buffer[k][1];
            eq_buffer[k][1] = Q27_MULT(c[k][2], y[k]) - Q27_MULT(c[k][4], y[k+1]);
        }

        data[n] = (int16_t) CLAMP(y[EQ_ORDER] >> 16,-1*(1<<15),1*(1<<15)-1);
    }
}