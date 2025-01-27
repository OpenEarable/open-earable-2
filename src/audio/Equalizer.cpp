#include "Equalizer.h"

#include <utility>

#include "math.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(equalizer, CONFIG_AUDIO_DATAPATH_LOG_LEVEL);

#define Q27_MULT(a, b) (((int64_t)(a) * (int64_t)(b)) >> 27)

/*
static const float a[EQ_ORDER][3] = {
    {1.000000000000000, -1.990844014663719, 0.990890061149213},
    {1.000000000000000, -1.962529035911005, 0.962907404996144},
    {1.000000000000000, -1.921473171685143, 0.925596020576637},
    {1.000000000000000, -1.544449409119988, 0.733376094554248},
    {1.000000000000000, -1.121170890228473, 0.413204978524321},
    {1.000000000000000, -1.050022035983737, 0.668501531043800},
    {1.000000000000000, -0.152825855141626, 0.462050147530658},
    {1.000000000000000, -0.500598440492308, 0.216225368070061}
};

static const float b[EQ_ORDER][3] = {
    {1.001528139184109, -1.990826096147937, 0.989379840480887},
    {0.995361463599325, -1.962529035911005, 0.967545941396819},
    {1.004539329281577, -1.921473171685143, 0.921056691295060},
    {0.926236309196037, -1.544449409119988, 0.807139785358210},
    {0.902692930993568, -1.121170890228473, 0.510512047530753},
    {0.886665275659910, -1.050022035983737, 0.781836255383889},
    {0.784692627164086, -0.152825855141626, 0.677357520366571},
    {0.671454248995156, -0.072411623830529, 0.116584302413126}
};*/

static const int32_t c[EQ_ORDER][5] = {
    {(int32_t) 0x0803212F, (int32_t) 0xF012C9C5, (int32_t) 0x07EA3FFA, (int32_t) 0xF012C060, (int32_t) 0x07ED57C5},
    {(int32_t) 0x07F68012, (int32_t) 0xF04CBD94, (int32_t) 0x07BD88BA, (int32_t) 0xF04CBD94, (int32_t) 0x07B408CC},
    {(int32_t) 0x08094BEA, (int32_t) 0xF0A0D2AC, (int32_t) 0x075E52F8, (int32_t) 0xF0A0D2AC, (int32_t) 0x07679EE3},
    {(int32_t) 0x0768EE95, (int32_t) 0xF3A4F7B5, (int32_t) 0x067505B4, (int32_t) 0xF3A4F7B5, (int32_t) 0x05DDF449},
    {(int32_t) 0x0738B712, (int32_t) 0xF707D78E, (int32_t) 0x04158757, (int32_t) 0xF707D78E, (int32_t) 0x034E3E69},
    {(int32_t) 0x0717E3F7, (int32_t) 0xF7998E0C, (int32_t) 0x0641335E, (int32_t) 0xF7998E0C, (int32_t) 0x05591755},
    {(int32_t) 0x06470CEE, (int32_t) 0xFEC7033D, (int32_t) 0x056B3A6B, (int32_t) 0xFEC7033D, (int32_t) 0x03B24759},
    {(int32_t) 0x055F2368, (int32_t) 0xFF6BB374, (int32_t) 0x00EEC3C0, (int32_t) 0xFBFEC63F, (int32_t) 0x01BAD45E}
};

//static const int32_t * c = filter_coeff;

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

        data[n] = (int16_t) CLAMP(y[EQ_ORDER] >> (16 + 1),-1*(1<<15),1*(1<<15)-1); //-6dB
    }
}