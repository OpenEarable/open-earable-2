#include "Equalizer.h"

#include <utility>

#include "math.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(equalizer, CONFIG_AUDIO_DATAPATH_LOG_LEVEL);


/*
static const float a[EQ_ORDER][3] = {
    {1.000000000000000, -1.955030104871120, 0.955653879248517},
    {1.000000000000000, -1.232336077107481, 0.307322124851148},
    //{1.000000000000000, -0.647226652765832, 0.294453305531665}
};

static const float b[EQ_ORDER][3] = {
    {1.015334727647332, -1.954462974530710, 0.940886281941595},
    {0.808364983163067, -1.232336077107481, 0.498957141688081},
    //{1.043044858548443, -0.647226652765832, 0.251408446983222}
};*/

static const float a[EQ_ORDER][3] = {
    {1.000000000000000, -1.938299642346885, 0.938918078681754},
    {1.000000000000000, -0.860182182050345, -0.068945515318215},
    //{1.000000000000000, -0.647226652765832, 0.294453305531665}
};

static const float b[EQ_ORDER][3] = {
    {1.021012465331586, -1.937737365309185, 0.918467890387869},
    {0.704267453638894, -0.860182182050344, 0.226787031042891},
    //{1.043044858548443, -0.647226652765832, 0.251408446983222}
};

float eq_buffer[EQ_ORDER][2] = {0};

void reset_eq() {
    #pragma unroll
    for(int i = 0; i < EQ_ORDER; i++) {
        eq_buffer[i][0] = 0;
        eq_buffer[i][1] = 0;
    }
}

void equalize(int16_t * data, int length) {
    float y[EQ_ORDER+1] = {0};
    for (int n = 0; n < length; n+=2) {
        y[0] = data[n];

        #pragma unroll
        for (int k = 0; k < EQ_ORDER; k++) {
            y[k+1] = b[k][0] * y[k] + eq_buffer[k][0];
            eq_buffer[k][0] = b[k][1] * y[k] - a[k][1] * y[k+1] + eq_buffer[k][1];
            eq_buffer[k][1] = b[k][2] * y[k] - a[k][2] * y[k+1];
        }

        if (abs(0.4 * y[EQ_ORDER]) > 1*(1<<15)) LOG_WRN("Clip: %f", 0.4f * y[EQ_ORDER]);

        data[n] = (int16_t) CLAMP(0.4 * y[EQ_ORDER],-1*(1<<15),1*(1<<15)-1);
    }
}