#include "BiQuadFilter.h"

#include <utility>

#include "math.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(biquad, CONFIG_AUDIO_DATAPATH_LOG_LEVEL);

BiQuadFilter::BiQuadFilter() : num_stages(1) {
    reset_filter();
    
    // Initialize coefficients with default values (passthrough)
    for (int i = 0; i < MAX_FILTER_ORDER; i++) {
        c[i][0] = 1.0f;  // b0
        c[i][1] = 0.0f;  // b1
        c[i][2] = 0.0f;  // b2
        c[i][3] = 0.0f;  // a1
        c[i][4] = 0.0f;  // a2
    }
}

BiQuadFilter::BiQuadFilter(int num_stages) : num_stages(num_stages > MAX_FILTER_ORDER ? MAX_FILTER_ORDER : num_stages) {
    if (num_stages > MAX_FILTER_ORDER) {
        LOG_WRN("Number of stages exceeds MAX_FILTER_ORDER, setting to MAX_FILTER_ORDER");
    }

    // Initialize coefficients and buffers
    reset_filter();
    
    // Initialize coefficients with default values (passthrough)
    for (int i = 0; i < MAX_FILTER_ORDER; i++) {
        c[i][0] = 1.0f;  // b0
        c[i][1] = 0.0f;  // b1
        c[i][2] = 0.0f;  // b2
        c[i][3] = 0.0f;  // a1
        c[i][4] = 0.0f;  // a2
    }
}

void BiQuadFilter::reset_filter() {
    #pragma unroll
    for(int i = 0; i < num_stages; i++) {
        eq_buffer[i][0] = 0;
        eq_buffer[i][1] = 0;
    }
    
    // Clear output buffer
    for(int i = 0; i <= MAX_FILTER_ORDER; i++) {
        y[i] = 0;
    }
}

/**
 * Set filter coefficients for the biquad filter.
 * Coefficients should be provided in the format:
 * c[i][0] = b0, c[i][1] = b1, c[i][2] = b2, c[i][3] = a1, c[i][4] = a2
 * where i is the stage index.
 */
void BiQuadFilter::set_coefficients(float coefficients[][5], int stages) {
    int stages_to_set = (stages > num_stages) ? num_stages : stages;
    
    for (int i = 0; i < stages_to_set; i++) {
        for (int j = 0; j < 5; j++) {
            c[i][j] = coefficients[i][j];
        }
    }
    
    LOG_DBG("Biquad filter coefficients set for %d stages", stages_to_set);
}

void BiQuadFilter::apply(int16_t * data, int length) {
    for (int n = 0; n < length; n+=2) {
        y[0] = data[n];

        #pragma unroll
        for (int k = 0; k < num_stages; k++) {
            y[k+1] = c[k][0] * y[k] + eq_buffer[k][0];
            eq_buffer[k][0] = c[k][1] * y[k] - c[k][3] * y[k+1] + eq_buffer[k][1];
            eq_buffer[k][1] = c[k][2] * y[k] - c[k][4] * y[k+1];
        }

        data[n] = y[num_stages]; //CLAMP(y[EQ_ORDER],-1*(1<<15),1*(1<<15)-1);
    }
}