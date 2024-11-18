#include "SOSFilter.h"

SOSFilter::SOSFilter() {
    //reset();
}

SOSFilter::~SOSFilter() {
    //reset();
}

void SOSFilter::reset() {
    #pragma unroll
    for(int i = 0; i < EQ_ORDER; i++) {
        buffer[i][0] = 0;
        buffer[i][1] = 0;
    }
}

void SOSFilter::update(int32_t * data, int length) {
    float y[EQ_ORDER+1] = {0};
    for (int n = 0; n < length; n++) {
        y[0] = data[n];

        #pragma unroll
        for (int k = 0; k < EQ_ORDER; k++) {
            y[k+1] = b[k][0] * y[k] + buffer[k][0];
            buffer[k][0] = b[k][1] * y[k] - a[k][1] * y[k+1] + buffer[k][1];
            buffer[k][1] = b[k][2] * y[k] - a[k][2] * y[k+1];
        }

        //data[n] = CLAMP(y[EQ_ORDER],-1*(1<<31),(1<<31)-1);
        data[n] = y[EQ_ORDER];
    }
}