#ifndef OPEN_EARABLE_BQFILTER_H
#define OPEN_EARABLE_BQFILTER_H

#include <zephyr/kernel.h>

#define MAX_FILTER_ORDER 10

class BiQuadFilter {
public:
    BiQuadFilter();
    BiQuadFilter(int num_stages);
    void reset_filter();
    void apply(int16_t * data, int length);
    void set_coefficients(float coefficients[][5], int stages);

private:
    const int num_stages = 1;
    float c[MAX_FILTER_ORDER][5];          // Filter coefficients for each stage
    float eq_buffer[MAX_FILTER_ORDER][2];   // Filter state for each stage
    float y[MAX_FILTER_ORDER + 1];          // Temporary buffer for filter output
};

#endif //OPEN_EARABLE_AUDIO_PLAYER_H
