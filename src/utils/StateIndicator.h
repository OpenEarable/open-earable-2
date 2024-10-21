#ifndef _STATE_INDICATOR_H
#define _STATE_INDICATOR_H

#include "nrf5340_audio_common.h"

class StateIndicator {
public:
    void init(enum earable_state state);

    void set_state(enum earable_state state);
};

extern StateIndicator state_indicator;

#endif