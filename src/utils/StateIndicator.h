#ifndef _STATE_INDICATOR_H
#define _STATE_INDICATOR_H

#include "nrf5340_audio_common.h"

class StateIndicator {
public:
    void init(struct earable_state state);

    void set_state(struct earable_state state);

    void set_charging_state(enum charging_state state);
    void set_pairing_state(enum pairing_state state);

private:
    earable_state _state;
};

extern StateIndicator state_indicator;

#endif