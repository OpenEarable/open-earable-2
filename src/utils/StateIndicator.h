#ifndef _STATE_INDICATOR_H
#define _STATE_INDICATOR_H

#include "openearable_common.h"

class StateIndicator {
public:
    void init(struct earable_state state);

    void set_state(struct earable_state state);

    void set_charging_state(enum charging_state state);
    void set_pairing_state(enum pairing_state state);
    void set_led_state(enum led_state state);

private:
    earable_state _state;
};

extern StateIndicator state_indicator;

#endif