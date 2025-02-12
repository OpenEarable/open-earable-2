#ifndef _STATE_INDICATOR_H
#define _STATE_INDICATOR_H

#include "openearable_common.h"

class StateIndicator {
public:
    void init(struct earable_state state);

    void set_state(struct earable_state state);

    void set_charging_state(enum charging_state state);
    void set_pairing_state(enum pairing_state state);
    void set_indication_mode(enum led_mode state);
    void set_custom_color(RGBColor color);

private:
    earable_state _state;
    RGBColor color;
};

extern StateIndicator state_indicator;

#endif