#ifndef _STATE_INDICATOR_H
#define _STATE_INDICATOR_H

#include "openearable_common.h"

static const RGBColor LED_OFF = {0, 0, 0};
static const RGBColor LED_RED = {32, 0, 0};
static const RGBColor LED_GREEN = {0, 32, 0};
static const RGBColor LED_BLUE = {0, 0, 32};
static const RGBColor LED_YELLOW = {16, 16, 0};
static const RGBColor LED_ORANGE = {24, 8, 0};
static const RGBColor LED_CYAN = {0, 16, 16};
static const RGBColor LED_MAGENTA = {16, 0, 16};

class StateIndicator {
public:
    void init(struct earable_state state);

    void set_state(struct earable_state state);

    void set_sd_state(enum sd_state state);

    void set_charging_state(enum charging_state state);
    void set_pairing_state(enum pairing_state state);
    void set_indication_mode(enum led_mode state);
    void set_custom_color(const RGBColor &color);

private:
    earable_state _state;
    RGBColor color;
};

extern StateIndicator state_indicator;

#endif