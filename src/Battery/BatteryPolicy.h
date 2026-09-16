#pragma once

#include <cmath>
#include "openearable_common.h"

/** Battery admission rules shared by boot, button wake, and periodic checks.
 * Voltage is in V, temperature in Celsius. Failed measurements must be rejected
 * by the caller before applying these rules. Hardware UVLO is a last resort.
 */
namespace battery_policy {
inline float shutdown_voltage(const battery_settings &settings) {
    return settings.u_vlo + CONFIG_BATTERY_SYSDOWN_SET_OFFSET / 1000.0f;
}

inline float start_voltage(const battery_settings &settings) {
    return shutdown_voltage(settings) + CONFIG_BATTERY_SYSDOWN_HYSTERESIS / 1000.0f;
}

inline bool can_start(float voltage, float temperature, bool system_down,
                      const battery_settings &settings) {
    return std::isfinite(voltage) && std::isfinite(temperature) && !system_down &&
           voltage >= start_voltage(settings) && voltage <= settings.u_term + 0.1f &&
           temperature >= settings.temp_min && temperature <= settings.temp_max;
}

inline bool can_charge(float voltage, float temperature, bool inhibited,
                       const battery_settings &settings) {
    return std::isfinite(voltage) && std::isfinite(temperature) && !inhibited &&
           voltage >= settings.u_charge_prevent && voltage <= settings.u_term + 0.1f &&
           temperature >= settings.temp_min && temperature <= settings.temp_max;
}
}
