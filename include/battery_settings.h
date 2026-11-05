/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OPENEARABLE_BATTERY_SETTINGS_H
#define OPENEARABLE_BATTERY_SETTINGS_H

/** Charger/gauge settings in volts, milliamps, mAh and degrees Celsius. */
struct battery_settings {
    float u_nominal;
    float u_term;
    float u_vlo;
    float u_charge_prevent;
    float i_term;
    float i_charge;
    /** Charger USB input limit, not a battery discharge current limiter. */
    float i_max;
    float capacity;
    float temp_min;
    float temp_fast_min;
    float temp_fast_max;
    float temp_max;
    /** Fuel-gauge EDV overload threshold; does not disconnect the battery. */
    float i_gauge_overload;
};

#endif
