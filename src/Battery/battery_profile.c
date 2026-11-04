/* SPDX-License-Identifier: Apache-2.0 */
#include "battery_profile.h"
#include <math.h>

/* 100 mA is representable by the BQ25120A and below the nominal 105 mA
 * VARTA fast-charge / 111 mA Grepow maximum settings. Retain the existing
 * 10 mA termination threshold; no Grepow-specific taper calibration is claimed.
 */
static const struct battery_profile varta = {
    .name = "VARTA CP1454 A4X",
    .settings = {
        .u_nominal = 3.7f, .u_term = 4.3f, .u_vlo = 3.0f,
        .u_charge_prevent = 2.5f,
        .i_term = 10.0f, .i_charge = 100.0f, .i_max = 200.0f,
        .capacity = 108.0f,
        .temp_min = 0.0f, .temp_fast_min = 15.0f,
        .temp_fast_max = 45.0f, .temp_max = 45.0f,
        .i_gauge_overload = 200.0f,
    },
};

/* Grepow pack design guideline, 2026-07-16: 111 mAh typical, 108 mAh
 * minimum; 4.20 V; 111 mA max charge/discharge; 3.0 V discharge cutoff;
 * charge at 0..45 C. Undervoltage recovery below 3.0 V is not specified,
 * so do not automatically recover a cell outside that operating window.
 */
static const struct battery_profile grepow = {
    .name = "Grepow GRP1454M1",
    .settings = {
        .u_nominal = 3.7f, .u_term = 4.2f, .u_vlo = 3.0f,
        .u_charge_prevent = 3.0f,
        .i_term = 10.0f, .i_charge = 100.0f, .i_max = 200.0f,
        .capacity = 111.0f,
        .temp_min = 0.0f, .temp_fast_min = 15.0f,
        .temp_fast_max = 40.0f, .temp_max = 45.0f,
        .i_gauge_overload = 111.0f,
    },
};

static const struct battery_profile unidentified = {
    .name = "Unidentified (4.2 V fallback)",
    .settings = {
        .u_nominal = 3.7f, .u_term = 4.2f, .u_vlo = 3.0f,
        .u_charge_prevent = 3.0f,
        .i_term = 10.0f, .i_charge = 100.0f, .i_max = 200.0f,
        .capacity = 108.0f,
        .temp_min = 0.0f, .temp_fast_min = 15.0f,
        .temp_fast_max = 40.0f, .temp_max = 45.0f,
        .i_gauge_overload = 111.0f,
    },
};

const struct battery_profile *battery_profile_for_imu(uint8_t address)
{
    if (address == 0x68) return &varta;
    if (address == 0x69) return &grepow;
    return &unidentified;
}

float battery_profile_charge_current(const struct battery_settings *settings,
                                    float voltage, float temperature)
{
    if (!isfinite(voltage) || !isfinite(temperature) ||
        voltage < settings->u_charge_prevent ||
        temperature < settings->temp_min || temperature > settings->temp_max) {
        return 0.0f;
    }
    if (temperature < settings->temp_fast_min || temperature > settings->temp_fast_max) {
        return settings->i_charge / 2.0f;
    }
    return settings->i_charge;
}
