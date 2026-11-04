/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OPENEARABLE_BATTERY_PROFILE_H
#define OPENEARABLE_BATTERY_PROFILE_H

#include <stdint.h>
#include "battery_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

struct battery_profile {
    const char *name;
    struct battery_settings settings;
};

/** Select by the validated IMU probe result. Only 0x68 permits 4.3 V.
 * Pass zero on a failed/ambiguous probe; unknown addresses use the 4.2 V
 * fallback regardless of the hardware revision stored in UICR.
 * Returned profiles have static lifetime.
 */
const struct battery_profile *battery_profile_for_imu(uint8_t address);

/** Requested charge current, or zero if voltage/temperature prohibits charging.
 * Non-finite readings fail closed. This does not decide whether the MCU can
 * boot: a low battery may still charge while the MCU stays in charging mode.
 */
float battery_profile_charge_current(const struct battery_settings *settings,
                                    float voltage, float temperature);

#ifdef __cplusplus
}
#endif
#endif
