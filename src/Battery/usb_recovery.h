/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Call once at boot to check USB power and battery safety before the power manager.
 * Returns 0 for normal boot, 1 for latched USB recovery, or a negative GPIO
 * error. A negative result must never fall through to normal boot.
 * The recovery decision is retained until reboot, even if gauge values improve.
 */
int usb_recovery_prepare(void);

/** Return true when the normal battery, audio and sensor services are bypassed. */
bool usb_recovery_active(void);

/** Reassert charging inhibition; return 1 with USB present, 0 if removed, or error. */
int usb_recovery_poll(void);

/** Serve USB mcumgr without charging. Does not return; powers off on USB removal. */
void usb_recovery_run(void);

#ifdef __cplusplus
}
#endif
