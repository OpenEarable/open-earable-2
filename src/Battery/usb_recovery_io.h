/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdint.h>

/* Internal hardware boundary. The recovery logic is also tested with fault
 * injection, without depending on a working gauge or battery charger. */
struct usb_recovery_snapshot {
	uint16_t voltage_mv;
	uint16_t temperature_dk;
	uint16_t battery_flags;
	uint16_t operation_status;
};

/** Set up the USB power-good input without starting the normal battery workers. */
int usb_recovery_io_init(void);
/** Return 1 for valid USB input, 0 for absent input, or a GPIO error. */
int usb_recovery_io_power_present(void);
/** Drive CD high and verify its pad level. Never enable charging. */
int usb_recovery_io_inhibit_charge(void);
/** Read checked gauge registers, without resetting or programming either IC. */
int usb_recovery_io_read_snapshot(struct usb_recovery_snapshot *snapshot);
