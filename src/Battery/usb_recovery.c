/* SPDX-License-Identifier: Apache-2.0 */
#include "usb_recovery.h"
#include "usb_recovery_io.h"

#include <errno.h>

static bool active;

static bool needs_recovery(const struct usb_recovery_snapshot *s)
{
	/* Match the existing cell profile's 2.5 V charge-inhibit threshold and
	 * 0..45 C charging range. 4.5 V is beyond its 4.3 V charge target. A low
	 * state of charge/SYSDWN alone must still permit normal precharging. */
	return s->voltage_mv < 2500 || s->voltage_mv > 4500 ||
		s->temperature_dk < 2732 || s->temperature_dk > 3181 ||
		!(s->battery_flags & (1U << 3)) || /* BATTPRES */
		(s->battery_flags & ((1U << 8) | (1U << 10) | (1U << 11))) ||
		!(s->operation_status & (1U << 5)) || /* INITCOMP */
		(s->operation_status & (1U << 10)); /* CFG_UPDATE: stale readings */
}

int usb_recovery_prepare(void)
{
	/* Called once at boot, before any normal battery workers are started. */
	active = false;
	int ret = usb_recovery_io_init();
	if (ret) {
		return ret;
	}
	ret = usb_recovery_io_power_present();
	if (ret <= 0) {
		return ret;
	}
	/* GPIO inhibition is independent of gauge/I2C health and survives PMIC
	 * watchdog register resets. Normal boot takes ownership only after this
	 * checked snapshot; recovery never runs the normal charge-enable workers. */
	ret = usb_recovery_io_inhibit_charge();
	if (ret) {
		return ret;
	}
	struct usb_recovery_snapshot snapshot = {0};
	ret = usb_recovery_io_read_snapshot(&snapshot);
	active = ret != 0 || needs_recovery(&snapshot);
	/* USB disappearing during the reads must not admit a depleted-battery boot. */
	if (usb_recovery_io_power_present() != 1) {
		active = true;
	}
	return active ? 1 : 0;
}

bool usb_recovery_active(void)
{
	return active;
}

int usb_recovery_poll(void)
{
	if (!active) {
		return -EACCES;
	}
	int ret = usb_recovery_io_inhibit_charge();
	if (ret) {
		return ret;
	}
	return usb_recovery_io_power_present();
}
