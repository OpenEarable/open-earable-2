/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <unity.h>
#include "usb_recovery.h"
#include "usb_recovery_io.h"

static struct usb_recovery_snapshot battery;
static int init_error, inhibit_error, read_error, power;
static int inhibit_calls, read_calls, power_after_read;

void setUp(void)
{
	battery = (struct usb_recovery_snapshot){
		.voltage_mv = 3700, .temperature_dk = 2982,
		.battery_flags = 1U << 3, .operation_status = 1U << 5,
	};
	init_error = inhibit_error = read_error = inhibit_calls = read_calls = 0;
	power = power_after_read = 1;
}

void tearDown(void) {}

int usb_recovery_io_init(void) { return init_error; }
int usb_recovery_io_power_present(void) { return power; }
int usb_recovery_io_inhibit_charge(void)
{
	++inhibit_calls;
	return inhibit_error;
}
int usb_recovery_io_read_snapshot(struct usb_recovery_snapshot *snapshot)
{
	/* No battery access is allowed before charging has been inhibited. */
	TEST_ASSERT_EQUAL(1, inhibit_calls);
	++read_calls;
	*snapshot = battery;
	power = power_after_read;
	return read_error;
}

void test_battery_only_boot_does_not_touch_charger_or_gauge(void)
{
	power = 0;
	TEST_ASSERT_EQUAL(0, usb_recovery_prepare());
	TEST_ASSERT_FALSE(usb_recovery_active());
	TEST_ASSERT_EQUAL(0, inhibit_calls);
	TEST_ASSERT_EQUAL(0, read_calls);
}

void test_healthy_usb_boot_hands_control_to_normal_power_manager(void)
{
	TEST_ASSERT_EQUAL(0, usb_recovery_prepare());
	TEST_ASSERT_FALSE(usb_recovery_active());
	TEST_ASSERT_EQUAL(1, inhibit_calls);
	TEST_ASSERT_EQUAL(1, read_calls);
	TEST_ASSERT_EQUAL(-EACCES, usb_recovery_poll());
}

void test_safe_depleted_cell_can_still_precharge_normally(void)
{
	battery.voltage_mv = 2500;
	battery.battery_flags |= 1U << 1; /* SYSDWN is not a charging prohibition. */
	TEST_ASSERT_EQUAL(0, usb_recovery_prepare());
}

void test_unsafe_voltage_and_temperature_use_recovery(void)
{
	const unsigned voltages[] = {0, 2499, 4501, 65535};
	for (unsigned i = 0; i < sizeof(voltages) / sizeof(voltages[0]); ++i) {
		setUp();
		battery.voltage_mv = voltages[i];
		TEST_ASSERT_EQUAL(1, usb_recovery_prepare());
	}
	const unsigned temperatures[] = {0, 2731, 3182, 65535};
	for (unsigned i = 0; i < sizeof(temperatures) / sizeof(temperatures[0]); ++i) {
		setUp();
		battery.temperature_dk = temperatures[i];
		TEST_ASSERT_EQUAL(1, usb_recovery_prepare());
	}
}

void test_temperature_boundaries_allow_normal_boot(void)
{
	battery.temperature_dk = 2732;
	TEST_ASSERT_EQUAL(0, usb_recovery_prepare());
	setUp();
	battery.temperature_dk = 3181;
	TEST_ASSERT_EQUAL(0, usb_recovery_prepare());
}

void test_missing_inhibited_or_hot_battery_uses_recovery(void)
{
	battery.battery_flags = 0;
	TEST_ASSERT_EQUAL(1, usb_recovery_prepare());
	const unsigned faults[] = {8, 10, 11};
	for (unsigned i = 0; i < sizeof(faults) / sizeof(faults[0]); ++i) {
		setUp();
		battery.battery_flags |= 1U << faults[i];
		TEST_ASSERT_EQUAL(1, usb_recovery_prepare());
	}
}

void test_uninitialized_or_configuring_gauge_uses_recovery(void)
{
	battery.operation_status = 0;
	TEST_ASSERT_EQUAL(1, usb_recovery_prepare());
	setUp();
	battery.operation_status |= 1U << 10;
	TEST_ASSERT_EQUAL(1, usb_recovery_prepare());
}

void test_i2c_failure_keeps_charging_inhibited_and_enters_recovery(void)
{
	read_error = -ETIMEDOUT;
	TEST_ASSERT_EQUAL(1, usb_recovery_prepare());
	TEST_ASSERT_TRUE(usb_recovery_active());
	TEST_ASSERT_EQUAL(1, inhibit_calls);
}

void test_gpio_failure_never_permits_normal_boot(void)
{
	init_error = -ENODEV;
	TEST_ASSERT_EQUAL(-ENODEV, usb_recovery_prepare());
	TEST_ASSERT_EQUAL(0, read_calls);
	setUp();
	power = -EIO;
	TEST_ASSERT_EQUAL(-EIO, usb_recovery_prepare());
	TEST_ASSERT_EQUAL(0, read_calls);
	setUp();
	inhibit_error = -EIO;
	TEST_ASSERT_EQUAL(-EIO, usb_recovery_prepare());
	TEST_ASSERT_EQUAL(0, read_calls);
}

void test_usb_removed_during_probe_does_not_enter_normal_boot(void)
{
	power_after_read = 0;
	TEST_ASSERT_EQUAL(1, usb_recovery_prepare());
	TEST_ASSERT_EQUAL(0, usb_recovery_poll());
}

void test_recovery_stays_latched_even_if_gauge_readings_improve(void)
{
	battery.voltage_mv = 1000;
	TEST_ASSERT_EQUAL(1, usb_recovery_prepare());
	battery.voltage_mv = 3700;
	for (int i = 0; i < 100; ++i) {
		TEST_ASSERT_EQUAL(1, usb_recovery_poll());
	}
	TEST_ASSERT_TRUE(usb_recovery_active());
	TEST_ASSERT_EQUAL(1, read_calls);
	TEST_ASSERT_EQUAL(101, inhibit_calls);
	power = 0;
	TEST_ASSERT_EQUAL(0, usb_recovery_poll());
	TEST_ASSERT_EQUAL(102, inhibit_calls);
}

void test_recovery_reports_failed_charge_inhibition(void)
{
	read_error = -EIO;
	TEST_ASSERT_EQUAL(1, usb_recovery_prepare());
	inhibit_error = -EIO;
	TEST_ASSERT_EQUAL(-EIO, usb_recovery_poll());
}

extern int unity_main(void);
int main(void) { return unity_main(); }
