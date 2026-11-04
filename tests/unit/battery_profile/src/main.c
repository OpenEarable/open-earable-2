/* SPDX-License-Identifier: Apache-2.0 */
#include <math.h>
#include <unity.h>
#include "battery_profile.h"

void setUp(void) {}
void tearDown(void) {}

void test_legacy_imu_selects_varta_voltage_and_nominal_capacity(void)
{
    const struct battery_settings *s = &battery_profile_for_imu(0x68)->settings;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.3f, s->u_term);
    TEST_ASSERT_EQUAL_FLOAT(108.0f, s->capacity);
}

void test_external_imu_selects_grepow_specification(void)
{
    const struct battery_settings *s = &battery_profile_for_imu(0x69)->settings;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.2f, s->u_term);
    TEST_ASSERT_EQUAL_FLOAT(111.0f, s->capacity);
    TEST_ASSERT_TRUE(s->i_charge <= 111.0f);
    TEST_ASSERT_EQUAL_FLOAT(45.0f, s->temp_max);
    TEST_ASSERT_EQUAL_FLOAT(111.0f, s->i_gauge_overload);
}

void test_failed_and_unexpected_probes_cannot_select_4_3_volts(void)
{
    for (unsigned int address = 0; address <= 255; address++) {
        if (address == 0x68) continue;
        TEST_ASSERT_TRUE(battery_profile_for_imu(address)->settings.u_term <= 4.2f);
    }
    TEST_ASSERT_EQUAL_FLOAT(108.0f, battery_profile_for_imu(0)->settings.capacity);
}

void test_charge_current_matches_each_cell_and_fallback(void)
{
    const uint8_t addresses[] = {0, 0x68, 0x69};
    const float expected[] = {80.0f, 100.0f, 90.0f};
    for (unsigned int i = 0; i < sizeof(addresses); i++) {
        const struct battery_settings *s = &battery_profile_for_imu(addresses[i])->settings;
        TEST_ASSERT_EQUAL_FLOAT(expected[i], battery_profile_charge_current(s, 3.7f, 25.0f));
    }
}

void test_charging_stops_outside_zero_to_45_degrees(void)
{
    const uint8_t addresses[] = {0, 0x68, 0x69};
    for (unsigned int i = 0; i < sizeof(addresses); i++) {
        const struct battery_settings *s = &battery_profile_for_imu(addresses[i])->settings;
        TEST_ASSERT_EQUAL_FLOAT(0.0f, battery_profile_charge_current(s, 3.7f, -0.1f));
        TEST_ASSERT_EQUAL_FLOAT(0.0f, battery_profile_charge_current(s, 3.7f, 45.1f));
        TEST_ASSERT_TRUE(battery_profile_charge_current(s, 3.7f, 0.0f) > 0);
        TEST_ASSERT_TRUE(battery_profile_charge_current(s, 3.7f, 45.0f) > 0);
    }
}

void test_grepow_and_unknown_cells_do_not_auto_recover_below_3_volts(void)
{
    const uint8_t addresses[] = {0, 0x69};
    for (unsigned int i = 0; i < sizeof(addresses); i++) {
        const struct battery_settings *s = &battery_profile_for_imu(addresses[i])->settings;
        TEST_ASSERT_EQUAL_FLOAT(0.0f, battery_profile_charge_current(s, 2.99f, 25.0f));
        TEST_ASSERT_TRUE(battery_profile_charge_current(s, 3.0f, 25.0f) > 0);
    }
}

void test_legacy_low_voltage_recovery_boundary_is_preserved(void)
{
    const struct battery_settings *s = &battery_profile_for_imu(0x68)->settings;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, battery_profile_charge_current(s, 2.49f, 25.0f));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, battery_profile_charge_current(s, 2.5f, 25.0f));
}

void test_temperature_derating_is_reapplied_after_recovery(void)
{
    const struct battery_settings *s = &battery_profile_for_imu(0x69)->settings;
    TEST_ASSERT_EQUAL_FLOAT(40.0f, battery_profile_charge_current(s, 3.7f, 10.0f));
    TEST_ASSERT_EQUAL_FLOAT(40.0f, battery_profile_charge_current(s, 3.7f, 42.0f));
    TEST_ASSERT_EQUAL_FLOAT(90.0f, battery_profile_charge_current(s, 3.7f, 25.0f));
}

void test_invalid_measurements_prohibit_charging(void)
{
    const struct battery_settings *s = &battery_profile_for_imu(0)->settings;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, battery_profile_charge_current(s, NAN, 25.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, battery_profile_charge_current(s, 3.7f, NAN));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, battery_profile_charge_current(s, INFINITY, 25.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, battery_profile_charge_current(s, 3.7f, INFINITY));
}

extern int unity_main(void);
int main(void) { return unity_main(); }
