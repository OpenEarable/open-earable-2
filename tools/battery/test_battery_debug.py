#!/usr/bin/env python3

import io
import sys
import types
import unittest
from pathlib import Path
from unittest import mock


try:
    import pylink  # noqa: F401
except ImportError:
    pylink = types.ModuleType("pylink")
    pylink.JLink = object
    pylink.errors = types.SimpleNamespace(JLinkException=Exception)
    pylink.enums = types.ModuleType("pylink.enums")
    pylink.enums.JLinkInterfaces = types.SimpleNamespace(SWD=1)
    sys.modules["pylink"] = pylink
    sys.modules["pylink.enums"] = pylink.enums

sys.path.insert(0, str(Path(__file__).resolve().parent))
import battery_debug as battery


def charger_status(
    *,
    ctrl: int = 0x41,
    fault: int = 0x00,
    ts_fault: int = 0x88,
    charge_ctrl: int = 0x9C,
    pg_present: bool = True,
    cd_raw: int = 0,
) -> battery.ChargerStatus:
    return battery.ChargerStatus(
        ctrl=ctrl,
        fault=fault,
        ts_fault=ts_fault,
        charge_ctrl=charge_ctrl,
        ilim_uvlo=0x0A,
        pg_present=pg_present,
        cd_raw=cd_raw,
    )


def recovery_args() -> types.SimpleNamespace:
    return types.SimpleNamespace(
        charge_current_ma=110,
        termination_current_ma=10.0,
        target_voltage_mv=4300,
        input_limit_ma=200,
        uvlo_mv=2500,
        full_charger_config=False,
    )


def fuel_status(voltage_mv: int) -> battery.FuelGaugeStatus:
    return battery.FuelGaugeStatus(
        voltage_mv=voltage_mv,
        temperature_c=25.0,
        state_of_charge_pct=5,
        average_current_ma=10,
        flags=0,
        gauging_status=None,
    )


class ChargerStatusTests(unittest.TestCase):
    def test_reset_and_timer_bits_are_distinct(self) -> None:
        charging = charger_status(ctrl=0x51)
        timer_fault = charger_status(ctrl=0xD9)

        self.assertTrue(charging.reset_fault)
        self.assertFalse(charging.timer_fault)
        self.assertEqual(charging.fault_reasons, [])
        self.assertTrue(timer_fault.reset_fault)
        self.assertTrue(timer_fault.timer_fault)
        self.assertEqual(timer_fault.fault_reasons, ["safety_timer"])

    def test_all_fault_register_bits_are_named(self) -> None:
        status = charger_status(ctrl=0xC1, fault=0xF0)

        self.assertEqual(
            status.fault_reasons,
            ["VIN_OV", "VIN_UV", "BAT_UVLO", "BAT_OCP"],
        )
        self.assertEqual(status.blocking_fault_reasons, ["VIN_OV", "BAT_OCP"])

    def test_temperature_states_are_named_and_classified(self) -> None:
        suspended = charger_status(ctrl=0xC1, ts_fault=0xA8)
        cool = charger_status(ctrl=0xC1, ts_fault=0xC8)
        warm = charger_status(ctrl=0xC1, ts_fault=0xE8)

        self.assertEqual(suspended.ts_state, "TS_hot_or_cold")
        self.assertEqual(suspended.blocking_fault_reasons, ["TS_hot_or_cold"])
        self.assertEqual(cool.ts_state, "TS_cool_current_reduced")
        self.assertEqual(cool.blocking_fault_reasons, [])
        self.assertEqual(warm.ts_state, "TS_warm_voltage_reduced")
        self.assertEqual(warm.blocking_fault_reasons, [])

    def test_only_resettable_conditions_request_a_reset(self) -> None:
        timer = charger_status(ctrl=0xC9)
        unknown = charger_status(ctrl=0xC1)
        vin_undervoltage = charger_status(ctrl=0xC1, fault=0x40)
        bat_uvlo = charger_status(ctrl=0xC1, fault=0x20)
        vindpm = charger_status(ctrl=0xC5)

        self.assertEqual(
            battery.charger_recovery_reason(timer, False), "safety timer fault"
        )
        self.assertIsNone(battery.charger_recovery_reason(unknown, False))
        self.assertEqual(
            battery.charger_recovery_reason(unknown, True),
            "unclassified charger status fault",
        )
        self.assertIsNone(battery.charger_recovery_reason(vin_undervoltage, True))
        self.assertIsNone(battery.charger_recovery_reason(bat_uvlo, True))
        self.assertIsNone(battery.charger_recovery_reason(vindpm, True))

    def test_missing_input_power_is_blocking(self) -> None:
        status = charger_status(pg_present=False)

        self.assertEqual(
            battery.charger_blocking_reasons(status), ["input_power_missing"]
        )


class ChargerControlTests(unittest.TestCase):
    def test_cd_gpio_enables_input_for_level_verification(self) -> None:
        interface = object.__new__(battery.JLinkBatteryInterface)
        interface.w32 = mock.Mock()

        interface.setup_gpio()

        self.assertIn(
            mock.call(
                battery.pin_cnf(battery.CD_PIN),
                battery.GPIO_PIN_CNF_OUTPUT_WITH_INPUT,
            ),
            interface.w32.call_args_list,
        )

    @mock.patch.object(battery.time, "sleep")
    def test_cd_uses_gpio_port_offsets(self, _sleep: mock.Mock) -> None:
        interface = object.__new__(battery.JLinkBatteryInterface)
        interface.w32 = mock.Mock()

        interface.set_cd(1)
        interface.set_cd(0)

        self.assertEqual(
            interface.w32.call_args_list,
            [
                mock.call(battery.GPIO0 + 0x08, 1 << battery.CD_PIN),
                mock.call(battery.GPIO0 + 0x0C, 1 << battery.CD_PIN),
            ],
        )

    @mock.patch.object(battery.time, "sleep")
    def test_reset_pulses_and_verifies_cd(self, _sleep: mock.Mock) -> None:
        interface = object.__new__(battery.JLinkBatteryInterface)
        interface.set_cd = mock.Mock()
        interface.raw_gpio0_in = mock.Mock(side_effect=[1 << battery.CD_PIN, 0])
        interface.bq25120a_write_u8 = mock.Mock()

        interface.reset_charger()

        self.assertEqual(
            interface.set_cd.call_args_list,
            [mock.call(1), mock.call(0), mock.call(0)],
        )
        interface.bq25120a_write_u8.assert_called_once_with(0x09, 0x80)

    def test_minimal_configuration_preserves_temperature_monitoring(self) -> None:
        interface = object.__new__(battery.JLinkBatteryInterface)
        interface.set_cd = mock.Mock()
        interface.bq25120a_write_u8 = mock.Mock()

        interface.configure_charger(
            charge_current_ma=110,
            termination_current_ma=10.0,
            target_voltage_mv=4300,
            input_limit_ma=200,
            uvlo_mv=2500,
        )

        interface.bq25120a_write_u8.assert_called_once_with(
            0x03, battery.encode_charge_current(110)
        )

    @mock.patch.object(battery.time, "sleep")
    def test_resume_target_is_verified(self, _sleep: mock.Mock) -> None:
        interface = object.__new__(battery.JLinkBatteryInterface)
        interface.jlink = mock.Mock()
        interface.jlink.halted.return_value = False

        interface.resume_target()

        interface.jlink.restart.assert_called_once_with()
        interface.jlink.halted.assert_called_once_with()

    @mock.patch.object(battery.time, "sleep")
    def test_recovery_verifies_timer_cleared(self, _sleep: mock.Mock) -> None:
        link = mock.Mock()
        link.read_charger.return_value = charger_status(ctrl=0xD9)

        with (
            mock.patch.object(sys, "stderr", new=io.StringIO()),
            self.assertRaisesRegex(
                battery.BatteryDebugError, "failed after two attempts"
            ),
        ):
            battery.reset_and_configure_charger(link, recovery_args())

        self.assertEqual(link.reset_charger.call_count, 2)
        link.recover_target_state.assert_called_once_with()

    @mock.patch.object(battery.time, "sleep")
    def test_recovery_returns_verified_status(self, _sleep: mock.Mock) -> None:
        link = mock.Mock()
        expected = charger_status(ctrl=0x41)
        link.read_charger.return_value = expected

        actual = battery.reset_and_configure_charger(link, recovery_args())

        self.assertIs(actual, expected)
        link.reset_charger.assert_called_once_with()
        link.configure_charger.assert_called_once_with(
            charge_current_ma=110,
            termination_current_ma=10.0,
            target_voltage_mv=4300,
            input_limit_ma=200,
            uvlo_mv=2500,
            full_config=False,
        )

    @mock.patch.object(battery.time, "sleep")
    def test_blocking_fault_takes_priority_over_retry(self, _sleep: mock.Mock) -> None:
        link = mock.Mock()
        expected = charger_status(ctrl=0xD9, fault=0x10)
        link.read_charger.return_value = expected

        actual = battery.reset_and_configure_charger(link, recovery_args())

        self.assertIs(actual, expected)
        link.reset_charger.assert_called_once_with()
        link.recover_target_state.assert_not_called()


class RecoveryCommandTests(unittest.TestCase):
    def recover_args(self) -> object:
        return battery.build_parser().parse_args(
            ["recover", "--snr", "1", "--reset-on-fault", "--interval-s", "0"]
        )

    @mock.patch.object(battery, "reset_and_configure_charger")
    @mock.patch.object(battery, "open_link")
    def test_timer_is_cleared_before_target_success(
        self, open_link: mock.Mock, reset_charger: mock.Mock
    ) -> None:
        link = mock.MagicMock()
        open_link.return_value = link
        link.__enter__.return_value = link
        link.read_fuel_gauge.side_effect = [fuel_status(3400), fuel_status(3400)]
        link.read_charger.side_effect = [
            charger_status(ctrl=0xD9),
            charger_status(ctrl=0x41),
        ]
        reset_charger.return_value = charger_status(ctrl=0x41)

        with mock.patch.object(sys, "stdout", new=io.StringIO()):
            result = battery.cmd_recover(self.recover_args())

        self.assertEqual(result, 0)
        reset_charger.assert_called_once_with(link, mock.ANY)

    @mock.patch.object(battery, "reset_and_configure_charger")
    @mock.patch.object(battery, "open_link")
    def test_physical_fault_stops_without_reset(
        self, open_link: mock.Mock, reset_charger: mock.Mock
    ) -> None:
        link = mock.MagicMock()
        open_link.return_value = link
        link.__enter__.return_value = link
        link.read_fuel_gauge.return_value = fuel_status(3400)
        link.read_charger.return_value = charger_status(ctrl=0xC1, fault=0x80)

        with (
            mock.patch.object(sys, "stdout", new=io.StringIO()),
            mock.patch.object(sys, "stderr", new=io.StringIO()),
        ):
            result = battery.cmd_recover(self.recover_args())

        self.assertEqual(result, 1)
        reset_charger.assert_not_called()

    @mock.patch.object(battery, "reset_and_configure_charger")
    @mock.patch.object(battery, "open_link")
    def test_physical_fault_seen_during_reset_is_not_lost(
        self, open_link: mock.Mock, reset_charger: mock.Mock
    ) -> None:
        link = mock.MagicMock()
        open_link.return_value = link
        link.__enter__.return_value = link
        link.read_fuel_gauge.return_value = fuel_status(3400)
        link.read_charger.return_value = charger_status(ctrl=0xD9)
        reset_charger.return_value = charger_status(ctrl=0xC1, fault=0x10)

        with (
            mock.patch.object(sys, "stdout", new=io.StringIO()),
            mock.patch.object(sys, "stderr", new=io.StringIO()),
        ):
            result = battery.cmd_recover(self.recover_args())

        self.assertEqual(result, 1)
        reset_charger.assert_called_once_with(link, mock.ANY)


if __name__ == "__main__":
    unittest.main()
