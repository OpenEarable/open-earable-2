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
    preterm_ctrl: int = 0x92,
    ilim_uvlo: int = 0x1C,
    pg_present: bool = True,
    cd_raw: int = 0,
) -> battery.ChargerStatus:
    return battery.ChargerStatus(
        ctrl=ctrl,
        fault=fault,
        ts_fault=ts_fault,
        charge_ctrl=charge_ctrl,
        preterm_ctrl=preterm_ctrl,
        ilim_uvlo=ilim_uvlo,
        pg_present=pg_present,
        cd_raw=cd_raw,
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

    @mock.patch.object(battery.time, "sleep")
    def test_reset_restores_cd_low_when_high_write_fails(
        self, _sleep: mock.Mock
    ) -> None:
        interface = object.__new__(battery.JLinkBatteryInterface)
        interface.set_cd = mock.Mock(
            side_effect=[battery.BatteryDebugError("write failed"), None]
        )

        with self.assertRaisesRegex(battery.BatteryDebugError, "write failed"):
            interface.reset_charger()

        self.assertEqual(interface.set_cd.call_args_list, [mock.call(1), mock.call(0)])

    def test_recovery_configuration_sets_current_precharge_and_uvlo(self) -> None:
        interface = object.__new__(battery.JLinkBatteryInterface)
        interface.set_cd = mock.Mock()
        interface.bq25120a_write_u8 = mock.Mock()

        interface.configure_charger()

        self.assertEqual(
            interface.bq25120a_write_u8.call_args_list,
            [
                mock.call(
                    0x03,
                    battery.encode_charge_current(
                        battery.RECOVERY_CHARGE_CURRENT_MA
                    ),
                ),
                mock.call(
                    0x04,
                    battery.encode_termination_current(
                        battery.RECOVERY_PRETERM_CURRENT_MA
                    ),
                ),
                mock.call(
                    0x09,
                    battery.encode_ilim_uvlo(
                        battery.RECOVERY_INPUT_LIMIT_MA,
                        battery.RECOVERY_UVLO_MV,
                    ),
                ),
            ],
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
            battery.reset_and_configure_charger(link)

        self.assertEqual(link.reset_charger.call_count, 2)
        link.recover_target_state.assert_called_once_with()

    @mock.patch.object(battery.time, "sleep")
    def test_recovery_returns_verified_status(self, _sleep: mock.Mock) -> None:
        link = mock.Mock()
        expected = charger_status(ctrl=0x41)
        link.read_charger.return_value = expected

        actual = battery.reset_and_configure_charger(link)

        self.assertIs(actual, expected)
        link.reset_charger.assert_called_once_with()
        link.configure_charger.assert_called_once_with()

    @mock.patch.object(battery.time, "sleep")
    def test_recovery_retries_when_current_configuration_does_not_stick(
        self, _sleep: mock.Mock
    ) -> None:
        link = mock.Mock()
        link.read_charger.return_value = charger_status(charge_ctrl=0x14)

        with (
            mock.patch.object(sys, "stderr", new=io.StringIO()),
            self.assertRaisesRegex(
                battery.BatteryDebugError, "current configuration did not stick"
            ),
        ):
            battery.reset_and_configure_charger(link)

        self.assertEqual(link.reset_charger.call_count, 2)
        link.recover_target_state.assert_called_once_with()

    @mock.patch.object(battery.time, "sleep")
    def test_recovery_retries_when_uvlo_configuration_does_not_stick(
        self, _sleep: mock.Mock
    ) -> None:
        link = mock.Mock()
        link.read_charger.return_value = charger_status(ilim_uvlo=0x0A)

        with (
            mock.patch.object(sys, "stderr", new=io.StringIO()),
            self.assertRaisesRegex(
                battery.BatteryDebugError, "input-limit/UVLO configuration"
            ),
        ):
            battery.reset_and_configure_charger(link)

        self.assertEqual(link.reset_charger.call_count, 2)
        link.recover_target_state.assert_called_once_with()

    @mock.patch.object(battery.time, "sleep")
    def test_recovery_retries_when_precharge_configuration_does_not_stick(
        self, _sleep: mock.Mock
    ) -> None:
        link = mock.Mock()
        link.read_charger.return_value = charger_status(preterm_ctrl=0x00)

        with (
            mock.patch.object(sys, "stderr", new=io.StringIO()),
            self.assertRaisesRegex(
                battery.BatteryDebugError, "precharge/termination configuration"
            ),
        ):
            battery.reset_and_configure_charger(link)

        self.assertEqual(link.reset_charger.call_count, 2)
        link.recover_target_state.assert_called_once_with()

    @mock.patch.object(battery.time, "sleep")
    def test_blocking_fault_takes_priority_over_retry(self, _sleep: mock.Mock) -> None:
        link = mock.Mock()
        expected = charger_status(ctrl=0xD9, fault=0x10)
        link.read_charger.return_value = expected

        actual = battery.reset_and_configure_charger(link)

        self.assertIs(actual, expected)
        link.reset_charger.assert_called_once_with()
        link.recover_target_state.assert_not_called()


class JLinkLifecycleTests(unittest.TestCase):
    def interface(self) -> battery.JLinkBatteryInterface:
        interface = object.__new__(battery.JLinkBatteryInterface)
        interface.snr = 1
        interface.speed_khz = 1000
        interface.resume = True
        interface.reset_target = False
        interface.jlink = mock.Mock()
        interface.was_halted = False
        interface.probe_open = False
        interface.target_connected = False
        interface.resume_on_exit = False
        interface.setup_gpio = mock.Mock()
        interface.setup_twim = mock.Mock()
        return interface

    @mock.patch.object(battery.time, "sleep")
    def test_enter_failure_resumes_running_target_and_closes_probe(
        self, _sleep: mock.Mock
    ) -> None:
        interface = self.interface()
        interface.jlink.halted.side_effect = [False, False]
        interface.setup_gpio.side_effect = battery.BatteryDebugError("setup failed")

        with self.assertRaisesRegex(battery.BatteryDebugError, "setup failed"):
            interface.__enter__()

        interface.jlink.halt.assert_called_once_with()
        interface.jlink.restart.assert_called_once_with()
        interface.jlink.close.assert_called_once_with()
        self.assertFalse(interface.probe_open)

    def test_enter_failure_preserves_target_that_was_already_halted(self) -> None:
        interface = self.interface()
        interface.jlink.halted.return_value = True
        interface.setup_gpio.side_effect = battery.BatteryDebugError("setup failed")

        with self.assertRaisesRegex(battery.BatteryDebugError, "setup failed"):
            interface.__enter__()

        interface.jlink.halt.assert_not_called()
        interface.jlink.restart.assert_not_called()
        interface.jlink.close.assert_called_once_with()

    def test_open_failure_still_closes_probe_handle(self) -> None:
        interface = self.interface()
        interface.jlink.open.side_effect = battery.BatteryDebugError("open failed")

        with self.assertRaisesRegex(battery.BatteryDebugError, "open failed"):
            interface.__enter__()

        interface.jlink.close.assert_called_once_with()


class TwimTests(unittest.TestCase):
    def interface(self) -> battery.JLinkBatteryInterface:
        interface = object.__new__(battery.JLinkBatteryInterface)
        interface.setup_twim = mock.Mock()
        interface.clear_twim_events = mock.Mock()
        interface.wait_twim = mock.Mock()
        interface.w8 = mock.Mock()
        interface.w32 = mock.Mock()
        interface.r8 = mock.Mock()
        interface.r32 = mock.Mock()
        return interface

    def test_twim_error_is_checked_before_stopped_and_sends_stop(self) -> None:
        interface = self.interface()
        interface.r32.side_effect = [1, battery.TWIM_ERROR_ANACK, 1, 0]
        interface.stop_twim = mock.Mock(return_value=True)

        with self.assertRaisesRegex(battery.BatteryDebugError, "address NACK"):
            interface.wait_twim = battery.JLinkBatteryInterface.wait_twim.__get__(
                interface
            )
            interface.wait_twim()

        self.assertEqual(
            interface.r32.call_args_list[0],
            mock.call(battery.TWIM1 + battery.EVENTS_ERROR),
        )
        interface.stop_twim.assert_called_once_with()

    def test_timeout_reports_bus_line_levels(self) -> None:
        interface = self.interface()
        interface.r32.side_effect = [0, 1, 0]
        interface.raw_gpio0_in = mock.Mock(return_value=1 << battery.SCL_PIN)
        interface.stop_twim = mock.Mock(return_value=True)

        with self.assertRaisesRegex(
            battery.BatteryDebugError, "SCL=high, SDA=low"
        ):
            battery.JLinkBatteryInterface.wait_twim(interface, timeout_s=0)

    def test_register_read_accepts_value_equal_to_old_sentinel(self) -> None:
        interface = self.interface()
        interface.r32.side_effect = [1, 1]
        interface.r8.return_value = [0xCF]

        actual = interface.i2c_read_reg(battery.BQ25120A_ADDR, 0x00, 1)

        self.assertEqual(actual, [0xCF])

    def test_write_stops_after_last_transmitted_byte(self) -> None:
        interface = self.interface()
        interface.r32.return_value = 2

        interface.i2c_write(battery.BQ25120A_ADDR, [0x03, 0x9C])

        self.assertIn(
            mock.call(battery.TWIM1 + battery.TWIM_SHORTS, 1 << 9),
            interface.w32.call_args_list,
        )
        self.assertNotIn(
            mock.call(battery.TWIM1 + battery.TWIM_SHORTS, 1 << 8),
            interface.w32.call_args_list,
        )

    def test_register_read_clears_shorts_after_transaction_error(self) -> None:
        interface = self.interface()
        interface.wait_twim.side_effect = battery.BatteryDebugError("transaction failed")

        with self.assertRaisesRegex(battery.BatteryDebugError, "transaction failed"):
            interface.i2c_read_reg(battery.BQ25120A_ADDR, 0x00, 1)

        self.assertEqual(
            interface.w32.call_args_list[-1],
            mock.call(battery.TWIM1 + battery.TWIM_SHORTS, 0),
        )


class RecoveryCommandTests(unittest.TestCase):
    def recover_args(self) -> object:
        return battery.build_parser().parse_args(
            [
                "recover",
                "--snr",
                "1",
                "--reset-on-fault",
                "--interval-s",
                "0.001",
            ]
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
        reset_charger.assert_called_once_with(link)

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
        reset_charger.assert_called_once_with(link)

    @mock.patch.object(battery, "reset_and_configure_charger")
    @mock.patch.object(battery, "open_link")
    def test_max_timeout_applies_while_fault_reset_is_in_cooldown(
        self, open_link: mock.Mock, reset_charger: mock.Mock
    ) -> None:
        link = mock.MagicMock()
        open_link.return_value = link
        link.__enter__.return_value = link
        link.read_fuel_gauge.side_effect = [fuel_status(2900), fuel_status(2900)]
        link.read_charger.side_effect = [
            charger_status(ctrl=0x41),
            charger_status(ctrl=0xD9),
        ]
        reset_charger.return_value = charger_status(ctrl=0x41)
        args = self.recover_args()
        args.max_minutes = 1e-12

        with (
            mock.patch.object(sys, "stdout", new=io.StringIO()),
            mock.patch.object(sys, "stderr", new=io.StringIO()),
        ):
            result = battery.cmd_recover(args)

        self.assertEqual(result, 1)
        reset_charger.assert_called_once_with(link)


class ArgumentTests(unittest.TestCase):
    def test_invalid_values_are_rejected_without_tracebacks(self) -> None:
        invalid_cases = [
            (["status", "--snr", "not-a-number"], "must be an integer"),
            (["status", "--snr", "-1"], "must be greater than zero"),
            (["status", "--speed-khz", "0"], "must be greater than zero"),
            (["recover", "--interval-s", "0"], "greater than zero"),
            (["recover", "--interval-s", "31"], "watchdog"),
            (["recover", "--max-minutes", "-1"], "zero or greater"),
            (["recover", "--stall-minutes", "nan"], "finite number"),
            (["recover", "--max-fault-resets", "-1"], "zero or greater"),
        ]

        for argv, expected in invalid_cases:
            with self.subTest(argv=argv):
                stderr = io.StringIO()
                with (
                    mock.patch.object(sys, "stderr", new=stderr),
                    self.assertRaises(SystemExit) as raised,
                ):
                    battery.build_parser().parse_args(argv)
                self.assertEqual(raised.exception.code, 2)
                self.assertIn(expected, stderr.getvalue())
                self.assertNotIn("Traceback", stderr.getvalue())

    def test_keyboard_interrupt_is_reported_cleanly(self) -> None:
        stderr = io.StringIO()
        with (
            mock.patch.object(
                battery, "cmd_voltage", side_effect=KeyboardInterrupt
            ),
            mock.patch.object(sys, "stderr", new=stderr),
        ):
            result = battery.main(["voltage"])

        self.assertEqual(result, 130)
        self.assertIn("battery_debug: interrupted.", stderr.getvalue())
        self.assertNotIn("Traceback", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
