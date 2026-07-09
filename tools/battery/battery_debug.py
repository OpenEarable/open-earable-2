#!/usr/bin/env python3
"""OpenEarable v2 battery debugging over J-Link/SWD.

The tool talks directly to the battery ICs on I2C1 by briefly halting the
nRF5340 application core and using its TWIM1 peripheral. It does not require a
special firmware image on the target.
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
import sys
import time
from typing import Iterable

try:
    import pylink
    from pylink.enums import JLinkInterfaces
except ImportError as exc:  # pragma: no cover - depends on local workstation
    raise SystemExit(
        "Missing Python package 'pylink'. Install pylink-square or use the "
        "Nordic toolchain Python that already provides it."
    ) from exc


APP_CORE_DEVICE = "NRF5340_XXAA_APP"
DEFAULT_SPEED_KHZ = 1000

TWIM1 = 0x50009000
GPIO0 = 0x50842500
SCRATCH_TX = 0x20070000
SCRATCH_RX = 0x20070080

SDA_PIN = 21
SCL_PIN = 24
PG_PIN = 18
CD_PIN = 17

BQ27220_ADDR = 0x55
BQ25120A_ADDR = 0x6A

FREQUENCY_100K = 0x01980000
FREQUENCY_250K = 0x04000000
FREQUENCY_400K = 0x06400000

GPIO_PIN_CNF_INPUT_PULLUP_S0D1 = 0x0000060C
GPIO_PIN_CNF_INPUT_PULLUP = 0x0000000C
GPIO_PIN_CNF_OUTPUT = 0x00000003

TWIM_ENABLE = 0x500
TWIM_PSEL_SCL = 0x508
TWIM_PSEL_SDA = 0x50C
TWIM_FREQUENCY = 0x524
TWIM_RXD_PTR = 0x534
TWIM_RXD_MAXCNT = 0x538
TWIM_RXD_AMOUNT = 0x53C
TWIM_TXD_PTR = 0x544
TWIM_TXD_MAXCNT = 0x548
TWIM_TXD_AMOUNT = 0x54C
TWIM_ADDRESS = 0x588
TWIM_SHORTS = 0x200
TWIM_ERRORSRC = 0x4C4

TASKS_STARTRX = 0x000
TASKS_STARTTX = 0x008
TASKS_STOP = 0x014
EVENTS_STOPPED = 0x104
EVENTS_ERROR = 0x124
EVENTS_LASTRX = 0x15C
EVENTS_LASTTX = 0x160

SHORT_LASTTX_STARTRX = 1 << 7
SHORT_LASTTX_STOP = 1 << 8
SHORT_LASTRX_STOP = 1 << 12


class BatteryDebugError(RuntimeError):
    pass


def u16le(data: Iterable[int]) -> int:
    raw = bytes(data)
    return int.from_bytes(raw[:2], "little", signed=False)


def i16le(data: Iterable[int]) -> int:
    raw = bytes(data)
    return int.from_bytes(raw[:2], "little", signed=True)


def pin_cnf(pin: int) -> int:
    return GPIO0 + 0x200 + pin * 4


@dataclasses.dataclass
class FuelGaugeStatus:
    voltage_mv: int
    temperature_c: float | None
    state_of_charge_pct: int | None
    average_current_ma: int | None
    flags: int | None
    gauging_status: int | None


@dataclasses.dataclass
class ChargerStatus:
    ctrl: int
    fault: int
    ts_fault: int
    charge_ctrl: int
    ilim_uvlo: int
    pg_present: bool
    cd_raw: int

    @property
    def charging_state_code(self) -> int:
        return self.ctrl >> 6

    @property
    def charging_state(self) -> str:
        return {
            0: "ready/discharge",
            1: "charging",
            2: "done",
            3: "fault",
        }.get(self.charging_state_code, f"unknown-{self.charging_state_code}")

    @property
    def timer_fault(self) -> bool:
        return bool(self.ctrl & (1 << 4))

    @property
    def bat_uvlo(self) -> bool:
        return bool(self.fault & (1 << 5))

    @property
    def charge_enabled(self) -> bool:
        return not bool(self.charge_ctrl & 0x02)

    @property
    def high_z(self) -> bool:
        return bool(self.charge_ctrl & 0x01)


class JLinkBatteryInterface:
    def __init__(self, snr: str | None, speed_khz: int, resume: bool = True):
        self.snr = int(snr) if snr else None
        self.speed_khz = speed_khz
        self.resume = resume
        self.jlink = pylink.JLink()
        self.was_halted = False

    def __enter__(self) -> "JLinkBatteryInterface":
        self.jlink.open(serial_no=self.snr)
        self.jlink.set_tif(JLinkInterfaces.SWD)
        self.jlink.connect(APP_CORE_DEVICE, speed=self.speed_khz, verbose=False)
        self.was_halted = bool(self.jlink.halted())
        if not self.was_halted:
            self.jlink.halt()
        self.setup_gpio()
        self.setup_twim()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        with contextlib.suppress(Exception):
            self.w32(TWIM1 + TWIM_ENABLE, 0)
        if self.resume and not self.was_halted:
            with contextlib.suppress(Exception):
                self.jlink.restart()
        with contextlib.suppress(Exception):
            self.jlink.close()

    def r32(self, addr: int) -> int:
        return self.jlink.memory_read32(addr, 1)[0]

    def w32(self, addr: int, value: int) -> None:
        self.jlink.memory_write32(addr, [value & 0xFFFFFFFF])

    def r8(self, addr: int, count: int) -> list[int]:
        return list(self.jlink.memory_read8(addr, count))

    def w8(self, addr: int, data: Iterable[int]) -> None:
        self.jlink.memory_write8(addr, list(data))

    def setup_gpio(self) -> None:
        self.w32(pin_cnf(SDA_PIN), GPIO_PIN_CNF_INPUT_PULLUP_S0D1)
        self.w32(pin_cnf(SCL_PIN), GPIO_PIN_CNF_INPUT_PULLUP_S0D1)
        self.w32(pin_cnf(PG_PIN), GPIO_PIN_CNF_INPUT_PULLUP)
        self.w32(pin_cnf(CD_PIN), GPIO_PIN_CNF_OUTPUT)

    def setup_twim(self, frequency: int = FREQUENCY_400K) -> None:
        self.w32(TWIM1 + TWIM_ENABLE, 0)
        self.w32(TWIM1 + TWIM_PSEL_SDA, SDA_PIN)
        self.w32(TWIM1 + TWIM_PSEL_SCL, SCL_PIN)
        self.w32(TWIM1 + TWIM_FREQUENCY, frequency)
        self.w32(TWIM1 + TWIM_ENABLE, 6)

    def clear_twim_events(self) -> None:
        for offset in (EVENTS_STOPPED, EVENTS_ERROR, EVENTS_LASTRX, EVENTS_LASTTX):
            self.w32(TWIM1 + offset, 0)
        self.w32(TWIM1 + TWIM_ERRORSRC, 0xFFFFFFFF)

    def wait_twim(self, timeout_s: float = 0.050) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.r32(TWIM1 + EVENTS_STOPPED):
                return
            if self.r32(TWIM1 + EVENTS_ERROR):
                err = self.r32(TWIM1 + TWIM_ERRORSRC)
                amount_tx = self.r32(TWIM1 + TWIM_TXD_AMOUNT)
                amount_rx = self.r32(TWIM1 + TWIM_RXD_AMOUNT)
                raise BatteryDebugError(
                    f"TWIM error errsrc=0x{err:08x} tx_amount={amount_tx} rx_amount={amount_rx}"
                )
            time.sleep(0.001)
        self.w32(TWIM1 + TASKS_STOP, 1)
        raise BatteryDebugError("TWIM transaction timed out")

    def i2c_write(self, addr: int, data: Iterable[int]) -> None:
        payload = bytes(data)
        if not payload:
            raise ValueError("i2c_write payload must not be empty")
        self.w8(SCRATCH_TX, payload)
        self.clear_twim_events()
        self.w32(TWIM1 + TWIM_ADDRESS, addr)
        self.w32(TWIM1 + TWIM_TXD_PTR, SCRATCH_TX)
        self.w32(TWIM1 + TWIM_TXD_MAXCNT, len(payload))
        self.w32(TWIM1 + TWIM_RXD_PTR, SCRATCH_RX)
        self.w32(TWIM1 + TWIM_RXD_MAXCNT, 0)
        self.w32(TWIM1 + TWIM_SHORTS, SHORT_LASTTX_STOP)
        self.w32(TWIM1 + TASKS_STARTTX, 1)
        self.wait_twim()
        sent = self.r32(TWIM1 + TWIM_TXD_AMOUNT)
        self.w32(TWIM1 + TWIM_SHORTS, 0)
        if sent != len(payload):
            raise BatteryDebugError(f"short I2C write to 0x{addr:02x}: sent {sent}/{len(payload)}")

    def i2c_read_reg(self, addr: int, reg: int, count: int) -> list[int]:
        if count <= 0:
            raise ValueError("read count must be positive")
        self.w8(SCRATCH_TX, [reg & 0xFF])
        self.clear_twim_events()
        self.w32(TWIM1 + TWIM_ADDRESS, addr)
        self.w32(TWIM1 + TWIM_TXD_PTR, SCRATCH_TX)
        self.w32(TWIM1 + TWIM_TXD_MAXCNT, 1)
        self.w32(TWIM1 + TWIM_RXD_PTR, SCRATCH_RX)
        self.w32(TWIM1 + TWIM_RXD_MAXCNT, count)
        self.w32(TWIM1 + TWIM_SHORTS, SHORT_LASTTX_STARTRX | SHORT_LASTRX_STOP)
        self.w32(TWIM1 + TASKS_STARTTX, 1)
        self.wait_twim()
        rx_amount = self.r32(TWIM1 + TWIM_RXD_AMOUNT)
        self.w32(TWIM1 + TWIM_SHORTS, 0)
        if rx_amount != count:
            raise BatteryDebugError(f"short I2C read from 0x{addr:02x}: got {rx_amount}/{count}")
        return self.r8(SCRATCH_RX, count)

    def bq27220_u16(self, reg: int) -> int:
        return u16le(self.i2c_read_reg(BQ27220_ADDR, reg, 2))

    def bq27220_i16(self, reg: int) -> int:
        return i16le(self.i2c_read_reg(BQ27220_ADDR, reg, 2))

    def bq25120a_u8(self, reg: int) -> int:
        return self.i2c_read_reg(BQ25120A_ADDR, reg, 1)[0]

    def bq25120a_write_u8(self, reg: int, value: int) -> None:
        self.i2c_write(BQ25120A_ADDR, [reg & 0xFF, value & 0xFF])

    def raw_gpio0_in(self) -> int:
        return self.r32(GPIO0 + 0x10)

    def set_cd(self, raw_value: int) -> None:
        if raw_value:
            self.w32(GPIO0 + 0x508, 1 << CD_PIN)  # OUTSET
        else:
            self.w32(GPIO0 + 0x50C, 1 << CD_PIN)  # OUTCLR

    def read_fuel_gauge(self) -> FuelGaugeStatus:
        voltage_mv = self.bq27220_u16(0x08)
        try:
            temp_k_tenths = self.bq27220_u16(0x06)
            temp_c = temp_k_tenths / 10.0 - 273.15
        except Exception:
            temp_c = None
        return FuelGaugeStatus(
            voltage_mv=voltage_mv,
            temperature_c=temp_c,
            state_of_charge_pct=self._maybe_u16(0x2C),
            average_current_ma=self._maybe_i16(0x14),
            flags=self._maybe_u16(0x0A),
            gauging_status=self.read_gauging_status(),
        )

    def _maybe_u16(self, reg: int) -> int | None:
        try:
            return self.bq27220_u16(reg)
        except Exception:
            return None

    def _maybe_i16(self, reg: int) -> int | None:
        try:
            return self.bq27220_i16(reg)
        except Exception:
            return None

    def read_gauging_status(self) -> int | None:
        try:
            self.i2c_write(BQ27220_ADDR, [0x3E, 0x56, 0x00])
            time.sleep(0.002)
            return self.bq27220_u16(0x40)
        except Exception:
            return None

    def read_charger(self) -> ChargerStatus:
        raw_in = self.raw_gpio0_in()
        return ChargerStatus(
            ctrl=self.bq25120a_u8(0x00),
            fault=self.bq25120a_u8(0x01),
            ts_fault=self.bq25120a_u8(0x02),
            charge_ctrl=self.bq25120a_u8(0x03),
            ilim_uvlo=self.bq25120a_u8(0x09),
            pg_present=not bool(raw_in & (1 << PG_PIN)),
            cd_raw=1 if raw_in & (1 << CD_PIN) else 0,
        )

    def configure_charger(
        self,
        charge_current_ma: int,
        termination_current_ma: float,
        target_voltage_mv: int,
        input_limit_ma: int,
        uvlo_mv: int,
    ) -> None:
        self.set_cd(0)
        self.bq25120a_write_u8(0x02, 0x00)  # TS disabled / clear TS fault bits
        self.bq25120a_write_u8(0x05, encode_target_voltage(target_voltage_mv))
        self.bq25120a_write_u8(0x03, encode_charge_current(charge_current_ma))
        self.bq25120a_write_u8(0x04, encode_termination_current(termination_current_ma))
        self.bq25120a_write_u8(0x09, encode_ilim_uvlo(input_limit_ma, uvlo_mv))

    def reset_charger(self) -> None:
        self.bq25120a_write_u8(0x09, 0x80)
        time.sleep(0.010)


def encode_charge_current(ma: int) -> int:
    ma = max(5, min(300, int(ma)))
    if ma >= 40:
        code = int(round((ma - 40) / 10)) & 0x1F
        return (1 << 7) | (code << 2)
    code = int(round(ma - 5)) & 0x1F
    return code << 2


def encode_termination_current(ma: float) -> int:
    ma = max(0.5, min(37.0, float(ma)))
    if ma >= 6:
        code = int(round(ma - 6)) & 0x1F
        value = (1 << 7) | (code << 2)
    else:
        code = int(round(2 * (ma - 0.5))) & 0x1F
        value = code << 2
    return value | 0x02


def encode_target_voltage(mv: int) -> int:
    volts = max(3.6, min(4.65, mv / 1000.0))
    return (int(round((volts - 3.6) * 100)) & 0x7F) << 1


def encode_ilim_uvlo(input_limit_ma: int, uvlo_mv: int) -> int:
    ilim = max(50, min(400, int(input_limit_ma)))
    uvlo = max(2200, min(3000, int(uvlo_mv))) / 1000.0
    ilim_code = int(round(ilim / 50 - 1)) & 0x07
    uvlo_code = int(round((3.0 - uvlo) * 5 + 2)) & 0x07
    return (ilim_code << 3) | uvlo_code


def format_status(fuel: FuelGaugeStatus, charger: ChargerStatus) -> str:
    fields = [
        f"voltage={fuel.voltage_mv} mV",
        f"charger={charger.charging_state}",
        f"timer_fault={charger.timer_fault}",
        f"BAT_UVLO={charger.bat_uvlo}",
        f"charge_enabled={charger.charge_enabled}",
        f"high_z={charger.high_z}",
        f"PG_present={charger.pg_present}",
        f"CD_raw={charger.cd_raw}",
    ]
    if fuel.temperature_c is not None:
        fields.append(f"temperature={fuel.temperature_c:.1f} C")
    if fuel.state_of_charge_pct is not None:
        fields.append(f"soc={fuel.state_of_charge_pct}%")
    if fuel.average_current_ma is not None:
        fields.append(f"avg_current={fuel.average_current_ma} mA")
    if fuel.flags is not None:
        fields.append(f"gauge_flags=0x{fuel.flags:04x}")
    fields.extend(
        [
            f"charger_ctrl=0x{charger.ctrl:02x}",
            f"charger_fault=0x{charger.fault:02x}",
            f"ts_fault=0x{charger.ts_fault:02x}",
            f"charge_ctrl=0x{charger.charge_ctrl:02x}",
            f"ilim_uvlo=0x{charger.ilim_uvlo:02x}",
        ]
    )
    if fuel.gauging_status is not None:
        fields.append(f"gauging_status=0x{fuel.gauging_status:04x}")
    return ", ".join(fields)


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--snr", help="J-Link serial number, for example 261010806")
    parser.add_argument("--speed-khz", type=int, default=DEFAULT_SPEED_KHZ)
    parser.add_argument(
        "--no-resume",
        action="store_true",
        help="Leave the app core halted after the SWD read/debug operation.",
    )


def open_link(args: argparse.Namespace) -> JLinkBatteryInterface:
    return JLinkBatteryInterface(args.snr, args.speed_khz, resume=not args.no_resume)


def cmd_voltage(args: argparse.Namespace) -> int:
    with open_link(args) as link:
        mv = link.bq27220_u16(0x08)
    if args.raw:
        print(mv)
    else:
        print(f"{mv} mV")
    return 0


def cmd_status(args: argparse.Namespace) -> int:
    with open_link(args) as link:
        fuel = link.read_fuel_gauge()
        charger = link.read_charger()
    print(format_status(fuel, charger))
    return 0


def cmd_recover(args: argparse.Namespace) -> int:
    with open_link(args) as link:
        fuel = link.read_fuel_gauge()
        charger = link.read_charger()
        print("initial:", format_status(fuel, charger))

        if fuel.voltage_mv < args.min_safe_mv and not args.allow_deep_discharge:
            print(
                f"Refusing recovery below {args.min_safe_mv} mV without "
                "--allow-deep-discharge.",
                file=sys.stderr,
            )
            return 2

        needs_recovery = fuel.voltage_mv < args.start_below_mv or args.force
        needs_fault_reset = args.reset_on_fault and charger.charging_state_code == 3
        if needs_recovery or needs_fault_reset or charger.timer_fault:
            print("resetting/configuring charger")
            link.reset_charger()
            link.configure_charger(
                charge_current_ma=args.charge_current_ma,
                termination_current_ma=args.termination_current_ma,
                target_voltage_mv=args.target_voltage_mv,
                input_limit_ma=args.input_limit_ma,
                uvlo_mv=args.uvlo_mv,
            )
        elif not args.continuous:
            print(
                f"Voltage is not below {args.start_below_mv} mV; no recovery needed. "
                "Use --force to reset/configure the charger anyway."
            )
            return 0

        started = time.monotonic()
        reset_count = 0
        last_reset = 0.0
        while True:
            fuel = link.read_fuel_gauge()
            charger = link.read_charger()
            print(time.strftime("%H:%M:%S"), format_status(fuel, charger), flush=True)

            if fuel.voltage_mv >= args.target_mv and not args.continuous:
                print(f"Target reached: {fuel.voltage_mv} mV >= {args.target_mv} mV")
                return 0

            if args.max_minutes and (time.monotonic() - started) > args.max_minutes * 60:
                print("Timed out before reaching target voltage.", file=sys.stderr)
                return 1

            fault_now = charger.timer_fault or (
                args.reset_on_fault and charger.charging_state_code == 3
            )
            cooldown_ok = (time.monotonic() - last_reset) >= args.fault_reset_cooldown_s
            if fault_now and cooldown_ok and reset_count < args.max_fault_resets:
                reset_count += 1
                last_reset = time.monotonic()
                print(f"fault/reset condition seen; reset {reset_count}/{args.max_fault_resets}")
                link.reset_charger()
                link.configure_charger(
                    charge_current_ma=args.charge_current_ma,
                    termination_current_ma=args.termination_current_ma,
                    target_voltage_mv=args.target_voltage_mv,
                    input_limit_ma=args.input_limit_ma,
                    uvlo_mv=args.uvlo_mv,
                )

            time.sleep(args.interval_s)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generic OpenEarable v2 battery debug helper over J-Link/SWD."
    )
    subparsers = parser.add_subparsers(dest="command")

    voltage = subparsers.add_parser("voltage", help="Read only the battery voltage.")
    add_common_args(voltage)
    voltage.add_argument("--raw", action="store_true", help="Print only integer millivolts.")
    voltage.set_defaults(func=cmd_voltage)

    status = subparsers.add_parser("status", help="Read fuel-gauge and charger status.")
    add_common_args(status)
    status.set_defaults(func=cmd_status)

    recover = subparsers.add_parser(
        "recover", help="Configure/reset charging and optionally monitor progress."
    )
    add_common_args(recover)
    recover.add_argument("--start-below-mv", type=int, default=3000)
    recover.add_argument("--target-mv", type=int, default=3300)
    recover.add_argument("--min-safe-mv", type=int, default=2500)
    recover.add_argument("--charge-current-ma", type=int, default=110)
    recover.add_argument("--termination-current-ma", type=float, default=10.0)
    recover.add_argument("--target-voltage-mv", type=int, default=4300)
    recover.add_argument("--input-limit-ma", type=int, default=200)
    recover.add_argument("--uvlo-mv", type=int, default=2500)
    recover.add_argument("--interval-s", type=float, default=10.0)
    recover.add_argument("--max-minutes", type=float, default=0.0)
    recover.add_argument("--max-fault-resets", "--max-timer-resets", type=int, default=3)
    recover.add_argument("--fault-reset-cooldown-s", type=float, default=30.0)
    recover.add_argument("--continuous", action="store_true")
    recover.add_argument("--reset-on-fault", action="store_true")
    recover.add_argument("--force", action="store_true")
    recover.add_argument("--allow-deep-discharge", action="store_true")
    recover.set_defaults(func=cmd_recover)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not hasattr(args, "func"):
        parser.print_help()
        return 2
    try:
        return args.func(args)
    except BatteryDebugError as exc:
        print(f"battery_debug: {exc}", file=sys.stderr)
        return 1
    except pylink.errors.JLinkException as exc:
        print(f"battery_debug: J-Link error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
