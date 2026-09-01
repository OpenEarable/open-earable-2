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
import math
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
RECOVERY_CHARGE_CURRENT_MA = 110
RECOVERY_PRETERM_CURRENT_MA = 10.0
RECOVERY_INPUT_LIMIT_MA = 200
RECOVERY_UVLO_MV = 2500

TWIM1 = 0x50009000
GPIO0 = 0x50842500
SCRATCH_TX = 0x20070000
SCRATCH_RX = 0x20070080
SCRATCH_BUFFER_SIZE = SCRATCH_RX - SCRATCH_TX

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
GPIO_PIN_CNF_OUTPUT_WITH_INPUT = 0x00000001

GPIO_OUTSET = 0x008
GPIO_OUTCLR = 0x00C
GPIO_IN = 0x010

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
# Bit 8 is LASTTX_SUSPEND on nRF5340; a completed write needs LASTTX_STOP.
SHORT_LASTTX_STOP = 1 << 9
SHORT_LASTRX_STOP = 1 << 12

TWIM_ERROR_OVERRUN = 1 << 0
TWIM_ERROR_ANACK = 1 << 1
TWIM_ERROR_DNACK = 1 << 2


class BatteryDebugError(RuntimeError):
    pass


def format_jlink_detail(exc: "pylink.errors.JLinkException") -> str:
    message = str(exc).strip() or getattr(exc, "message", "") or exc.__class__.__name__
    message = message.rstrip(".")
    code = getattr(exc, "code", None)
    if code is not None:
        return f"{message} (J-Link error code {code})."
    return f"{message}."


def jlink_failure_hint(operation: str, exc: "pylink.errors.JLinkException") -> str | None:
    message = str(exc).lower()
    unspecified = getattr(exc, "code", None) == -1 or "unspecified error" in message
    if not unspecified:
        return None

    operation = operation.lower()
    if "open" in operation:
        return (
            "Check that the J-Link is connected over USB and that --snr matches "
            "a probe listed by 'nrfjprog -i'."
        )
    if "connect" in operation:
        return (
            "Check target power and SWD wiring, confirm the app core can be "
            "debugged, and try a lower --speed-khz such as 100."
        )
    if "halt" in operation or "read" in operation or "write" in operation:
        return (
            "SWD access dropped after connecting; target power may be marginal. "
            "Check target power/SWD wiring and try --speed-khz 100."
        )
    return (
        "Check J-Link USB, target power, SWD wiring, --snr, and try a lower "
        "--speed-khz such as 100."
    )


def format_jlink_failure(operation: str, exc: "pylink.errors.JLinkException") -> str:
    message = f"J-Link {operation} failed: {format_jlink_detail(exc)}"
    hint = jlink_failure_hint(operation, exc)
    if hint:
        message += f" Hint: {hint}"
    return message


class JLinkOperationError(BatteryDebugError):
    def __init__(self, operation: str, exc: "pylink.errors.JLinkException"):
        self.operation = operation
        self.original = exc
        super().__init__(format_jlink_failure(operation, exc))


def u16le(data: Iterable[int]) -> int:
    raw = bytes(data)
    return int.from_bytes(raw[:2], "little", signed=False)


def i16le(data: Iterable[int]) -> int:
    raw = bytes(data)
    return int.from_bytes(raw[:2], "little", signed=True)


def describe_twim_error(error_source: int) -> str:
    reasons = []
    if error_source & TWIM_ERROR_OVERRUN:
        reasons.append("EasyDMA overrun")
    if error_source & TWIM_ERROR_ANACK:
        reasons.append("address NACK")
    if error_source & TWIM_ERROR_DNACK:
        reasons.append("data NACK")
    unknown = error_source & ~(
        TWIM_ERROR_OVERRUN | TWIM_ERROR_ANACK | TWIM_ERROR_DNACK
    )
    if unknown:
        reasons.append(f"unknown bits 0x{unknown:08x}")
    return "+".join(reasons) if reasons else "no error source reported"


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
    preterm_ctrl: int
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
    def reset_fault(self) -> bool:
        return bool(self.ctrl & (1 << 4))

    @property
    def timer_fault(self) -> bool:
        return bool(self.ctrl & (1 << 3))

    @property
    def vindpm_active(self) -> bool:
        return bool(self.ctrl & (1 << 2))

    @property
    def cd_stat(self) -> bool:
        return bool(self.ctrl & (1 << 1))

    @property
    def sys_enabled(self) -> bool:
        return bool(self.ctrl & 0x01)

    @property
    def fault_reasons(self) -> list[str]:
        reasons = []
        if self.timer_fault:
            reasons.append("safety_timer")
        if self.vin_overvoltage:
            reasons.append("VIN_OV")
        if self.vin_undervoltage:
            reasons.append("VIN_UV")
        if self.bat_uvlo:
            reasons.append("BAT_UVLO")
        if self.battery_overcurrent:
            reasons.append("BAT_OCP")
        if self.ts_fault_code:
            reasons.append(self.ts_state)
        if self.vindpm_active and self.charging_state_code == 3:
            reasons.append("VINDPM")
        if self.charging_state_code == 3 and not reasons:
            reasons.append("unknown_status_fault")
        return reasons

    @property
    def blocking_fault_reasons(self) -> list[str]:
        reasons = []
        if self.vin_overvoltage:
            reasons.append("VIN_OV")
        if self.battery_overcurrent:
            reasons.append("BAT_OCP")
        if self.ts_fault_code == 1:
            reasons.append(self.ts_state)
        return reasons

    @property
    def vin_overvoltage(self) -> bool:
        return bool(self.fault & (1 << 7))

    @property
    def vin_undervoltage(self) -> bool:
        return bool(self.fault & (1 << 6))

    @property
    def bat_uvlo(self) -> bool:
        return bool(self.fault & (1 << 5))

    @property
    def battery_overcurrent(self) -> bool:
        return bool(self.fault & (1 << 4))

    @property
    def ts_enabled(self) -> bool:
        return bool(self.ts_fault & (1 << 7))

    @property
    def ts_fault_code(self) -> int:
        return (self.ts_fault >> 5) & 0x03

    @property
    def ts_state(self) -> str:
        return {
            0: "TS_normal",
            1: "TS_hot_or_cold",
            2: "TS_cool_current_reduced",
            3: "TS_warm_voltage_reduced",
        }[self.ts_fault_code]

    @property
    def charge_enabled(self) -> bool:
        return not bool(self.charge_ctrl & 0x02)

    @property
    def high_z(self) -> bool:
        return bool(self.charge_ctrl & 0x01)


class JLinkBatteryInterface:
    def __init__(
        self,
        snr: int | str | None,
        speed_khz: int,
        resume: bool = True,
        reset_target: bool = False,
    ):
        self.snr = int(snr) if snr is not None else None
        self.speed_khz = speed_khz
        self.resume = resume
        self.reset_target = reset_target
        self.jlink = pylink.JLink()
        self.was_halted = False
        self.probe_open = False
        self.target_connected = False
        self.resume_on_exit = False

    def __enter__(self) -> "JLinkBatteryInterface":
        try:
            with self.jlink_operation(f"open probe {self.probe_description()}"):
                self.jlink.open(serial_no=self.snr)
            self.probe_open = True
            with self.jlink_operation("select SWD interface"):
                self.jlink.set_tif(JLinkInterfaces.SWD)
            with self.jlink_operation(
                f"connect to {APP_CORE_DEVICE} at {self.speed_khz} kHz"
            ):
                self.jlink.connect(APP_CORE_DEVICE, speed=self.speed_khz, verbose=False)
            self.target_connected = True
            if self.reset_target:
                self.resume_on_exit = True
                self.reset_and_halt_target()
                self.was_halted = False
            else:
                with self.jlink_operation("read target halt state"):
                    self.was_halted = bool(self.jlink.halted())
                if not self.was_halted:
                    self.resume_on_exit = True
                    with self.jlink_operation("halt target"):
                        self.jlink.halt()
            self.setup_gpio()
            self.setup_twim()
            return self
        except BaseException:
            self.cleanup_after_enter_failure()
            raise

    def cleanup_after_enter_failure(self) -> None:
        if self.target_connected:
            with contextlib.suppress(Exception):
                self.w32(TWIM1 + TWIM_ENABLE, 0)
        if self.resume and self.resume_on_exit:
            with contextlib.suppress(Exception):
                self.resume_target()
        with contextlib.suppress(Exception):
            self.jlink.close()
        self.probe_open = False

    def __exit__(self, exc_type, exc, tb) -> None:
        resume_error = None
        with contextlib.suppress(Exception):
            self.w32(TWIM1 + TWIM_ENABLE, 0)
        if self.resume and self.resume_on_exit:
            try:
                self.resume_target()
            except Exception as restart_exc:
                if exc_type is None:
                    resume_error = restart_exc
                else:
                    print(
                        f"warning: failed to resume application core: {restart_exc}",
                        file=sys.stderr,
                    )
        if self.probe_open:
            with contextlib.suppress(Exception):
                self.jlink.close()
            self.probe_open = False
        if resume_error is not None:
            raise resume_error

    def probe_description(self) -> str:
        if self.snr is not None:
            return f"serial {self.snr}"
        return "auto-selected by PyLink"

    def reset_and_halt_target(self) -> None:
        with self.jlink_operation("reset and halt target"):
            self.jlink.reset(ms=10, halt=True)
        time.sleep(0.010)

    def resume_target(self) -> None:
        with self.jlink_operation("resume application core"):
            self.jlink.restart()
            time.sleep(0.010)
            if self.jlink.halted():
                raise BatteryDebugError("application core remained halted after restart")

    def recover_target_state(self) -> None:
        if self.reset_target:
            self.reset_and_halt_target()
            self.setup_gpio()
        self.setup_twim()

    @contextlib.contextmanager
    def jlink_operation(self, operation: str):
        try:
            yield
        except pylink.errors.JLinkException as exc:
            raise JLinkOperationError(operation, exc) from exc

    def r32(self, addr: int) -> int:
        with self.jlink_operation(f"read 32-bit word at 0x{addr:08x}"):
            values = self.jlink.memory_read32(addr, 1)
        if len(values) != 1:
            raise BatteryDebugError(
                f"J-Link returned {len(values)} words while reading 0x{addr:08x}; expected 1"
            )
        return values[0]

    def w32(self, addr: int, value: int) -> None:
        with self.jlink_operation(f"write 32-bit word at 0x{addr:08x}"):
            self.jlink.memory_write32(addr, [value & 0xFFFFFFFF])

    def r8(self, addr: int, count: int) -> list[int]:
        with self.jlink_operation(f"read {count} byte(s) at 0x{addr:08x}"):
            values = list(self.jlink.memory_read8(addr, count))
        if len(values) != count:
            raise BatteryDebugError(
                f"J-Link returned {len(values)} byte(s) while reading 0x{addr:08x}; "
                f"expected {count}"
            )
        return values

    def w8(self, addr: int, data: Iterable[int]) -> None:
        payload = list(data)
        with self.jlink_operation(f"write {len(payload)} byte(s) at 0x{addr:08x}"):
            self.jlink.memory_write8(addr, payload)

    def setup_gpio(self) -> None:
        self.w32(pin_cnf(SDA_PIN), GPIO_PIN_CNF_INPUT_PULLUP_S0D1)
        self.w32(pin_cnf(SCL_PIN), GPIO_PIN_CNF_INPUT_PULLUP_S0D1)
        self.w32(pin_cnf(PG_PIN), GPIO_PIN_CNF_INPUT_PULLUP)
        self.w32(pin_cnf(CD_PIN), GPIO_PIN_CNF_OUTPUT_WITH_INPUT)

    def setup_twim(self, frequency: int = FREQUENCY_400K) -> None:
        self.w32(TWIM1 + TWIM_ENABLE, 0)
        self.w32(TWIM1 + TWIM_SHORTS, 0)
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
            if self.r32(TWIM1 + EVENTS_ERROR):
                err = self.r32(TWIM1 + TWIM_ERRORSRC)
                amount_tx = self.r32(TWIM1 + TWIM_TXD_AMOUNT)
                amount_rx = self.r32(TWIM1 + TWIM_RXD_AMOUNT)
                stopped = self.stop_twim()
                raise BatteryDebugError(
                    f"TWIM error ({describe_twim_error(err)}; "
                    f"errsrc=0x{err:08x}, tx_amount={amount_tx}, "
                    f"rx_amount={amount_rx}, stop={'ok' if stopped else 'timed out'})"
                )
            if self.r32(TWIM1 + EVENTS_STOPPED):
                return
            time.sleep(0.001)
        stopped = self.stop_twim()
        err = self.r32(TWIM1 + TWIM_ERRORSRC)
        amount_tx = self.r32(TWIM1 + TWIM_TXD_AMOUNT)
        amount_rx = self.r32(TWIM1 + TWIM_RXD_AMOUNT)
        gpio_in = self.raw_gpio0_in()
        scl = "high" if gpio_in & (1 << SCL_PIN) else "low"
        sda = "high" if gpio_in & (1 << SDA_PIN) else "low"
        raise BatteryDebugError(
            f"TWIM transaction timed out after {timeout_s * 1000:g} ms "
            f"(SCL={scl}, SDA={sda}, {describe_twim_error(err)}, "
            f"tx_amount={amount_tx}, rx_amount={amount_rx}, "
            f"stop={'ok' if stopped else 'timed out'}). "
            "A low SCL/SDA level indicates a stuck I2C bus; high lines usually "
            "mean the addressed battery IC is not responding."
        )

    def stop_twim(self, timeout_s: float = 0.010) -> bool:
        self.w32(TWIM1 + TASKS_STOP, 1)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.r32(TWIM1 + EVENTS_STOPPED):
                return True
            time.sleep(0.001)
        return False

    def i2c_write(self, addr: int, data: Iterable[int]) -> None:
        payload = bytes(data)
        if not payload:
            raise ValueError("i2c_write payload must not be empty")
        if len(payload) > SCRATCH_BUFFER_SIZE:
            raise ValueError(
                f"i2c_write payload exceeds {SCRATCH_BUFFER_SIZE}-byte scratch buffer"
            )
        self.setup_twim()
        self.w8(SCRATCH_TX, payload)
        self.clear_twim_events()
        self.w32(TWIM1 + TWIM_ADDRESS, addr)
        self.w32(TWIM1 + TWIM_TXD_PTR, SCRATCH_TX)
        self.w32(TWIM1 + TWIM_TXD_MAXCNT, len(payload))
        self.w32(TWIM1 + TWIM_RXD_PTR, SCRATCH_RX)
        self.w32(TWIM1 + TWIM_RXD_MAXCNT, 0)
        self.w32(TWIM1 + TWIM_SHORTS, SHORT_LASTTX_STOP)
        try:
            self.w32(TWIM1 + TASKS_STARTTX, 1)
            self.wait_twim()
            sent = self.r32(TWIM1 + TWIM_TXD_AMOUNT)
        finally:
            self.w32(TWIM1 + TWIM_SHORTS, 0)
        if sent != len(payload):
            raise BatteryDebugError(f"short I2C write to 0x{addr:02x}: sent {sent}/{len(payload)}")

    def i2c_read_reg(self, addr: int, reg: int, count: int) -> list[int]:
        if count <= 0:
            raise ValueError("read count must be positive")
        if count > SCRATCH_BUFFER_SIZE:
            raise ValueError(
                f"read count exceeds {SCRATCH_BUFFER_SIZE}-byte scratch buffer"
            )
        self.setup_twim()
        self.w8(SCRATCH_TX, [reg & 0xFF])
        self.w8(SCRATCH_RX, [0] * count)
        self.clear_twim_events()
        self.w32(TWIM1 + TWIM_ADDRESS, addr)
        self.w32(TWIM1 + TWIM_TXD_PTR, SCRATCH_TX)
        self.w32(TWIM1 + TWIM_TXD_MAXCNT, 1)
        self.w32(TWIM1 + TWIM_RXD_PTR, SCRATCH_RX)
        self.w32(TWIM1 + TWIM_RXD_MAXCNT, count)
        self.w32(TWIM1 + TWIM_SHORTS, SHORT_LASTTX_STARTRX | SHORT_LASTRX_STOP)
        try:
            self.w32(TWIM1 + TASKS_STARTTX, 1)
            self.wait_twim()
            rx_amount = self.r32(TWIM1 + TWIM_RXD_AMOUNT)
            last_rx = self.r32(TWIM1 + EVENTS_LASTRX)
        finally:
            self.w32(TWIM1 + TWIM_SHORTS, 0)
        if not last_rx:
            raise BatteryDebugError(
                f"I2C read from 0x{addr:02x} register 0x{reg:02x} did not "
                "signal LASTRX"
            )
        if rx_amount != count:
            raise BatteryDebugError(f"short I2C read from 0x{addr:02x}: got {rx_amount}/{count}")
        return self.r8(SCRATCH_RX, count)

    def bq27220_u16(self, reg: int) -> int:
        try:
            return u16le(self.i2c_read_reg(BQ27220_ADDR, reg, 2))
        except BatteryDebugError as exc:
            raise BatteryDebugError(f"BQ27220 read reg 0x{reg:02x} failed: {exc}") from exc

    def bq27220_i16(self, reg: int) -> int:
        try:
            return i16le(self.i2c_read_reg(BQ27220_ADDR, reg, 2))
        except BatteryDebugError as exc:
            raise BatteryDebugError(f"BQ27220 read reg 0x{reg:02x} failed: {exc}") from exc

    def bq25120a_u8(self, reg: int) -> int:
        try:
            time.sleep(0.001)
            return self.i2c_read_reg(BQ25120A_ADDR, reg, 1)[0]
        except BatteryDebugError as exc:
            raise BatteryDebugError(f"BQ25120A read reg 0x{reg:02x} failed: {exc}") from exc

    def bq25120a_write_u8(self, reg: int, value: int) -> None:
        try:
            time.sleep(0.001)
            self.i2c_write(BQ25120A_ADDR, [reg & 0xFF, value & 0xFF])
        except BatteryDebugError as exc:
            raise BatteryDebugError(
                f"BQ25120A write reg 0x{reg:02x} <- 0x{value & 0xFF:02x} failed: {exc}"
            ) from exc

    def raw_gpio0_in(self) -> int:
        return self.r32(GPIO0 + GPIO_IN)

    def set_cd(self, raw_value: int) -> None:
        if raw_value:
            self.w32(GPIO0 + GPIO_OUTSET, 1 << CD_PIN)
        else:
            self.w32(GPIO0 + GPIO_OUTCLR, 1 << CD_PIN)
        time.sleep(0.001)

    def read_fuel_gauge(self, include_gauging_status: bool = False) -> FuelGaugeStatus:
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
            gauging_status=(
                self.read_gauging_status() if include_gauging_status else None
            ),
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
            preterm_ctrl=self.bq25120a_u8(0x04),
            ilim_uvlo=self.bq25120a_u8(0x09),
            pg_present=not bool(raw_in & (1 << PG_PIN)),
            cd_raw=1 if raw_in & (1 << CD_PIN) else 0,
        )

    def configure_charger(self) -> None:
        self.set_cd(0)
        writes = [
            (
                0x03,
                encode_charge_current(RECOVERY_CHARGE_CURRENT_MA),
                "set charge current",
            ),
            (
                0x04,
                encode_termination_current(RECOVERY_PRETERM_CURRENT_MA),
                "set precharge/termination current",
            ),
            (
                0x09,
                encode_ilim_uvlo(RECOVERY_INPUT_LIMIT_MA, RECOVERY_UVLO_MV),
                "set input limit/UVLO",
            ),
        ]
        for reg, value, description in writes:
            try:
                self.bq25120a_write_u8(reg, value)
            except BatteryDebugError as exc:
                raise BatteryDebugError(f"failed to {description}: {exc}") from exc

    def reset_charger(self) -> None:
        # The BQ25120A safety-timer latch is cleared by toggling CD or power.
        try:
            self.set_cd(1)
            if not (self.raw_gpio0_in() & (1 << CD_PIN)):
                raise BatteryDebugError("charger CD pin did not go high")
            time.sleep(0.020)
        finally:
            self.set_cd(0)
        if self.raw_gpio0_in() & (1 << CD_PIN):
            raise BatteryDebugError("charger CD pin did not return low")
        time.sleep(0.020)
        self.bq25120a_write_u8(0x09, 0x80)
        time.sleep(0.010)
        self.set_cd(0)


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
        f"reset_fault={charger.reset_fault}",
        f"BAT_UVLO={charger.bat_uvlo}",
        f"charge_enabled={charger.charge_enabled}",
        f"high_z={charger.high_z}",
        f"VINDPM={charger.vindpm_active}",
        f"TS_enabled={charger.ts_enabled}",
        f"TS_state={charger.ts_state}",
        f"CD_stat={charger.cd_stat}",
        f"SYS_enabled={charger.sys_enabled}",
        f"PG_present={charger.pg_present}",
        f"CD_raw={charger.cd_raw}",
    ]
    if charger.fault_reasons:
        fields.append(f"fault_reason={'+'.join(charger.fault_reasons)}")
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
            f"preterm_ctrl=0x{charger.preterm_ctrl:02x}",
            f"ilim_uvlo=0x{charger.ilim_uvlo:02x}",
        ]
    )
    if fuel.gauging_status is not None:
        fields.append(f"gauging_status=0x{fuel.gauging_status:04x}")
    return ", ".join(fields)


def positive_int_arg(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def nonnegative_int_arg(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def positive_float_arg(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number") from exc
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return parsed


def nonnegative_float_arg(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number") from exc
    if not math.isfinite(parsed) or parsed < 0:
        raise argparse.ArgumentTypeError("must be a finite number zero or greater")
    return parsed


def monitoring_interval_arg(value: str) -> float:
    parsed = positive_float_arg(value)
    if parsed > 30:
        raise argparse.ArgumentTypeError(
            "must be 30 seconds or less to service the charger's watchdog"
        )
    return parsed


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--snr",
        type=positive_int_arg,
        help="J-Link serial number, for example 261010806",
    )
    parser.add_argument("--speed-khz", type=positive_int_arg, default=DEFAULT_SPEED_KHZ)
    reset_group = parser.add_mutually_exclusive_group()
    reset_group.add_argument(
        "--reset-target",
        dest="reset_target",
        action="store_true",
        help="Reset and halt the app core before using TWIM.",
    )
    reset_group.add_argument(
        "--no-reset-target",
        dest="reset_target",
        action="store_false",
        help="Do not reset the app core before using TWIM.",
    )
    parser.set_defaults(reset_target=False)
    parser.add_argument(
        "--no-resume",
        action="store_true",
        help="Leave the app core halted after the SWD read/debug operation.",
    )


def open_link(args: argparse.Namespace) -> JLinkBatteryInterface:
    return JLinkBatteryInterface(
        args.snr,
        args.speed_khz,
        resume=not args.no_resume,
        reset_target=args.reset_target,
    )


def charger_blocking_reasons(charger: ChargerStatus) -> list[str]:
    reasons = []
    if not charger.pg_present:
        reasons.append("input_power_missing")
    reasons.extend(charger.blocking_fault_reasons)
    return reasons


def charger_recovery_reason(charger: ChargerStatus, reset_on_fault: bool) -> str | None:
    if charger.timer_fault:
        return "safety timer fault"
    if not reset_on_fault:
        return None
    if charger.cd_stat or charger.cd_raw:
        return "charger disable pin is high"
    if not charger.charge_enabled:
        return "charging is disabled"
    if charger.high_z:
        return "charger is in high-impedance mode"
    if not charger.sys_enabled:
        return "charger system output is disabled"
    known_nonblocking_fault = (
        charger.bat_uvlo
        or charger.vin_undervoltage
        or charger.vindpm_active
        or charger.ts_fault_code in (2, 3)
    )
    if (
        charger.charging_state_code == 3
        and not charger.blocking_fault_reasons
        and not known_nonblocking_fault
    ):
        return "unclassified charger status fault"
    return None


def reset_and_configure_charger(link: JLinkBatteryInterface) -> ChargerStatus:
    last_error = None
    for attempt in range(2):
        try:
            link.reset_charger()
            link.configure_charger()
            time.sleep(0.050)
            charger = link.read_charger()
            if charger_blocking_reasons(charger):
                return charger
            if charger.timer_fault:
                raise BatteryDebugError(
                    "safety timer fault remained set after the CD pulse"
                )
            if charger.cd_stat or charger.cd_raw:
                raise BatteryDebugError("charger CD pin remained high after reset")
            if not charger.charge_enabled:
                raise BatteryDebugError("charging remained disabled after configuration")
            if charger.high_z:
                raise BatteryDebugError(
                    "charger remained in high-impedance mode after configuration"
                )
            if not charger.sys_enabled:
                raise BatteryDebugError(
                    "charger system output remained disabled after reset"
                )
            expected_charge_ctrl = encode_charge_current(RECOVERY_CHARGE_CURRENT_MA)
            if charger.charge_ctrl != expected_charge_ctrl:
                raise BatteryDebugError(
                    "charger current configuration did not stick: "
                    f"read 0x{charger.charge_ctrl:02x}, expected "
                    f"0x{expected_charge_ctrl:02x}"
                )
            expected_preterm_ctrl = encode_termination_current(
                RECOVERY_PRETERM_CURRENT_MA
            )
            if charger.preterm_ctrl != expected_preterm_ctrl:
                raise BatteryDebugError(
                    "charger precharge/termination configuration did not stick: "
                    f"read 0x{charger.preterm_ctrl:02x}, expected "
                    f"0x{expected_preterm_ctrl:02x}"
                )
            expected_ilim_uvlo = encode_ilim_uvlo(
                RECOVERY_INPUT_LIMIT_MA, RECOVERY_UVLO_MV
            )
            if charger.ilim_uvlo != expected_ilim_uvlo:
                raise BatteryDebugError(
                    "charger input-limit/UVLO configuration did not stick: "
                    f"read 0x{charger.ilim_uvlo:02x}, expected "
                    f"0x{expected_ilim_uvlo:02x}"
                )
            return charger
        except BatteryDebugError as exc:
            last_error = exc
            if attempt == 0:
                print(
                    f"warning: charger recovery attempt failed; retrying once: {exc}",
                    file=sys.stderr,
                )
                link.recover_target_state()
    raise BatteryDebugError(f"charger recovery failed after two attempts: {last_error}")


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
        fuel = link.read_fuel_gauge(include_gauging_status=args.extended_gauge_status)
        charger = link.read_charger()
    print(format_status(fuel, charger))
    return 0


def cmd_recover(args: argparse.Namespace) -> int:
    with open_link(args) as link:
        fuel = link.read_fuel_gauge()
        charger = link.read_charger()
        print("initial:", format_status(fuel, charger))

        blockers = charger_blocking_reasons(charger)
        if blockers:
            print(
                "Recovery cannot continue while blocking fault(s) are active: "
                f"{'+'.join(blockers)}.",
                file=sys.stderr,
            )
            return 1

        if fuel.voltage_mv < args.min_safe_mv and not args.allow_deep_discharge:
            print(
                f"Refusing recovery below {args.min_safe_mv} mV without "
                "--allow-deep-discharge.",
                file=sys.stderr,
            )
            return 2

        needs_recovery = fuel.voltage_mv < args.start_below_mv or args.force
        reset_reason = charger_recovery_reason(charger, args.reset_on_fault)
        available_reset = charger_recovery_reason(charger, True)
        if needs_recovery or reset_reason:
            if args.force:
                reason = "forced"
            elif fuel.voltage_mv < args.start_below_mv:
                reason = f"voltage below {args.start_below_mv} mV"
            else:
                reason = reset_reason
            print(f"resetting/configuring charger ({reason})")
            charger = reset_and_configure_charger(link)
            print("after reset:", format_status(fuel, charger))
            blockers = charger_blocking_reasons(charger)
            if blockers:
                print(
                    "Recovery stopped because blocking fault(s) remained after "
                    f"reset: {'+'.join(blockers)}.",
                    file=sys.stderr,
                )
                return 1
        elif available_reset:
            print(
                f"Recovery stopped: {available_reset}. Rerun with --reset-on-fault "
                "to allow an automatic charger reset.",
                file=sys.stderr,
            )
            return 1
        elif not args.continuous:
            print(
                f"Voltage is not below {args.start_below_mv} mV; no recovery needed. "
                "Use --force to reset/configure the charger anyway."
            )
            return 0

        started = time.monotonic()
        progress_started = started
        progress_voltage_mv = fuel.voltage_mv
        reset_count = 0
        last_reset = 0.0
        while True:
            fuel = link.read_fuel_gauge()
            charger = link.read_charger()
            print(time.strftime("%H:%M:%S"), format_status(fuel, charger), flush=True)

            blockers = charger_blocking_reasons(charger)
            if blockers:
                print(
                    "Recovery stopped because blocking fault(s) became active: "
                    f"{'+'.join(blockers)}.",
                    file=sys.stderr,
                )
                return 1

            reset_reason = charger_recovery_reason(charger, args.reset_on_fault)
            available_reset = charger_recovery_reason(charger, True)
            if available_reset and not reset_reason:
                print(
                    f"Recovery stopped: {available_reset}. Rerun with "
                    "--reset-on-fault to allow an automatic charger reset.",
                    file=sys.stderr,
                )
                return 1

            if reset_reason is None and fuel.voltage_mv >= args.target_mv and not args.continuous:
                if args.no_resume:
                    action = "Leaving the application core halted (--no-resume)."
                else:
                    action = "Restarting the application core."
                print(
                    f"Target reached with no blocking charger fault: "
                    f"{fuel.voltage_mv} mV >= {args.target_mv} mV. {action}"
                )
                return 0

            now = time.monotonic()
            if args.max_minutes and (now - started) > args.max_minutes * 60:
                print("Recovery monitoring timed out.", file=sys.stderr)
                return 1

            if reset_reason:
                cooldown_ok = (now - last_reset) >= args.fault_reset_cooldown_s
                if cooldown_ok:
                    if reset_count >= args.max_fault_resets:
                        print(
                            f"Recovery stopped: {reset_reason} is still present and "
                            f"the reset limit ({args.max_fault_resets}) was reached.",
                            file=sys.stderr,
                        )
                        return 1
                    reset_count += 1
                    last_reset = time.monotonic()
                    print(
                        f"{reset_reason} seen; reset "
                        f"{reset_count}/{args.max_fault_resets}"
                    )
                    charger = reset_and_configure_charger(link)
                    print("after reset:", format_status(fuel, charger))
                    blockers = charger_blocking_reasons(charger)
                    if blockers:
                        print(
                            "Recovery stopped because blocking fault(s) remained "
                            f"after reset: {'+'.join(blockers)}.",
                            file=sys.stderr,
                        )
                        return 1
                    progress_started = time.monotonic()
                    progress_voltage_mv = fuel.voltage_mv
                time.sleep(args.interval_s)
                continue

            progress_voltage_mv = min(progress_voltage_mv, fuel.voltage_mv)
            if fuel.voltage_mv >= progress_voltage_mv + args.stall_min_rise_mv:
                progress_started = time.monotonic()
                progress_voltage_mv = fuel.voltage_mv
            elif (
                args.stall_minutes
                and fuel.voltage_mv < args.target_mv
                and (time.monotonic() - progress_started) > args.stall_minutes * 60
            ):
                print(
                    "Recovery stalled: voltage did not rise by at least "
                    f"{args.stall_min_rise_mv} mV within {args.stall_minutes:g} "
                    "minute(s). Check the USB supply, battery, and device temperature.",
                    file=sys.stderr,
                )
                return 1

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
    status.add_argument(
        "--extended-gauge-status",
        action="store_true",
        help=(
            "Also request BQ27220 gauging-status data. This is optional and may "
            "wedge I2C on low-power boards."
        ),
    )
    status.set_defaults(func=cmd_status)

    recover = subparsers.add_parser(
        "recover", help="Configure/reset charging and optionally monitor progress."
    )
    add_common_args(recover)
    recover.add_argument("--start-below-mv", type=positive_int_arg, default=3000)
    recover.add_argument("--target-mv", type=positive_int_arg, default=3300)
    recover.add_argument("--min-safe-mv", type=positive_int_arg, default=2500)
    recover.add_argument("--interval-s", type=monitoring_interval_arg, default=10.0)
    recover.add_argument("--max-minutes", type=nonnegative_float_arg, default=0.0)
    recover.add_argument(
        "--stall-minutes",
        type=nonnegative_float_arg,
        default=10.0,
        help="Stop if voltage makes no meaningful progress for this long; 0 disables.",
    )
    recover.add_argument(
        "--stall-min-rise-mv",
        type=positive_int_arg,
        default=10,
        help="Voltage rise that resets the stall timer.",
    )
    recover.add_argument(
        "--max-fault-resets",
        "--max-timer-resets",
        type=nonnegative_int_arg,
        default=3,
    )
    recover.add_argument(
        "--fault-reset-cooldown-s", type=nonnegative_float_arg, default=30.0
    )
    recover.add_argument("--continuous", action="store_true")
    recover.add_argument("--reset-on-fault", action="store_true")
    recover.add_argument("--force", action="store_true")
    recover.add_argument("--allow-deep-discharge", action="store_true")
    recover.set_defaults(func=cmd_recover, reset_target=True)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not hasattr(args, "func"):
        parser.print_help()
        return 2
    try:
        return args.func(args)
    except KeyboardInterrupt:
        print("battery_debug: interrupted.", file=sys.stderr)
        return 130
    except BatteryDebugError as exc:
        print(f"battery_debug: {exc}", file=sys.stderr)
        return 1
    except pylink.errors.JLinkException as exc:
        print(f"battery_debug: {format_jlink_failure('operation', exc)}", file=sys.stderr)
        return 1
    except OSError as exc:
        print(
            f"battery_debug: operating-system error: {exc}. Check that J-Link "
            "software 8.82 is installed and the probe is available.",
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
