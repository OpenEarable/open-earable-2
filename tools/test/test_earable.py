"""Check OpenEarable 2 main-board peripherals over J-Link SWD."""

import argparse
import signal
import sys
import time

from pylink.errors import JLinkException

from audio_check import AudioCheck
from components import IMU, LEDDriver
from hardware import Cancelled, HardwareError, I2CBus, SPIBus, Target, TWIM1, TWIM2, TWIM3, list_probes
from storage import NORFlash, SDCard


def read_charger_registers(bus):
    """Check reserved bits before relying on the BQ25120A register values."""
    ldo = bus.read(0x6A, 0x07)[0]
    input_limit = bus.read(0x6A, 0x09)[0]
    if ldo & 0x02 or input_limit & 0xC0:
        raise HardwareError(f"Invalid BQ25120A registers (0x07={ldo:#04x}, 0x09={input_limit:#04x})")
    return ldo, input_limit


def check_charger(target, bus):
    """Read the charger registers and report USB power good."""
    ldo, input_limit = read_charger_registers(bus)
    target.gpio_input(18, pullup=True)
    power_good = "active" if target.gpio_level(18) == 0 else "inactive"
    return f"LDO={ldo:#04x}, ILIM/UVLO={input_limit:#04x}, USB power-good {power_good}"


def check_dac(target, bus):
    """Wake the ADAU1860 and verify its vendor and product ID over I²C."""
    target.gpio_output(4, True)  # P0.04, codec PD#
    time.sleep(0.04)
    identity = bus.read(0x64, 0x4000C000, 4, width=4)
    if identity[:3] != bytes.fromhex("416018"):
        raise HardwareError(f"Expected ADAU1860 ID 416018, read {identity.hex()}")
    return f"ID {identity[:3].hex()}, revision {identity[3]:#04x}"


def check_flash(target, bus):
    """Verify the external NOR JEDEC ID and read from the device."""
    target.gpio_output(45, True)  # P1.13, NOR RESET#
    bus.setup()
    flash = NORFlash(bus)
    identity = flash.identify()
    if flash.read(0, 32) != flash.read(0, 32):
        raise HardwareError("NOR read-back differs between reads")
    return f"JEDEC {identity['jedec_id']}, 32 bytes read twice"


def check_sd(target, bus):
    """Read card identity and one block, with SD data CRC verification."""
    target.gpio_output(45, False)  # Keep the NOR in reset while using the shared SPI bus.
    target.gpio_output(44, True)  # P1.12, SD power switch
    time.sleep(0.02)
    target.gpio_input(15, pullup=True)  # P0.15, active-low card detect
    detect = "inactive" if target.gpio_level(15) else "active"
    card = SDCard(bus)
    identity = card.initialize()
    card.read_sector(0)
    return (f"CID {identity['cid']}, {identity['capacity_bytes']} bytes, "
            f"block 0 CRC OK, card-detect {detect}")


def run_tests(serial, check_cancel):
    """Run the bus checks and restore the target even if an operation fails."""
    target = Target(serial, check_cancel)
    power_bus = I2CBus(target, TWIM1, 21, 24)
    codec_bus = I2CBus(target, TWIM2, 47, 32)
    imu_bus = I2CBus(target, TWIM3, 35, 34)
    imu = IMU(imu_bus)
    spi_bus = SPIBus(target)
    original_ldo = None
    passed = 0
    restore_failed = False

    def write_ldo(value):
        power_bus.write(0x6A, 0x07, value)
        actual = power_bus.read(0x6A, 0x07)[0]
        if actual != value:
            raise HardwareError(f"BQ25120A LDO readback {actual:#04x}, expected {value:#04x}")

    def enable_3v3():
        nonlocal original_ldo
        # The charger resets to pass-through. Select the board's normal 3.3 V
        # setting before enabling LSCTRL, and restore the previous value on exit.
        current, _ = read_charger_registers(power_bus)
        regulated = (current & 0x01) | (25 << 2)
        if (current & 0x7F) != regulated:
            if original_ldo is None:
                original_ldo = current
            # Both enable paths must be off before the charger accepts a new voltage.
            target.gpio_output(44, False)  # Power down SD before the shared rail.
            target.gpio_output(14, False)
            if current & 0x80:
                write_ldo(current & 0x7F)
            time.sleep(0.02)
            write_ldo(regulated)
        target.gpio_output(14, True)  # P0.14, LSCTRL
        time.sleep(0.02)

    def flash_test():
        enable_3v3()
        return check_flash(target, spi_bus)

    def sd_test():
        enable_3v3()
        return check_sd(target, spi_bus)

    def led_test():
        enable_3v3()
        led = LEDDriver(target, power_bus)
        currents = bytes(led.read(reg) for reg in (6, 7, 8))
        return f"I²C 0x30, current registers 0x06–0x08: {currents.hex(' ')}"

    checks = (
        ("Charger (BQ25120A)", lambda: check_charger(target, power_bus)),
        ("DAC (ADAU1860)", lambda: check_dac(target, codec_bus)),
        ("IMU (BMX160)", imu.check_data),
        ("Magnetometer (BMM150)", imu.check_magnetometer),
        ("Flash (MX25R6435F)", flash_test),
        ("microSD", sd_test),
        ("LED driver (KTD2026)", led_test),
        ("Main-board microphone", AudioCheck(target, codec_bus).check_microphone),
    )
    try:
        print(f"Connecting to J-Link {serial}…", flush=True)
        device = target.open()
        print(f"nRF5340 {device['device_id']}, VTref {device['target_mv']} mV", flush=True)
        target.gpio_output(17, True)  # P0.17, keep charging disabled during the check
        target.gpio_output(43, True)  # P1.11, switched 1.8 V for codec and IMU
        time.sleep(0.02)
        for label, check in checks:
            check_cancel()
            try:
                target.feed_watchdogs()
                detail = check()
            except Cancelled:
                raise
            except (HardwareError, JLinkException, OSError) as exc:
                print(f"❌ FAIL {label} — {exc}", flush=True)
            else:
                print(f"✅ PASS {label} — {detail}", flush=True)
                passed += 1
    finally:
        target.check_cancel = lambda: None
        try:
            if original_ldo is not None:
                target.gpio_output(44, False)  # Power down SD before restoring the rail.
                target.gpio_output(14, False)
                write_ldo(original_ldo & 0x7F)
                if original_ldo & 0x80:
                    write_ldo(original_ldo)
        except (HardwareError, JLinkException, OSError) as exc:
            print(f"❌ FAIL restore — {exc}", file=sys.stderr)
            restore_failed = True
        finally:
            state = target.cleanup()
        if target.touched:
            if state["errors"]:
                detail = ", ".join(state["errors"])
                print(f"❌ FAIL restore — {detail}", file=sys.stderr)
                restore_failed = True
            elif not restore_failed:
                print("✅ PASS restore — settings restored and target reset", flush=True)

    print(f"{passed}/{len(checks)} checks passed", flush=True)
    return 0 if passed == len(checks) and not restore_failed else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--snr", type=int, help="serial number of the J-Link probe")
    selection.add_argument("--list-probes", action="store_true", help="list J-Link probes and exit")
    args = parser.parse_args()

    try:
        if args.snr is None:
            probes = list_probes()
            if args.list_probes:
                for probe in probes:
                    print(f"{probe['serial']}  {probe['product']}")
                if not probes:
                    print("No J-Link probes found", file=sys.stderr)
                    return 2
                return 0
            if len(probes) != 1:
                parser.error("Specify --snr when zero or multiple J-Link probes are attached (see --list-probes)")
            args.snr = probes[0]["serial"]

        interrupted = False

        def request_stop(_signum, _frame):
            nonlocal interrupted
            interrupted = True

        def check_cancel():
            if interrupted:
                raise Cancelled("Test interrupted")

        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        result = run_tests(args.snr, check_cancel)
        check_cancel()
        return result
    except Cancelled as exc:
        print(exc, file=sys.stderr)
        return 130
    except (HardwareError, JLinkException, OSError) as exc:
        print(f"❌ FAIL J-Link/test setup — {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
