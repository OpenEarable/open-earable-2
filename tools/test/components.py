"""Read BMX160 motion samples and KTD2026 LED current registers."""

import struct
import time
from contextlib import contextmanager

from pylink.errors import JLinkException

from hardware import HardwareError


def require(condition, message):
    if not condition:
        raise HardwareError(message)


class IMU:
    """Check BMX160 motion sensors and its BMM150 magnetometer over I²C."""

    SAMPLE_COUNT = 8

    def __init__(self, bus):
        self.bus = bus

    def read(self, reg, count=1):
        return self.bus.read(0x68, reg, count)

    def identify(self):
        identity = self.read(0)[0]
        require(identity == 0xD8, f'Expected BMX160 ID D8, got {identity:02X}')
        return {'chip_id': f'{identity:02X}'}

    def _set_power_mode(self, command_base, shift, mode):
        target = self.bus.t
        target.check_cancel()
        # Let the command finish before cancellation can start restoring settings.
        previous_cancel, target.check_cancel = target.check_cancel, target.feed_watchdogs
        try:
            self.bus.write(0x68, 0x7E, command_base | mode)
            target.wait(lambda: ((self.read(0x03)[0] >> shift) & 3) == mode,
                        0.25, 'BMX160 power mode')
        finally:
            target.check_cancel = previous_cancel

    @contextmanager
    def _restore_settings(self):
        target = self.bus.t
        restore = []
        try:
            yield restore.append
        finally:
            previous_cancel, target.check_cancel = target.check_cancel, lambda: None
            try:
                for restore_setting in reversed(restore):
                    try:
                        target.feed_watchdogs()
                        restore_setting()
                    except (HardwareError, JLinkException, OSError) as exc:
                        target.cleanup_errors.append(f'restore BMX160: {exc}')
            finally:
                target.check_cancel = previous_cancel

    @staticmethod
    def _check_samples(label, samples):
        require(any(any(sample) for sample in samples), f'{label} returned only zeros')
        require(len(set(samples)) > 1, f'{label} repeated the same XYZ value {samples[0]}')

    def check_data(self):
        """Read fresh XYZ samples and restore the original power and filter settings."""
        identity = self.identify()['chip_id']
        target = self.bus.t
        power = self.read(0x03)[0]
        accel_mode, gyro_mode = (power >> 4) & 3, (power >> 2) & 3
        require(accel_mode != 3 and gyro_mode != 2, f'Invalid BMX160 PMU status {power:#04x}')
        accel_config = self.read(0x40)[0]

        def data_ready():
            target.feed_watchdogs()
            return (self.read(0x1B)[0] & 0xC0) == 0xC0

        def write_accel_config(value):
            self.bus.write(0x68, 0x40, value)
            time.sleep(0.002)
            require(self.read(0x40)[0] == value, 'BMX160 accelerometer configuration did not latch')

        with self._restore_settings() as restore:
            if accel_mode != 1:
                restore(lambda: self._set_power_mode(0x10, 4, accel_mode))
            if accel_config & 0x80:
                # Undersampling is required in low power mode and disabled in normal mode.
                restore(lambda: write_accel_config(accel_config))
                write_accel_config(accel_config & 0x7F)
            if accel_mode != 1:
                self._set_power_mode(0x10, 4, 1)
            if gyro_mode != 1:
                restore(lambda: self._set_power_mode(0x14, 2, gyro_mode))
                self._set_power_mode(0x14, 2, 1)

            accel, gyro, timestamps = [], [], set()
            for _ in range(self.SAMPLE_COUNT):
                target.wait(data_ready, 2, 'BMX160 accelerometer and gyroscope data ready')
                # One burst reads gyro XYZ, accel XYZ, and the 24-bit sensor time.
                data = self.read(0x0C, 15)
                axes = struct.unpack('<6h', data[:12])
                gyro.append(axes[:3])
                accel.append(axes[3:])
                timestamps.add(int.from_bytes(data[12:], 'little'))
            require(len(timestamps) == len(accel), 'BMX160 sensor time did not advance between samples')
            self._check_samples('BMX160 accelerometer', accel)
            self._check_samples('BMX160 gyroscope', gyro)
            return f'ID 0x{identity}, {len(accel)} fresh samples, accel {accel[-1]}, gyro {gyro[-1]} (raw XYZ)'

    def _wait_aux(self):
        self.bus.t.wait(lambda: not (self.read(0x1B)[0] & 0x04), 0.1, 'BMX160 auxiliary transfer')

    def _aux_read(self, register):
        self.bus.write(0x68, 0x4D, register)
        self._wait_aux()
        return self.read(0x04, 8)

    def _aux_write(self, register, value):
        self.bus.write(0x68, 0x4F, value)
        self.bus.write(0x68, 0x4E, register)
        self._wait_aux()

    def check_magnetometer(self):
        """Trigger BMM150 conversions through the BMX160 auxiliary interface."""
        self.identify()
        target = self.bus.t
        aux_mode = self.read(0x03)[0] & 3
        require(aux_mode != 3, f'Invalid BMX160 auxiliary power mode {aux_mode}')
        aux_config = self.read(0x4B, 5)
        interface = self.read(0x6B)[0]
        samples = []

        def restore_aux():
            # Restoring the address registers while suspended prevents replaying a write.
            self._set_power_mode(0x18, 0, 0)
            for register, value in enumerate(aux_config, start=0x4B):
                self.bus.write(0x68, register, value)
                time.sleep(0.001)
            self.bus.write(0x68, 0x6B, interface)
            require(self.read(0x4B, 5) == aux_config and self.read(0x6B)[0] == interface,
                    'BMX160 auxiliary configuration did not restore')
            if aux_mode:
                self._set_power_mode(0x18, 0, aux_mode)

        def sample_ready():
            target.feed_watchdogs()
            data = self._aux_read(0x42)
            if not (data[6] & 1):
                return False
            x, y, z, hall = struct.unpack('<hhhH', data)
            axes = (x >> 3, y >> 3, z >> 1)
            require(axes[0] != -4096 and axes[1] != -4096 and axes[2] != -16384,
                    'BMM150 reported magnetic overflow')
            require(hall >> 2, 'BMM150 returned zero Hall resistance')
            samples.append(axes)
            return True

        with self._restore_settings() as restore:
            restore(restore_aux)
            if aux_mode != 1:
                self._set_power_mode(0x18, 0, 1)
            self.bus.write(0x68, 0x6B, interface | 0x20)
            self.bus.write(0x68, 0x4B, 0x20)  # BMM150 I²C address 0x10, shifted.
            self.bus.write(0x68, 0x4C, 0x83)  # Manual mode with eight-byte reads.
            power = self._aux_read(0x4B)[0] & 0x05
            if not (power & 1):
                restore(lambda: self._aux_write(0x4B, power))
                self._aux_write(0x4B, power | 1)
                time.sleep(0.005)
            identity = self._aux_read(0x40)[0]
            require(identity == 0x32, f'Expected BMM150 ID 32, got {identity:02X}')
            operation = self._aux_read(0x4C)[0]
            if power & 1:
                restore(lambda: self._aux_write(0x4C, operation))
            self._aux_write(0x4C, (operation & 0x38) | 0x06)  # Sleep, with the original data rate.
            self._aux_read(0x42)  # Clear any pending sample before triggering a conversion.
            for _ in range(self.SAMPLE_COUNT):
                self._aux_write(0x4C, (operation & 0x38) | 0x02)  # Single conversion.
                target.wait(sample_ready, 0.3, 'BMM150 data ready')
            self._check_samples('BMM150 magnetometer', samples)
            return f'ID 0x{identity:02X}, {len(samples)} fresh samples, field {samples[-1]} (raw XYZ)'


class LEDDriver:
    """KTD2026 hardware read-back using its non-standard three-byte read cycle."""
    SDA, SCL = 21, 24

    def __init__(self, target, bus):
        self.t, self.bus = target, bus

    def line(self, pin, high):
        self.t.w32(0x50842500 + (8 if high else 12), 1 << pin)
        if pin == self.SCL and high:
            self.t.wait(lambda: self.t.gpio_level(pin), .1, 'LED clock high')

    def send(self, value):
        for bit in range(7, -1, -1):
            self.line(self.SCL, False)
            self.line(self.SDA, bool(value & (1 << bit)))
            self.line(self.SCL, True)
        self.line(self.SCL, False)
        self.line(self.SDA, True)
        self.line(self.SCL, True)
        acknowledged = not self.t.gpio_level(self.SDA)
        self.line(self.SCL, False)
        require(acknowledged, 'KTD2026 did not acknowledge special read')

    def read(self, reg):
        self.t.w32(self.bus.base + 0x500, 0)
        for pin in (self.SDA, self.SCL):
            self.line(pin, True)
            self.t.w32(0x50842700 + pin * 4, 0x60D)  # Open-drain output, input connected, pull-up.
        require(self.t.gpio_level(self.SDA), 'KTD2026 bus SDA stuck low before START')
        self.line(self.SDA, False)
        try:
            self.send(0x61)  # Address with READ bit, then master still transmits register.
            self.send(reg)
            self.line(self.SDA, True)
            value = 0
            for _ in range(8):
                self.line(self.SCL, True)
                value = (value << 1) | self.t.gpio_level(self.SDA)
                self.line(self.SCL, False)
            self.line(self.SDA, True)  # Final master NACK
            self.line(self.SCL, True)
            self.line(self.SCL, False)
            return value
        finally:
            self.line(self.SDA, False)
            self.line(self.SCL, True)
            self.line(self.SDA, True)
