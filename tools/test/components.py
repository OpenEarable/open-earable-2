"""Read the BMX160 identity and the KTD2026 LED current registers."""

from hardware import HardwareError


def require(condition, message):
    if not condition:
        raise HardwareError(message)


class IMU:
    """BMX160 accel/gyro and its integrated BMM150 auxiliary magnetometer."""

    def __init__(self, bus):
        self.bus = bus

    def read(self, reg, count=1):
        return self.bus.read(0x68, reg, count)

    def identify(self):
        identity = self.read(0)[0]
        require(identity == 0xD8, f'Expected BMX160 ID D8, got {identity:02X}')
        return {'chip_id': f'{identity:02X}', 'errors': hex(self.read(2)[0])}


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
