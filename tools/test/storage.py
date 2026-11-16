"""Read-only SPI checks for the microSD card and external NOR flash."""

import binascii
import struct
import time

from hardware import FLASH_CS, SD_CS, HardwareError


def crc7(data: bytes) -> int:
    """SD command CRC, including its mandatory end bit."""
    crc = 0
    for byte in data:
        for bit in range(8):
            crc <<= 1
            if ((byte << bit) ^ crc) & 0x80:
                crc ^= 0x09
    return ((crc & 0x7F) << 1) | 1


class SDCard:
    """Bounded SDHC/SDSC SPI block device, with data CRC verification."""

    def __init__(self, bus):
        self.bus = bus
        self.high_capacity = False
        self.sectors = 0
        self.identity = {}

    def command(self, command, argument=0, trailing=0):
        raw = bytes([0x40 | command]) + struct.pack('>I', argument)
        # Clock one idle byte after asserting CS so the board's SD translator
        # is active before the command starts.
        self.bus.transfer(b'\xff' + raw + bytes([crc7(raw)]))
        for _ in range(16):
            response = self.bus.transfer(b'\xff')[0]
            if response & 0x80 == 0:
                return response, self.bus.transfer(b'\xff' * trailing) if trailing else b''
        raise HardwareError(f'SD CMD{command}: no response')

    def simple(self, command, argument=0, trailing=0):
        with self.bus.select(SD_CS):
            result = self.command(command, argument, trailing)
            self.bus.transfer(b'\xff')
        self.bus.transfer(b'\xff')
        return result

    def data(self, length):
        self.bus.t.wait(lambda: self._data_token(), 1.5, 'SD data token')
        block = self.bus.transfer(b'\xff' * (length + 2))
        expected = int.from_bytes(block[-2:], 'big')
        actual = binascii.crc_hqx(block[:-2], 0)
        if actual != expected:
            raise HardwareError(f'SD CRC mismatch: received {expected:04x}, calculated {actual:04x}')
        return block[:-2]

    def _data_token(self):
        token = self.bus.transfer(b'\xff')[0]
        if token == 0xFE:
            return True
        if token != 0xFF:
            raise HardwareError(f'SD rejected read: data token 0x{token:02x}')
        return False

    def initialize(self):
        self.bus.frequency = 0x04000000  # 250 kHz, <=400 kHz initialization limit.
        self.bus.setup()
        self.bus.transfer(b'\xff' * 10)
        # The board gates the SD translator with CS. Supply idle clocks with it enabled, too.
        with self.bus.select(SD_CS):
            self.bus.transfer(b'\xff' * 10)
        if self.simple(0)[0] != 1:
            raise HardwareError('SD card did not enter SPI idle state')
        r1, r7 = self.simple(8, 0x1AA, 4)
        v2 = r1 == 1
        if v2 and r7[-2:] != b'\x01\xaa':
            raise HardwareError(f'SD CMD8 voltage/echo mismatch: {r7.hex()}')
        if r1 not in (1, 5):
            raise HardwareError(f'SD CMD8 returned 0x{r1:02x}')
        deadline = time.monotonic() + 5
        while True:
            self.bus.t.check_cancel()
            self.simple(55)
            r1, _ = self.simple(41, 0x40000000 if v2 else 0)
            if r1 == 0:
                break
            if r1 != 1 or time.monotonic() >= deadline:
                raise HardwareError(f'SD ACMD41 initialization failed: 0x{r1:02x}')
            time.sleep(.02)
        r1, ocr = self.simple(58, trailing=4)
        if r1 != 0 or not int.from_bytes(ocr, 'big') & 0x80000000:
            raise HardwareError('SD OCR not ready')
        self.high_capacity = bool(ocr[0] & 0x40)
        if not self.high_capacity and self.simple(16, 512)[0]:
            raise HardwareError('SD card rejected 512-byte block size')
        if self.simple(59, 1)[0]:
            raise HardwareError('SD card rejected CRC enable')
        registers = {}
        for cmd, name in [(9, 'csd'), (10, 'cid')]:
            with self.bus.select(SD_CS):
                if self.command(cmd)[0]:
                    raise HardwareError(f'SD CMD{cmd} failed')
                registers[name] = self.data(16)
        csd = registers['csd']
        if csd[0] >> 6 == 1:
            self.sectors = ((((csd[7] & 0x3F) << 16) | (csd[8] << 8) | csd[9]) + 1) * 1024
        elif csd[0] >> 6 == 0:
            size = ((csd[6] & 3) << 10) | (csd[7] << 2) | (csd[8] >> 6)
            mult = ((csd[9] & 3) << 1) | (csd[10] >> 7)
            self.sectors = (size + 1) * (1 << (mult + 2)) * (1 << (csd[5] & 15)) // 512
        else:
            raise HardwareError('Unsupported SD CSD structure')
        self.bus.frequency = 0x10000000  # 1 MHz for block reads.
        self.bus.setup()
        self.identity = {'cid': registers['cid'].hex(), 'csd': csd.hex(),
                         'sectors': self.sectors, 'capacity_bytes': self.sectors * 512,
                         'high_capacity': self.high_capacity}
        return self.identity

    def _address(self, sector):
        if not 0 <= sector < self.sectors:
            raise HardwareError(f'SD sector {sector} outside card capacity')
        return sector if self.high_capacity else sector * 512

    def read_sector(self, sector):
        with self.bus.select(SD_CS):
            if self.command(17, self._address(sector))[0]:
                raise HardwareError(f'SD rejected read sector {sector}')
            return self.data(512)


class NORFlash:
    """Read the Macronix NOR identity and data over SPI."""
    SIZE = 8 * 1024 * 1024

    def __init__(self, bus):
        self.bus = bus

    def command(self, data, count=0):
        return self.bus.command(FLASH_CS, bytes(data), count)

    def identify(self):
        self.command(b'\xab')
        time.sleep(.001)
        identity = self.command(b'\x9f', 3)
        if identity != bytes.fromhex('c22817'):
            raise HardwareError(f'Expected MX25R6435F C22817, read {identity.hex().upper()}')
        return {'jedec_id': identity.hex().upper(), 'capacity_bytes': self.SIZE,
                'status': self.command(b'\x05', 1)[0]}

    def read(self, addr, count):
        if not 0 <= addr <= self.SIZE - count:
            raise HardwareError('NOR read outside device')
        out = bytearray()
        for offset in range(0, count, 1024):
            out.extend(self.command(b'\x03' + (addr + offset).to_bytes(3, 'big'), min(1024, count - offset)))
        return bytes(out)
