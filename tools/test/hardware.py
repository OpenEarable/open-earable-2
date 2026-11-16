"""SWD peripheral transports for OpenEarable 2.x.

Only RAM and a fixed set of volatile peripheral registers are writable. Internal
flash, UICR, NVMC, debug-unlock and programming APIs are deliberately unavailable.
The target is reset after testing.
"""
from __future__ import annotations

import contextlib
import os
import time
from typing import Callable

import pylink
from pylink.library import Library

TWIM1, TWIM2, TWIM3 = 0x50009000, 0x5000B000, 0x5000C000
SPIM4, I2S, PDM = 0x5000A000, 0x50028000, 0x50026000
CLOCK, DPPI, TIMER = 0x50005000, 0x50017000, 0x5000F000
GPIO = (0x50842500, 0x50842800)
RAM_START, RAM_END = 0x20070000, 0x20080000
TX, RX = RAM_END - 0x2000, RAM_END - 0x1000
FLASH_CS, SD_CS = 33, 11


class HardwareError(RuntimeError):
    """A hardware operation failed or an invariant could not be verified."""


class Cancelled(HardwareError):
    """The operator requested cancellation; cleanup is still required."""


def new_jlink() -> pylink.JLink:
    """Load the installed SEGGER DLL, disabling automatic updates and erase hooks."""
    path = os.environ.get('JLINK_LIBRARY')
    link = pylink.JLink(lib=Library(dllpath=path)) if path else pylink.JLink()
    # Report errors in the CLI without opening dialogs.
    for cmd in ('SuppressGUI', 'DisableAutoUpdateFW', 'DisableFlashDL', 'DisableFlashBPs'):
        link.exec_command(cmd)
    link.unsecure_hook = lambda *args: 0  # Never accept an erase/unlock operation.
    return link


def list_probes() -> list[dict]:
    """Enumerate probes without opening or resetting a target."""
    link = new_jlink()
    return [{'serial': p.SerialNumber,
             'product': bytes(p.acProduct).split(b'\0')[0].decode(errors='replace')}
            for p in link.connected_emulators()]


class Target:
    """Exclusive app-core debug session with bounded, restorable scratch RAM."""

    def __init__(self, serial: int, check_cancel: Callable[[], None]):
        self.serial = serial
        self.check_cancel = check_cancel
        self.link = None
        self.connected = False
        self.touched = False
        self.saved_ram = None
        self.original_gpio = {}
        self.cleanup_errors = []
        self.identity = {}

    def open(self) -> dict:
        """Attach without erasing, validate the SoC and halt before taking control."""
        self.link = new_jlink()
        self.link.open(serial_no=int(self.serial))
        voltage = self.link.hardware_status.VTarget
        if voltage < 1500:
            raise HardwareError(f'Target reference is {voltage} mV; check power/debug contacts')
        self.link.set_tif(pylink.enums.JLinkInterfaces.SWD)
        self.link.connect('NRF5340_XXAA_APP', speed=1000)
        self.connected = True
        # EasyDMA changes target RAM while the CPU is halted.
        self.link.exec_command('SetEnableMemCache = 0')
        part, variant, package, ram, flash = self.link.memory_read32(0x00FF020C, 5)
        if part != 0x5340:
            raise HardwareError(f'Expected nRF5340, found part 0x{part:x}')
        self.link.halt()
        self.touched = True
        self.identity = {'serial': self.serial, 'target_mv': voltage, 'part': hex(part),
                         'variant': hex(variant), 'ram_kb': ram, 'flash_kb': flash,
                         'device_id': ''.join(f'{v:08x}' for v in reversed(self.link.memory_read32(0x00FF0204, 2)))}
        # Network core held in reset: it cannot race shared RAM/peripherals.
        self.w32(CLOCK + 0x614, 1)
        self.quiesce()
        self.saved_ram = self.read(RAM_START, RAM_END - RAM_START)
        self.original_gpio = {b: (self.r32(b + 4), self.link.memory_read32(b + 0x200, pins))
                              for b, pins in zip(GPIO, (32, 16))}
        return self.identity

    def read(self, addr: int, size: int) -> bytes:
        """Read exactly size bytes; short debug reads are errors."""
        result = bytes(self.link.memory_read8(addr, size))
        if len(result) != size:
            raise HardwareError(f'Short SWD read at 0x{addr:x}: {len(result)}/{size}')
        return result

    def r32(self, addr: int) -> int:
        return self.link.memory_read32(addr, 1)[0]

    @staticmethod
    def writable(addr: int, size: int) -> bool:
        """The only writable regions are borrowed RAM and selected MMIO pages."""
        pages = (TWIM1, TWIM2, TWIM3, SPIM4, I2S, PDM, CLOCK, DPPI, TIMER,
                 0x50018000, 0x50019000, 0x50008000, 0x50036000)
        return size > 0 and ((RAM_START <= addr and addr + size <= RAM_END)
                or any(b <= addr and addr + size <= b + 0x1000 for b in pages)
                or any(b <= addr and addr + size <= b + 0x300 for b in GPIO))

    def write(self, addr: int, data: bytes) -> None:
        """Write RAM/MMIO only. Never call a SEGGER flash programming API."""
        if not self.writable(addr, len(data)):
            raise HardwareError(f'Write to protected address 0x{addr:x} refused')
        if self.link.memory_write8(addr, list(data)) != len(data):
            raise HardwareError(f'Short SWD write at 0x{addr:x}')

    def write_words(self, addr: int, values: list[int]) -> None:
        """Write aligned words to the same allowed regions as byte writes."""
        byte_count = 4 * len(values)
        if addr % 4 or not self.writable(addr, byte_count):
            raise HardwareError(f'Word write to address 0x{addr:x} refused')
        # PyLink forwards the SEGGER API's byte count even for word writes.
        if self.link.memory_write32(addr, values) != byte_count:
            raise HardwareError(f'Short SWD word write at 0x{addr:x}')

    def w32(self, addr: int, value: int) -> None:
        self.write_words(addr, [value & 0xFFFFFFFF])

    def wait(self, predicate, timeout=1.0, description='peripheral event') -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.check_cancel()
            if predicate():
                return
            time.sleep(.001)
        raise HardwareError(f'Timeout waiting for {description}')

    def quiesce(self) -> None:
        """Stop autonomous DMA/triggers before borrowing the top 64 KiB of RAM."""
        self.w32(DPPI + 0x508, 0xFFFFFFFF)
        for base, stop, event, enable in [(I2S, 4, 0x108, 1), (PDM, 4, 0x104, 1),
                                          (SPIM4, 0x14, 0x104, 7),
                                          (TWIM1, 0x14, 0x104, 6),
                                          (TWIM2, 0x14, 0x104, 6),
                                          (TWIM3, 0x14, 0x104, 6)]:
            if self.r32(base + 0x500) == enable:
                if base in (TWIM1, TWIM2, TWIM3):
                    self.w32(base + 0x20, 1)  # Resume a suspended transfer before STOP.
                self.w32(base + event, 0)
                self.w32(base + stop, 1)
                self.wait(lambda b=base, e=event: self.r32(b + e), .5, 'DMA STOPPED')
                self.w32(base + 0x500, 0)
            self.w32(base + 0x308, 0xFFFFFFFF)
        # UARTE shares IDs with TWIM1/2/3. Stop an active UART receiver too.
        for base in (0x50008000, TWIM1, TWIM2, TWIM3):
            if self.r32(base + 0x500) == 8:
                self.w32(base + 0x144, 0)
                self.w32(base + 4, 1)
                self.wait(lambda b=base: self.r32(b + 0x144), .5, 'UART RXTO')
                self.w32(base + 0xC, 1)
                self.w32(base + 0x500, 0)
        self.w32(0x50036000 + 0x500, 0)

    def gpio_output(self, pin: int, high: bool) -> None:
        base, number = GPIO[pin // 32], pin % 32
        self.w32(base + (8 if high else 12), 1 << number)
        self.w32(base + 0x200 + 4 * number, 1)

    def gpio_input(self, pin: int, pullup=False, open_drain=False) -> None:
        self.w32(GPIO[pin // 32] + 0x200 + 4 * (pin % 32),
                 (0xC if pullup else 0) | (0x600 if open_drain else 0))

    def gpio_level(self, pin: int) -> int:
        return (self.r32(GPIO[pin // 32] + 0x10) >> (pin % 32)) & 1

    def feed_watchdogs(self) -> None:
        """Feed any running app-core watchdog, without trying to disable it."""
        for base in (0x50018000, 0x50019000):
            if self.r32(base + 0x400):
                enabled = self.r32(base + 0x508)
                for i in range(8):
                    if enabled & (1 << i):
                        self.w32(base + 0x600 + 4 * i, 0x6E524635)

    def cleanup(self) -> dict:
        """Stop audio, restore RAM/GPIO, and reset the target."""
        if not self.connected:
            if self.link:
                self.link.close()
            return {'restored': False, 'reason': 'target never connected'}
        # Cleanup must run even after cancellation.
        self.check_cancel = lambda: None
        checks = {}
        if self.touched:
            dma_stopped = False
            try:
                self.quiesce()
                dma_stopped = True
            except Exception as exc:
                self.cleanup_errors.append(f'stop DMA: {exc}')
            operations = [('stop timer', lambda: self.w32(TIMER + 4, 1))]
            # Never overwrite a buffer while a peripheral might still own it.
            if self.saved_ram is not None and dma_stopped:
                operations.append(('restore scratch RAM', lambda: self.write(RAM_START, self.saved_ram)))
            for base, (out, config) in self.original_gpio.items():
                operations += [(f'restore GPIO {base:x}', lambda b=base, v=out: self.w32(b + 4, v)),
                               (f'restore pin configuration {base:x}', lambda b=base, v=config: self.write_words(b + 0x200, v))]
            for name, operation in operations:
                try:
                    operation()
                except Exception as exc:
                    self.cleanup_errors.append(f'{name}: {exc}')
            try:
                self.w32(CLOCK + 0x614, 0)
                self.link.reset(ms=20, halt=False)
                time.sleep(.15)
                checks['stock_firmware_running'] = not self.link.halted()
            except Exception as exc:
                self.cleanup_errors.append(f'restart stock firmware: {exc}')
        self.link.close()
        checks['errors'] = self.cleanup_errors
        return checks


class I2CBus:
    """One nRF TWIM bus, driven by SWD tasks and EasyDMA with CPU halted."""
    def __init__(self, target: Target, base: int, sda: int, scl: int):
        self.t, self.base, self.sda, self.scl = target, base, sda, scl

    def setup(self) -> None:
        t, b = self.t, self.base
        t.w32(b + 0x500, 0)
        for pin in (self.sda, self.scl):
            t.gpio_input(pin, pullup=True, open_drain=True)
        for offset, value in {0x508: self.scl, 0x50C: self.sda, 0x524: 0x06400000,
                              0x200: 0, 0x308: 0xFFFFFFFF, 0x500: 6}.items():
            t.w32(b + offset, value)

    def transfer(self, address: int, data=b'', count=0) -> bytes:
        if not 0 <= address < 128 or not 0 <= count <= 2048 or len(data) > 2048:
            raise ValueError('Invalid I2C transfer')
        if not data and not count:
            raise ValueError('Empty I2C transfer')
        t, b = self.t, self.base
        self.setup()
        if data:
            t.write(TX, bytes(data))
        for event in (0x104, 0x124, 0x15C, 0x160):
            t.w32(b + event, 0)
        t.w32(b + 0x4C4, 7)
        for offset, value in {0x588: address, 0x544: TX, 0x548: len(data), 0x550: 0,
                              0x534: RX, 0x538: count, 0x540: 0,
                              0x200: (0x1080 if count else 0x200) if data else 0x1000}.items():
            t.w32(b + offset, value)
        try:
            t.w32(b + (8 if data else 0), 1)
            t.wait(lambda: t.r32(b + 0x104) or t.r32(b + 0x124), .25, f'I2C 0x{address:02x}')
            error = t.r32(b + 0x4C4)
            if error:
                raise HardwareError(f'I2C 0x{address:02x}: error 0x{error:x} '
                                    f'(SCL={t.gpio_level(self.scl)}, SDA={t.gpio_level(self.sda)})')
            if data and t.r32(b + 0x54C) != len(data):
                raise HardwareError('Incomplete I2C write')
            if count and t.r32(b + 0x53C) != count:
                raise HardwareError('Incomplete I2C read')
            return t.read(RX, count) if count else b''
        finally:
            t.w32(b + 0x14, 1)
            t.w32(b + 0x200, 0)
            t.w32(b + 0x500, 0)

    def read(self, address, reg, count=1, width=1) -> bytes:
        return self.transfer(address, int(reg).to_bytes(width, 'big'), count)

    def write(self, address, reg, data, width=1) -> None:
        if isinstance(data, int):
            data = bytes([data])
        self.transfer(address, int(reg).to_bytes(width, 'big') + bytes(data))


class SPIBus:
    """SPI4 shared by NOR and SD; chip selects are always mutually exclusive."""
    def __init__(self, target: Target, frequency=0x10000000):
        self.t, self.frequency = target, frequency
        self.selected = None

    def setup(self) -> None:
        t, b = self.t, SPIM4
        t.w32(b + 0x500, 0)
        for pin in (FLASH_CS, SD_CS):
            t.gpio_output(pin, True)
        t.gpio_output(8, False)
        t.gpio_output(9, True)
        t.gpio_input(10, pullup=True)
        for offset, value in {0x508: 8, 0x50C: 9, 0x510: 10, 0x514: 0xFFFFFFFF,
                              0x524: self.frequency, 0x554: 0, 0x560: 0,
                              0x200: 0, 0x5C0: 0xFF, 0x308: 0xFFFFFFFF, 0x500: 7}.items():
            t.w32(b + offset, value)

    @contextlib.contextmanager
    def select(self, pin: int):
        if self.selected is not None or pin not in (FLASH_CS, SD_CS):
            raise HardwareError('Invalid or overlapping SPI chip select')
        self.selected = pin
        self.t.gpio_output(FLASH_CS, True)
        self.t.gpio_output(SD_CS, True)
        self.t.gpio_output(pin, False)
        try:
            yield
        finally:
            self.t.gpio_output(pin, True)
            self.selected = None

    def transfer(self, data: bytes) -> bytes:
        if not 0 < len(data) <= 4096:
            raise ValueError('SPI transfer must contain 1..4096 bytes')
        t, b = self.t, SPIM4
        t.write(TX, bytes(data))
        t.w32(b + 0x118, 0)
        for offset, value in {0x544: TX, 0x548: len(data), 0x550: 0,
                              0x534: RX, 0x538: len(data), 0x540: 0}.items():
            t.w32(b + offset, value)
        t.w32(b + 0x10, 1)
        t.wait(lambda: t.r32(b + 0x118), 1, 'SPI END')
        if t.r32(b + 0x53C) != len(data) or t.r32(b + 0x54C) != len(data):
            raise HardwareError('Incomplete SPI DMA transfer')
        return t.read(RX, len(data))

    def command(self, pin: int, command: bytes, count=0) -> bytes:
        with self.select(pin):
            response = self.transfer(command + b'\xff' * count)
        return response[len(command):]
