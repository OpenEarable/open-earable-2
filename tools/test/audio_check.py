"""Main-board microphone capture over I²S EasyDMA."""

import time

import numpy as np
from pylink.errors import JLinkException

from components import require
from hardware import CLOCK, DPPI, I2S, TIMER, HardwareError

FS, FRAMES = 48000, 3840
AUDIO_RX = 0x20074000


class AudioCheck:
    """Configure the existing codec's volatile audio path and capture its main mic."""

    def __init__(self, target, bus):
        self.t, self.bus = target, bus
        self.saved = {}

    def read(self, reg, count=1):
        return self.bus.read(0x64, reg, count, width=4)

    def write(self, reg, value):
        if reg not in self.saved:
            self.saved[reg] = self.read(reg)[0]
        self.bus.write(0x64, reg, value, width=4)

    def configure_codec(self):
        t = self.t
        t.gpio_output(4, True)
        time.sleep(.04)
        self.write(0x4000C051, 0x40)  # Mute before starting the codec clocks.
        self.write(0x4000C004, 0)  # Leave the DAC and headphone output powered down.
        self.write(0x4000C071, 1)  # The startup delay elapsed after bringing PD high.
        # Hibernate 1 powers the audio domain. Enable its blocks before polling status.
        self.write(0x4000C00E, 0x15)
        # Run from the 24.576 MHz crystal until the 49.152 MHz multiplier is ready.
        self.write(0x4000C01C, 0x90)
        self.write(0x4000C005, (self.read(0x4000C005)[0] | 0x02) & ~0x01)
        self.write(0x4000C01B, 1)
        t.wait(lambda: self.read(0x4000C402)[0] & 0x02, 1, 'codec frequency multiplier')
        self.write(0x4000C01C, 0x11)
        configuration = [
            (0xC0E0, 0x10), (0xC0E1, 0),
            (0xC007, 0x13), (0xC009, 0x31), (0xC006, 1),
            (0xC0A0, 4), (0xC0A1, 0x10), (0xC0A3, 2),
            (0xC00B, 3), (0xC080, 0x24), (0xC084, 39), (0xC085, 40),
            (0xC0A4, 39), (0xC0A5, 40),  # ASRCO0/1 <- FDEC0/1
            (0xC0E3, 32), (0xC0E4, 33),
            (0xC045, 0x40), (0xC046, 0x40), (0xC044, 0),
            (0xC040, 0x33), (0xC041, 4),
        ]
        for reg, val in configuration:
            self.write(0x40000000 + reg, val)

    def configure_i2s(self):
        t = self.t
        t.w32(I2S + 0x500, 0)
        # nRF5340 audio PLL frequency formula: 12.288 MHz uses FREQ=0x9BA6 (audio_clock.h).
        t.w32(CLOCK + 0x55C, 0x9BA6)
        t.w32(CLOCK + 0x120, 0)
        t.w32(CLOCK + 0x18, 1)
        t.wait(lambda: t.r32(CLOCK + 0x454) & 0x10000, 1, 'audio PLL clock')
        for pin in (42, 30):
            t.gpio_output(pin, False)
        t.gpio_input(31)
        for offset, value in {
            0x504: 0, 0x508: 1, 0x50C: 0, 0x510: 1, 0x514: 0x80000000,
            0x518: 4, 0x51C: 1, 0x520: 0, 0x524: 0, 0x528: 0, 0x52C: 1,
            0x560: 0xFFFFFFFF, 0x564: 42, 0x568: 30, 0x56C: 31,
            0x570: 0xFFFFFFFF,
            0x538: AUDIO_RX, 0x550: FRAMES,
            0x308: 0xFFFFFFFF, 0x104: 0, 0x108: 0, 0x114: 0,
        }.items():
            t.w32(I2S + offset, value)
        # Independent two-second stop, even if the host process disappears.
        for offset, value in {4: 1, 0xC: 1, 0x504: 0, 0x508: 3, 0x510: 4,
                              0x540: 2_000_000, 0x140: 0, 0x200: 1 << 8,
                              0x1C0: 0x8000001F}.items():
            t.w32(TIMER + offset, value)
        t.w32(I2S + 0x84, 0x8000001F)
        t.w32(DPPI + 0x504, 1 << 31)
        t.w32(I2S + 0x500, 1)

    def capture(self):
        """Capture PCM from the codec."""
        t = self.t
        t.write(AUDIO_RX, bytes(FRAMES * 4))
        t.w32(I2S + 0x104, 0)
        t.w32(I2S + 0x108, 0)
        t.w32(TIMER + 0xC, 1)
        t.w32(I2S, 1)
        t.w32(TIMER, 1)
        try:
            deadline = time.monotonic() + .5
            while time.monotonic() < deadline:
                t.check_cancel()
                time.sleep(.02)
            require(t.r32(I2S + 0x104), 'I2S microphone DMA never started')
        finally:
            original_cancel, t.check_cancel = t.check_cancel, lambda: None
            try:
                t.w32(I2S + 4, 1)
                t.wait(lambda: t.r32(I2S + 0x108), .3, 'I2S stopped')
                t.w32(TIMER + 4, 1)
            finally:
                t.check_cancel = original_cancel
        return np.frombuffer(t.read(AUDIO_RX, FRAMES * 4), dtype='<i2').reshape(-1, 2).copy()

    def check_microphone(self):
        """Verify that the main microphone produces varying PCM samples."""
        try:
            self.configure_codec()
            self.configure_i2s()
            samples = self.capture()[:, 0]  # DMIC0 is the main-board microphone.
            # Ignore the buffer edges, which may contain startup or stop transients.
            samples = samples[FRAMES // 4:3 * FRAMES // 4]
            low, high = int(samples.min()), int(samples.max())
            require(low != high, f'Microphone flatline at {low}')
            return f'{len(samples)} PCM samples at {FS} Hz, range {low} to {high}'
        finally:
            original_cancel, self.t.check_cancel = self.t.check_cancel, lambda: None
            try:
                self.restore()
            except (HardwareError, JLinkException, OSError) as exc:
                self.t.cleanup_errors.append(f'restore codec: {exc}')
                raise
            finally:
                self.t.check_cancel = original_cancel

    def restore(self):
        """Restore codec registers, keeping playback muted during the restore."""
        mute_register = 0x4000C051
        if self.saved:
            self.bus.write(0x64, mute_register, 0x40, width=4)
        for reg, value in reversed(list(self.saved.items())):
            if reg != mute_register:
                self.bus.write(0x64, reg, value, width=4)
        if mute_register in self.saved:
            self.bus.write(0x64, mute_register, self.saved[mute_register], width=4)
