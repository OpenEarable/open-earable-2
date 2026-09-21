#!/usr/bin/env python3
"""Exercise LC3 startup with the linked firmware's real Newlib heap.

Runs the application's ARM binary in isolation, including the LC3 allocator,
malloc and _sbrk. Only malloc's thread locks are bypassed: there is no scheduler
and each test runs on one host thread. This tests memory availability, not
Bluetooth timing or microphone hardware.
"""

import argparse
import itertools
import struct
import unittest
from pathlib import Path

from elftools.elf.elffile import ELFFile
from unicorn import UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_MCLASS, UC_MODE_THUMB, Uc
from unicorn.arm_const import (
    UC_ARM_REG_C1_C0_2,
    UC_ARM_REG_LR,
    UC_ARM_REG_PC,
    UC_ARM_REG_R0,
    UC_ARM_REG_R1,
    UC_ARM_REG_R2,
    UC_ARM_REG_R3,
    UC_ARM_REG_SP,
)


SAMPLE_RATE_PAIRS = tuple(itertools.product((16000, 24000, 48000), repeat=2))
MIN_HEAP_BYTES = 48 * 1024
# An explicit budget for other libc users, not an estimate of all runtime use.
RUNTIME_RESERVE_BYTES = 8 * 1024
RESTART_PASSES = 3
LC3_INSUFFICIENT_RESOURCES = -5002


class CodecError(AssertionError):
    def __init__(self, function, result):
        super().__init__(f"{function} returned {result}")
        self.result = result


class Firmware:
    """One fresh, unbooted application image with an isolated test stack."""

    RETURN_ADDRESS = 0x001FF000
    STACK = 0x2100F000
    RESULT = 0x2100E000

    def __init__(self, path):
        self.config = dict(
            line.split("=", 1)
            for line in path.with_name(".config").read_text().splitlines()
            if line.startswith("CONFIG_") and "=" in line
        )
        if self.config.get("CONFIG_NEWLIB_LIBC") != "y":
            raise AssertionError("This test requires the application's Newlib configuration")
        if self.config.get("CONFIG_AUDIO_DEV") != "1":
            raise AssertionError("This test covers the headset's encoder/decoder configuration")
        self.ram_end = (int(self.config["CONFIG_SRAM_BASE_ADDRESS"], 0)
                        + 1024 * int(self.config["CONFIG_SRAM_SIZE"]))
        self.cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.cpu.mem_map(0, 0x200000)
        self.cpu.mem_map(0x20000000, 0x80000)
        self.cpu.mem_map(0x21000000, 0x10000)
        with path.open("rb") as stream:
            elf = ELFFile(stream)
            self.symbols = {
                symbol.name: symbol["st_value"]
                for symbol in elf.get_section_by_name(".symtab").iter_symbols()
            }
            for segment in elf.iter_segments():
                if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
                    self.cpu.mem_write(segment["p_vaddr"], segment.data())
        self.lock_functions = {
            self.symbols[name] & ~1 for name in ("__malloc_lock", "__malloc_unlock")
        }
        self.cpu.hook_add(UC_HOOK_CODE, self._intercept)
        self.cpu.reg_write(UC_ARM_REG_C1_C0_2, 0xF << 20)

    def _intercept(self, cpu, address, _size, _data):
        if address == self.RETURN_ADDRESS:
            cpu.emu_stop()
        elif address in self.lock_functions:
            cpu.reg_write(UC_ARM_REG_PC, cpu.reg_read(UC_ARM_REG_LR))

    def invoke(self, name, *args):
        """Call an integer/pointer ARM ABI function and return its raw R0 value."""
        self.cpu.reg_write(UC_ARM_REG_SP, self.STACK)
        self.cpu.reg_write(UC_ARM_REG_LR, self.RETURN_ADDRESS | 1)
        registers = (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3)
        for register, argument in zip(registers, args):
            self.cpu.reg_write(register, argument)
        for index, argument in enumerate(args[4:]):
            self.cpu.mem_write(self.STACK + index * 4, struct.pack("<I", argument))
        self.cpu.emu_start(self.symbols[name] | 1, self.RETURN_ADDRESS, count=5_000_000)
        if self.cpu.reg_read(UC_ARM_REG_PC) != self.RETURN_ADDRESS:
            raise AssertionError(f"{name} did not return within the instruction limit")
        return self.cpu.reg_read(UC_ARM_REG_R0)

    def call(self, name, *args):
        """Call a codec function that reports zero on success."""
        result = self.invoke(name, *args)
        if result >= 2**31:
            result -= 2**32
        if result != 0:
            raise CodecError(name, result)

    def allocate(self, size):
        """Hold a real Newlib allocation alongside the codec's allocations."""
        pointer = self.invoke("malloc", size)
        if not pointer:
            raise AssertionError(f"malloc({size}) failed")
        return pointer

    def free(self, pointer):
        self.invoke("free", pointer)

    def initialize_codec(self):
        self.call("sw_codec_lc3_init", 0, 0, int(self.config["CONFIG_AUDIO_FRAME_DURATION_US"]))

    def start_codec(self, encoder_rate, decoder_rate):
        frame_duration = int(self.config["CONFIG_AUDIO_FRAME_DURATION_US"])
        bit_depth = int(self.config["CONFIG_AUDIO_BIT_DEPTH_BITS"])
        bitrate = int(self.config["CONFIG_LC3_BITRATE"])
        decoder_channels = 2 if self.config.get("CONFIG_SD_CARD_PLAYBACK") == "y" else 1
        self.call("sw_codec_lc3_enc_init", encoder_rate, bit_depth, frame_duration,
                  bitrate, 1, self.RESULT)
        self.call("sw_codec_lc3_dec_init", decoder_rate, bit_depth, frame_duration,
                  decoder_channels)

    def stop_codec(self):
        # The application retains shared LC3 tables across recording sessions.
        self.call("sw_codec_lc3_enc_uninit_all")
        self.call("sw_codec_lc3_dec_uninit_all")

    def available_heap(self):
        return self.ram_end - self.symbols["_end"]

    def heap_used(self):
        """Bytes obtained by Newlib via sbrk, including reusable freed blocks."""
        return struct.unpack("<I", self.cpu.mem_read(self.symbols["heap_sz"], 4))[0]


class CodecMemoryTests(unittest.TestCase):
    elf_path = None

    def test_linked_image_preserves_configured_heap_budget(self):
        firmware = Firmware(self.elf_path)
        configured = int(firmware.config["CONFIG_NEWLIB_LIBC_MIN_REQUIRED_HEAP_SIZE"])
        self.assertGreaterEqual(configured, MIN_HEAP_BYTES)
        self.assertGreaterEqual(firmware.available_heap(), configured)

    def test_all_sample_rates_leave_runtime_reserve(self):
        for rates in SAMPLE_RATE_PAIRS:
            with self.subTest(encoder_rate=rates[0], decoder_rate=rates[1]):
                firmware = Firmware(self.elf_path)
                firmware.initialize_codec()
                firmware.start_codec(*rates)
                # Probe the actual allocator, including fragmentation and headers,
                # rather than counting only the uncommitted space above sbrk.
                reserve = firmware.allocate(RUNTIME_RESERVE_BYTES)
                firmware.free(reserve)
                firmware.stop_codec()

    def test_all_sample_rates_start_with_other_memory_in_use(self):
        for rates in SAMPLE_RATE_PAIRS:
            with self.subTest(encoder_rate=rates[0], decoder_rate=rates[1]):
                firmware = Firmware(self.elf_path)
                reserve = firmware.allocate(RUNTIME_RESERVE_BYTES)
                firmware.initialize_codec()
                firmware.start_codec(*rates)
                firmware.stop_codec()
                firmware.free(reserve)

    def test_restarts_and_rate_changes_reuse_heap(self):
        firmware = Firmware(self.elf_path)
        reserve = firmware.allocate(RUNTIME_RESERVE_BYTES)
        firmware.initialize_codec()
        warmed_heap = None
        for cycle in range(RESTART_PASSES):
            for rates in SAMPLE_RATE_PAIRS:
                with self.subTest(cycle=cycle, encoder_rate=rates[0], decoder_rate=rates[1]):
                    firmware.start_codec(*rates)
                    firmware.stop_codec()
            # A repeated identical workload must reuse the first pass's arena.
            # sbrk itself need not shrink when the codec frees its sessions.
            if warmed_heap is None:
                warmed_heap = firmware.heap_used()
            self.assertEqual(firmware.heap_used(), warmed_heap,
                             "Recording restarts keep growing the Newlib arena")
        firmware.free(reserve)

    def test_insufficient_memory_is_detected(self):
        firmware = Firmware(self.elf_path)
        # Deliberately leave less than 48 kHz duplex startup requires. This
        # negative control proves the test cannot silently use unlimited RAM.
        firmware.allocate(firmware.available_heap() - 36 * 1024)
        with self.assertRaises(CodecError) as error:
            firmware.initialize_codec()
            firmware.start_codec(48000, 48000)
        self.assertEqual(error.exception.result, LC3_INSUFFICIENT_RESOURCES)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path, help="Application zephyr.elf, not ipc_radio")
    args = parser.parse_args()
    CodecMemoryTests.elf_path = args.elf
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(CodecMemoryTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(0 if result.wasSuccessful() else 1)


if __name__ == "__main__":
    main()
