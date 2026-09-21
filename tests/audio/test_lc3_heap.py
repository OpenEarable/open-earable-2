#!/usr/bin/env python3
"""Exercise LC3 startup with the linked firmware's real Newlib heap.

Runs the application's ARM binary in isolation, including the LC3 allocator,
malloc and _sbrk. Only malloc's thread locks are bypassed: there is no scheduler
and each test runs on one host thread. This tests memory availability, not
Bluetooth timing or microphone hardware.
"""

import argparse
import struct
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


class Firmware:
    """One fresh, unbooted application image with an isolated test stack."""

    RETURN_ADDRESS = 0x001FF000
    STACK = 0x2100F000
    RESULT = 0x2100E000

    def __init__(self, path):
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

    def call(self, name, *args):
        """Call an integer/pointer ARM ABI function and require a successful return."""
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
        result = self.cpu.reg_read(UC_ARM_REG_R0)
        if result >= 2**31:
            result -= 2**32
        if result != 0:
            raise AssertionError(f"{name} returned {result}")

    def heap_used(self):
        return struct.unpack("<I", self.cpu.mem_read(self.symbols["heap_sz"], 4))[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path, help="Application zephyr.elf, not ipc_radio")
    args = parser.parse_args()
    config = dict(
        line.split("=", 1)
        for line in args.elf.with_name(".config").read_text().splitlines()
        if line.startswith("CONFIG_") and "=" in line
    )
    if config.get("CONFIG_NEWLIB_LIBC") != "y":
        raise AssertionError("This test requires the application's Newlib configuration")
    ram_end = int(config["CONFIG_SRAM_BASE_ADDRESS"], 0) + 1024 * int(config["CONFIG_SRAM_SIZE"])
    decoder_channels = 2 if config.get("CONFIG_SD_CARD_PLAYBACK") == "y" else 1

    # The firmware accepts these three rates, independently in each direction.
    # Recreate startup for each pair so a prior test cannot supply cached tables.
    for encoder_rate in (16000, 24000, 48000):
        for decoder_rate in (16000, 24000, 48000):
            firmware = Firmware(args.elf)
            available = ram_end - firmware.symbols["_end"]
            firmware.call("sw_codec_lc3_init", 0, 0, 10000)
            firmware.call("sw_codec_lc3_enc_init", encoder_rate, 16, 10000, 80000,
                          1, firmware.RESULT)
            firmware.call("sw_codec_lc3_dec_init", decoder_rate, 16, 10000, decoder_channels)
            used = firmware.heap_used()
            print(f"PASS encode={encoder_rate} decode={decoder_rate}: "
                  f"Newlib uses {used}/{available} bytes ({available - used} free)")


if __name__ == "__main__":
    main()
