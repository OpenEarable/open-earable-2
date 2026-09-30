#!/usr/bin/env python3
"""Capture the diagnostic presentation ring over SWD without halting audio.

Use the ELF matching the flashed CONFIG_AUDIO_SYNC_DIAGNOSTICS image. Run one
process per probe. `completed - raw_sdu` is measured in the same controller
clock on each earphone; its difference between ears removes the DMA pipeline's
common fixed delay. This measures the digital path, not the analog DAC delay.
"""

import argparse
import json
from pathlib import Path
import re
import struct
import subprocess
import time

import pylink
from intelhex import IntelHex

FIELDS = ("sequence", "completed", "raw_sdu", "sdu", "received",
          "presentation_delay", "underruns", "clock", "states", "queued")
CAPACITY = 128
RECORD_SIZE = len(FIELDS) * 4


def signed_delta(a, b):
    return ((a - b + 2**31) % 2**32) - 2**31


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snr", type=int, required=True)
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--image", type=Path,
                        help="Matching flashed HEX (defaults to ELF path with .hex suffix)")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=120)
    parser.add_argument("--speed", type=int, default=1000,
                        help="SWD speed in kHz; lower speeds can help long fixture cables")
    parser.add_argument("--inject-at", type=float,
                        help="Seconds into capture to insert one silent 1 ms block")
    parser.add_argument("--stall-at", type=float, action="append", default=[],
                        help="Seconds into capture to delay one frame's decoding (repeatable)")
    parser.add_argument("--stall-us", type=int, default=30000,
                        help="Duration of diagnostic processing stalls; 1..60000 us")
    parser.add_argument("--fix-mask", type=lambda value: int(value, 0),
                        help="Diagnostic isolation mask; set only with phone playback stopped")
    parser.add_argument("--presentation-at", action="append", default=[],
                        metavar="SECONDS:MICROSECONDS",
                        help="Temporarily override the sink's playback target for a margin test")
    parser.add_argument("--gdb", default="/opt/nordic/ncs/toolchains/ef4fc6722e/opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb")
    parser.add_argument("--nm", default="/opt/nordic/ncs/toolchains/ef4fc6722e/opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm")
    args = parser.parse_args()
    if args.fix_mask is not None and not 0 <= args.fix_mask <= 127:
        parser.error("--fix-mask must be between 0 and 127")
    delay_steps = []
    for step in args.presentation_at:
        when, delay = step.split(":")
        when, delay = float(when), int(delay)
        if not 0 <= when < args.duration or not 3000 <= delay <= 60000:
            parser.error("Presentation steps require 0 <= seconds < duration and 3000..60000 us")
        delay_steps.append((when, delay))
    delay_steps.sort()
    if not 1 <= args.stall_us <= 60000 or any(
            not 0 <= when < args.duration for when in args.stall_at):
        parser.error("Processing stalls require 0 <= seconds < duration and 1..60000 us")
    stalls = sorted(args.stall_at)
    # Use the programmed image: ELF load-segment padding can differ from HEX.
    programmed = IntelHex(str(args.image or args.elf.with_suffix(".hex")))
    flash_segments = [(begin, bytes(programmed.tobinarray(start=begin, end=end-1)))
                      for begin, end in programmed.segments()
                      if 0x10000 <= begin < end <= 0xf0000]
    if not flash_segments:
        raise RuntimeError("No application flash segments found in image")
    symbols = subprocess.check_output([args.nm, str(args.elf)], text=True)
    address = int(next(line.split()[0] for line in symbols.splitlines()
                       if line.split()[-1] == "audio_sync_diagnostics"), 16)
    inject_address = None
    if args.inject_at is not None:
        inject_address = int(next(line.split()[0] for line in symbols.splitlines()
                                  if line.split()[-1] == "audio_sync_inject_blocks"), 16)
    stall_address = stall_count_address = None
    if stalls:
        stall_address = int(next(line.split()[0] for line in symbols.splitlines()
                                 if line.split()[-1] == "audio_sync_rx_stall_us"), 16)
        stall_count_address = int(next(line.split()[0] for line in symbols.splitlines()
                                       if line.split()[-1] == "audio_sync_rx_stall_count"), 16)
    probe = pylink.JLink()
    delay_address = None
    original_delay = None
    probe.open(serial_no=args.snr)
    probe.set_tif(pylink.enums.JLinkInterfaces.SWD)
    try:
        probe.connect("NRF5340_XXAA_APP", speed=args.speed)
        if probe.halted():
            raise RuntimeError("Target is halted; no valid live measurement possible")
        for location, expected in flash_segments:
            if bytes(probe.memory_read8(location, len(expected))) != expected:
                raise RuntimeError("Running flash does not match the supplied ELF")
        fix_mask = None
        mask_symbols = [line.split()[0] for line in symbols.splitlines()
                        if line.split()[-1] == "audio_sync_fix_mask"]
        if mask_symbols:
            mask_address = int(mask_symbols[0], 16)
            if args.fix_mask is not None:
                probe.memory_write32(mask_address, [args.fix_mask])
            fix_mask = probe.memory_read32(mask_address, 1)[0]
            if args.fix_mask is not None and fix_mask != args.fix_mask:
                raise RuntimeError("Isolation mask readback mismatch")
        elif args.fix_mask is not None:
            raise RuntimeError("Firmware has no diagnostic isolation mask")
        magic, initial = probe.memory_read32(address, 2)
        if magic not in (0x41535931, 0x41535932):
            raise RuntimeError("Diagnostic ABI mismatch: check the flashed ELF")
        fields = FIELDS + (("capture_rtc", "capture_timer", "free_timer") if magic == 0x41535932 else ())
        capacity = 96 if magic == 0x41535932 else CAPACITY
        record_size = len(fields) * 4
        if delay_steps:
            output = subprocess.check_output(
                [args.gdb, "-batch", str(args.elf), "-ex",
                 "p/x &ctrl_blk.pres_comp.pres_delay_us"], text=True)
            delay_address = int(re.search(r"\$\d+ = (0x[0-9a-f]+)", output)[1], 16)
            original_delay = probe.memory_read32(delay_address, 1)[0]
            if not 3000 <= original_delay <= 60000:
                raise RuntimeError("No negotiated presentation delay; start playback first")
        side = probe.memory_read32(0xff80f4, 1)[0]
        args.output.parent.mkdir(parents=True, exist_ok=True)
        started = time.monotonic()
        end = started + args.duration
        last = initial
        read_failures = 0
        stall_count = probe.memory_read32(stall_count_address, 1)[0] if stalls else None
        with args.output.open("x", buffering=1) as output:
            output.write(json.dumps(dict(event="start", snr=args.snr, side=side,
                                         initial=initial, address=address,
                                         fix_mask=fix_mask,
                                         elf=str(args.elf.resolve()), time=time.time())) + "\n")
            while time.monotonic() < end:
                while stalls and time.monotonic() - started >= stalls[0]:
                    stalls.pop(0)
                    probe.memory_write32(stall_address, [args.stall_us])
                    output.write(json.dumps(dict(event="processing_stall",
                                                 duration_us=args.stall_us,
                                                 time=time.time())) + "\n")
                while delay_steps and time.monotonic() - started >= delay_steps[0][0]:
                    _, delay = delay_steps.pop(0)
                    probe.memory_write32(delay_address, [delay])
                    if probe.memory_read32(delay_address, 1)[0] != delay:
                        raise RuntimeError("Presentation override readback mismatch")
                    output.write(json.dumps(dict(event="presentation_override", delay_us=delay,
                                                 time=time.time())) + "\n")
                if inject_address is not None and time.monotonic() - started >= args.inject_at:
                    probe.memory_write32(inject_address, [1])
                    output.write(json.dumps(dict(event="inject", blocks=1, time=time.time())) + "\n")
                    inject_address = None
                try:
                    if stall_count_address is not None:
                        new_stall_count = probe.memory_read32(stall_count_address, 1)[0]
                        if new_stall_count != stall_count:
                            output.write(json.dumps(dict(event="processing_stall_completed",
                                                         count=new_stall_count,
                                                         time=time.time())) + "\n")
                            stall_count = new_stall_count
                    before = probe.memory_read32(address + 4, 1)[0]
                    data = (bytes(probe.memory_read8(address + 8, capacity * record_size))
                            if before > last else b"")
                    after = probe.memory_read32(address + 4, 1)[0]
                    read_failures = 0
                except pylink.errors.JLinkReadException as error:
                    read_failures += 1
                    output.write(json.dumps(dict(event="read_error", time=time.time(),
                                                 message=str(error))) + "\n")
                    if read_failures >= 3:
                        raise
                    time.sleep(0.2)
                    continue
                if before < last:
                    raise RuntimeError("Target reset during measurement")
                if before - last > capacity:
                    output.write(json.dumps(dict(event="lost", records=before-last-capacity)) + "\n")
                    last = before - capacity
                if before > last:
                    now = time.time()
                    for sequence in range(last + 1, before + 1):
                        if after - sequence >= capacity:
                            output.write(json.dumps(dict(event="overwritten", sequence=sequence)) + "\n")
                            continue
                        values = struct.unpack_from(f"<{len(fields)}I", data, ((sequence-1) % capacity)*record_size)
                        record = dict(zip(fields, values))
                        if record["sequence"] != sequence:
                            raise RuntimeError("Inconsistent ring record")
                        record.update(time=now, snr=args.snr, side=side,
                                      timestamp_valid=not bool(record["states"] & (1 << 16)),
                                      bad_frame=bool(record["states"] & (1 << 17)),
                                      startup_muted=bool(record["states"] & (1 << 18)),
                                      startup_fading=bool(record["states"] & (1 << 19)),
                                      delay_us=signed_delta(record["completed"], record["raw_sdu"]),
                                      estimated_delay_us=signed_delta(record["completed"], record["sdu"]),
                                      receive_us=signed_delta(record["received"], record["raw_sdu"]))
                        output.write(json.dumps(record) + "\n")
                    last = before
                time.sleep(0.1)
            output.write(json.dumps(dict(event="end", final=last, time=time.time(),
                                         halted=probe.halted())) + "\n")
        print(json.dumps(dict(snr=args.snr, side=side, records=last-initial,
                              output=str(args.output))), flush=True)
    finally:
        try:
            if delay_address is not None and original_delay is not None:
                probe.memory_write32(delay_address, [original_delay])
        finally:
            probe.close()


if __name__ == "__main__":
    main()
