#!/usr/bin/env python3
"""Inspect decoded 440/660 Hz test audio without halting the earphone.

Requires the matching ELF, 48 kHz/16-bit stereo I2S, pylink-square and numpy.
This samples past FIFO contents, not the physical DAC output. Timestamp checks
reject chunks overwritten during the read or crossing a timing correction.
Run separately from capture.py so each probe has only one client.
"""

import argparse
import json
from pathlib import Path
import re
import subprocess
import time

import numpy as np
import pylink


def capture(serial, elf, output, gdb):
    expressions = ["&ctrl_blk.out.fifo", "&ctrl_blk.out.prod_blk_idx",
                   "&ctrl_blk.out.prod_blk_ts", "sizeof(ctrl_blk.out.prod_blk_ts)/4"]
    command = [gdb, "-batch", str(elf)]
    for expression in expressions:
        command += ["-ex", "p/x " + expression]
    addresses = [int(value, 16) for value in re.findall(
        r"\$\d+ = (0x[0-9a-f]+)", subprocess.check_output(command, text=True))]
    fifo, producer, timestamps, capacity = addresses
    probe = pylink.JLink()
    probe.open(serial_no=serial)
    probe.set_tif(pylink.enums.JLinkInterfaces.SWD)
    observations = []
    try:
        probe.connect("NRF5340_XXAA_APP", speed=4000)
        assert not probe.halted()
        for attempt in range(24):
            index = probe.memory_read16(producer, 1)[0]
            assert index < capacity
            indices = [(index - 40 + k) % capacity for k in range(20)]
            before = probe.memory_read32(timestamps, capacity)
            values = []
            spans = [(indices[0], min(capacity, indices[0] + 20))]
            if indices[-1] < indices[0]:
                spans += [(0, indices[-1] + 1)]
            for begin, end in spans:
                values += probe.memory_read16(fifo + begin * 192, (end - begin) * 96)
            after = probe.memory_read32(timestamps, capacity)
            coherent = all(before[i] == after[i] for i in indices)
            contiguous = all((before[b] - before[a]) % 2**32 == 1000
                             for a, b in zip(indices, indices[1:]))
            samples = np.array(values, dtype=np.uint16).view(np.int16).reshape(-1, 2)
            np.save(output / f"chunk-{attempt}.npy", samples)
            t = np.arange(len(samples)) / 48000
            basis = np.column_stack([np.ones(len(t)), np.sin(2*np.pi*440*t),
                                     np.cos(2*np.pi*440*t), np.sin(2*np.pi*660*t),
                                     np.cos(2*np.pi*660*t)])
            channels = []
            for channel in samples.T:
                y = channel.astype(float)
                fit = basis @ np.linalg.lstsq(basis, y, rcond=None)[0]
                error = np.mean((y-fit)**2)
                variance = np.var(y)
                channels.append(dict(rms=float(np.sqrt(np.mean(y*y))),
                                     clipped=int(np.sum(np.abs(y) >= 32767)),
                                     residual_rms=float(np.sqrt(error)),
                                     r2=float(1-error/variance) if variance else None))
            observations.append(dict(coherent=coherent, contiguous=contiguous,
                                     channels=channels, time=time.time()))
            if sum(o["coherent"] and o["contiguous"] for o in observations) >= 12:
                break
            time.sleep(.1)
        assert not probe.halted()
    finally:
        probe.close()
    (output / "waveform.json").write_text(json.dumps(observations, indent=2) + "\n")
    valid = [o for o in observations if o["coherent"] and o["contiguous"]]
    print(json.dumps(dict(serial=serial, valid_chunks=len(valid),
                         channels=valid[-1]["channels"] if valid else None)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snr", type=int, required=True)
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gdb", default="/opt/nordic/ncs/toolchains/ef4fc6722e/opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    capture(args.snr, args.elf, args.output, args.gdb)
