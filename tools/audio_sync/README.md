# Stereo timing diagnostics

Build the normal FOTA firmware with `CONFIG_AUDIO_SYNC_DIAGNOSTICS=y` in an
extra configuration fragment. Flash the same application image onto both
members of the pair, retaining their existing UICR identity and bonding data.
The September 30 investigation is summarized in
[`results-2026-09-30.json`](results-2026-09-30.json). That file identifies the
separate bench directory containing full captures, images and original backups.

`diagnostic.conf` selects the final candidate settings and the recorder.
Normal builds enable the playback-time, ongoing-correction, wraparound,
buffer-boundary and timestamp-validity fixes without the recorder. The more
aggressive clock regulator, restart reset and signed-clamp experiments remain
disabled by default.

Run `capture.py` in a separate process for each debug probe, supplying its
`--snr`, the matching `--elf`, an unused `--output` JSONL path and `--duration`
in seconds. Start music on the Android phone. Supply the matching flashed HEX with `--image` if its name differs from the
ELF with a `.hex` suffix. The tool verifies that running
flash matches that image before reading the ring, and never halts either core.
It needs `pylink-square`, `intelhex`, the SEGGER J-Link library, and an ARM
`nm` executable (override its path with `--nm`).

The recorder attaches the original Bluetooth SDU timestamp to each decoded
frame and records its I2S DMA release event using the hardware FRAMESTART
capture. `delay_us` subtracts those timestamps in the local controller's
clock, eliminating the arbitrary clock epoch. Comparing that latency between
the two ears of the same audio group reveals a digital presentation offset.
The fixed DMA completion offset is common to both ears. The measurement does
not include the codec's ASRC, DAC or acoustic path and therefore cannot by
itself certify acoustic phase alignment.

`analyze.py left.jsonl right.jsonl --output result.json` compares median
latency in common one-second host-time windows and also reports each ear's
latency distribution, underruns and synchronization states. Host time pairs
nearby observations; it is not used to measure microsecond latency. No paired
windows means no stereo result, not a pass. Retain raw records when inspecting
short-lived slips, stream transitions and timestamp anomalies. Allow a fresh
settling interval after each playback restart when interpreting steady-state
statistics. `--events events.json` accepts explicitly logged pause/play events
and records the excluded intervals. Do not infer a restart merely from a gap.
Per-frame target errors and the count outside ±100 µs are reported as well as
window medians, so a short slip cannot be hidden by aggregation.

Frames whose controller timestamp is absent are explicitly marked
`timestamp_valid=false`. Their reference is estimated from a recent valid
timestamp and the ISO sequence number to permit bounded packet-loss
concealment. The analysis counts these frames but excludes them from the
stereo timing comparison. `bad_frame` separately marks damaged or missing
encoded audio, including cases where its timestamp is valid. Neither kind of
missing data is evidence of good sound quality.

For a physical buffer positive control, `capture.py --inject-at 60 ...`
inserts one silent 1 ms block on that device after 60 seconds. This deliberately
changes playback timing, and must only be used with the matching diagnostic
image. It proves that the recorder observes an actual buffer delay, and lets
a continuous synchronizer demonstrate recovery. Omit it for normal tests.

For change isolation, pause phone playback before starting both recorders
with `--fix-mask N`, then resume playback. The mask uses these bits:

| Bit | Change |
| --- | --- |
| 0 | Measure the rendered block against its own SDU reference |
| 1 | Continue checking presentation delay after lock |
| 2 | Reset synchronization state on stream start (experimental) |
| 3 | Calculate phase differences across the 32-bit timestamp wrap |
| 4 | Bound buffer correction by the queued and free blocks |
| 5 | Clamp frequency before narrowing its signed argument (experimental) |
| 6 | Double the fine clock correction gain (experimental) |

The final candidate mask is 27. Timestamp-validity handling is a separate
compile-time option. Masks cannot disable that option or change Bluetooth's
negotiated presentation delay. The injected-block and mask variables are
available only in diagnostic builds and require physical debug access.

`--presentation-at 15:5000 --presentation-at 40:20000` deliberately overrides
the local playback target during an already-running stream for a buffer
margin experiment. This does **not** renegotiate Bluetooth QoS. The original
target is restored on normal exit and handled exceptions; resetting the
earphone clears a change if the host process is forcibly killed. This option
also requires the matching ARM `gdb` executable (`--gdb`).

`waveform.py` samples coherent chunks of decoded 48 kHz/16-bit audio from the
FIFO while the phone plays a known 440/660 Hz signal. It saves the samples and
reports clipping and sine-fit residuals. It requires NumPy and ARM GDB. Use
one probe client at a time; finish timing capture before running it. This is
an audio-content check before the codec and does not measure acoustic sound.

Run analysis checks with:

```sh
python3 -m unittest discover -s tools/audio_sync -v
```

## September 30 investigation

The PR applies the audio changes directly to the `2.2.10` branch at
`6bb51ef0`. The hardware measurements below used the already-installed
integration baseline `fcf67146` plus these application changes. The application
sources are identical between those baselines; the integration baseline also
set a 2500 µs controller connection-event reservation in the network-core
configuration. That radio override is not part of this PR. A clean FOTA build
and the eight host checks were repeated on the PR base; the complete rebased
application/network image has not been flashed for another hardware run.

The clocks could report lock while the two FIFOs retained different numbers
of audio blocks. The original presentation controller stopped measuring after
lock. It also compared the rendered block's reception timestamp with the
transport delay of a different, newly arriving frame. Measuring the rendered
SDU directly and continuing the presentation check removes that mismatch and
recovers from later slips.

On the firmware version originally installed on this pair, disabling ongoing
correction left a deliberately inserted 1 ms delay in place. Enabling it
recovered in approximately 0.4–0.6 seconds in the recorded tests. The two core
changes alone maintained a one-second median difference of −4 to 0 µs in the
45-second isolation run. The stronger clock regulator, restart reset and
signed frequency clamp were not needed for this result and are disabled in
the final configuration. Wrap-safe arithmetic and correction bounds remain
enabled to handle counter wrap and prevent corrections from crossing the
DMA/queue boundaries.

Short steady tests also aligned at a 5 ms presentation target. Longer tests
showed why that alone is insufficient: a frame received 18.4 ms after its SDU
reference caused two buffer underruns and a 2 ms slip even with a 20 ms target.
The controller recovered after 12 recorded frames, but that candidate failed
the per-frame check. The revised 40 ms default provides more delivery/decoding
margin. The delay override experiment changes only the sink's local target,
not the phone's QoS negotiation.

`--stall-at 20 --stall-us 30000` deliberately holds one received frame for
30 ms in the decoding thread while I2S and the radio continue running. Its
completion counter confirms that the stall actually ran. This permits a
controlled comparison of the 20 and 40 ms buffer margins. It is compiled only
in diagnostic builds; omit the option for normal playback measurements.

Both ears passed coherent decoded-waveform checks with no clipping and
similarly small residuals. The listener confirmed centered, clean sound on the
reduced candidate. Inspect the JSON for the long-run result, isolated damaged
packets, recording gaps and startup exclusions; a clean median alone is not
a claim of uninterrupted or acoustically measured output.
