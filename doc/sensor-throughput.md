# Sensor streaming throughput

Sensor GATT notifications aggregate consecutive samples of the same sensor, up to
244 value bytes (or the negotiated ATT MTU minus three, if smaller). The existing
10-byte header and trailing little-endian uint16 sample period in microseconds
are unchanged. Samples are combined only when their timestamps form an exact
arithmetic sequence; gaps, rate changes and nonmonotonic timestamps start another
packet. Partial batches are eligible for transmission after 20 ms. Queueing and
radio congestion can add latency. SD records and acquisition messages retain
their original fixed layout.

The sender keeps up to 12 notifications in flight, owns each payload until its
completion callback, wakes on completion, and retries temporary stack exhaustion
for at most 100 ms. Queue overflow and exhausted retries remain possible and are
counted. The application has 16 ATT/L2CAP/ACL TX buffers. The 128-record publication
and BLE queues leave the required audio heap available; increasing queues without
checking the audio heap assertion is unsafe.

Pressure and optical temperature run on a separate, lower-priority worker so
conversion waits do not block FIFO servicing. Stopping a sensor cancels and joins
its pending work before putting its peripheral supply or driver to sleep.
The network controller reserves 2500 us per ACL event and permits event extension.
Actual airtime still depends on the central, retransmissions and ISO audio events.

## Optional compact IMU format

The sensor service exposes an optional notification characteristic
`34c2e3c1-34aa-11eb-adc1-0242ac120002`. Reading its one-byte capabilities returns
bit 0 for compact IMU support. New clients subscribe to this characteristic;
older clients keep using `34c2e3bc-34aa-11eb-adc1-0242ac120002`, which always carries
the legacy float representation. Subscriptions reset on disconnect and pending
batches are discarded when subscriptions change. If both channels are subscribed,
both receive legacy-format batches; the new client supports either format. This
also keeps legacy clients safe when Android shares a physical BLE connection
between apps. Two subscribed channels duplicate notifications and use extra airtime.

Compact IMU packets use wire sensor ID `0x80`, mapped by the client to logical
sensor ID 0 and its unchanged nine-float ParseInfo scheme. Each 24-byte sample is:

| Offset | Encoding | Physical conversion |
|---|---|---|
| 0–5 | Three little-endian signed int16 accel axes | value × (2 × 9.80665 / 32768) m/s² |
| 6–11 | Three little-endian signed int16 gyro axes | value × (2000 / 32768) degrees/s |
| 12–23 | Three little-endian float32 compensated magnetometer axes | µT, unchanged |

All six integer axes must round-trip exactly through the firmware's current
float conversion. Otherwise that sample uses the legacy format, including NaNs,
out-of-range values or future driver scales. Magnetometer compensation remains
unchanged and retains float precision. Compact and legacy packets may coexist;
clients must support both after opting in. A matching Flutter SDK change handles
channel selection and decodes the same physical units for charts and CSV export.

## Diagnostics

`sensor_stream_stats[8]` contains cumulative atomic per-sensor sample counters:
`produced`, `acquisition_dropped`, `enqueued`, `queue_dropped`, `submitted`,
`completed`, `send_dropped`, `invalid`, `mtu_dropped`, plus notification `bytes`,
`notifications`, and `compact_samples`. `invalid` counts malformed records;
other drop/delivery fields count samples (transmitted copies when both channels
are subscribed). `bytes` includes notification headers,
not ATT/L2CAP/link-layer overhead. Inspect deltas over a recording window. The
counters do not establish hardware FIFO loss before acquisition or delivery into
the phone application; compare them with receiver sample counts. Completion is a
stack callback, not an application acknowledgement. Statistics reset on reboot.
