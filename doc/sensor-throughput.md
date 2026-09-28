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

## Unchanged sensor protocol

All sensor traffic uses the existing notification characteristic
`34c2e3bc-34aa-11eb-adc1-0242ac120002`. No characteristics, sensor IDs,
capability negotiation or value encodings are added. IMU samples retain all nine
float32 values (36 bytes), copied byte for byte. Configuration, timestamps,
ParseInfo and SD record formats are unchanged. No app or SDK update is required.

The existing decoder already accepts multiple samples followed by their uint16
sample period. Batching fills that existing layout more efficiently; packet
lengths and delivery timing change, while values and represented timestamps do
not. Samples with inconsistent timestamp spacing start a new packet.

## Diagnostics

`sensor_stream_stats[8]` contains cumulative atomic per-sensor sample counters:
`produced`, `acquisition_dropped`, `enqueued`, `queue_dropped`, `submitted`,
`completed`, `send_dropped`, `invalid`, `mtu_dropped`, plus notification `bytes`,
`notifications`. `invalid` counts malformed records;
other drop/delivery fields count samples. `bytes` includes notification headers,
not ATT/L2CAP/link-layer overhead. Inspect deltas over a recording window. The
counters do not establish hardware FIFO loss before acquisition or delivery into
the phone application; compare them with receiver sample counts. Completion is a
stack callback, not an application acknowledgement. Statistics reset on reboot.
