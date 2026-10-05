# Firmware wire-contract baseline

This baseline records the implementation before migration to the `protocol`
submodule. `tests/protocol_contracts/ble_contracts.json` freezes all eight custom
legacy services' UUIDs, characteristic properties, permissions, and source paths.
`wire_vectors.json` provides literal hexadecimal payloads with semantic inputs.
These are deterministic examples derived from source, not hardware captures.
Do not regenerate these fixtures from a new codec: use them as its compatibility
oracle. A deliberate wire change requires a separately reviewed versioning plan.

## Ownership and verification

Schemas should own byte order, field widths/order, UUIDs, and payload descriptions.
Firmware retains GATT offset/error handling, semantic validation, device policy,
clock acquisition, allocation, batching, queues, and notification lifetimes.

Run the host checks from the repository root:

```sh
python3 -m unittest discover -s tests/protocol_contracts -v
```

They check BLE metadata against production sources and generated Zephyr definitions and compile/run the existing
sensor transport and ParseInfo component serializers against golden vectors.
The audio-configuration, LED, button, and power-saving migrations also test generated C and
Dart encoders and decoders against the same bytes (including decoded field
values and short-input rejection). The remaining payload vectors are source-audited examples; they do not yet run
the full ParseInfo serializer or SDLogger. Power-saving GATT callbacks are
also exercised with host BLE and manager test doubles. Those remaining paths need firmware
integration tests during migration. Generated bindings must also be checked
against these same vectors in C and Dart. No firmware behavior changes here.

## Payload contracts

Numbers use the current nRF little-endian representation. Legacy raw struct and
float copies assume this platform; new codecs must encode it explicitly.
Lengths below exclude ATT headers. R/W/N mean read/write/notify.

| Service / value | Properties | Ordered payload | Limits / numeric meanings |
|---|---|---|---|
| Audio / mode | R/W | `uint8 mode` | 0 normal, 1 transparency, 2 ANC |
| Audio / microphone | R/W | `uint8 microphone` | 0 left, 1 right |
| Audio / channel | R | `uint8 channel` | Channel-assignment value |
| Audio / DMIC gain | R/W | `uint8 outer, uint8 inner` | Raw codec register values; 0x40 = 0 dB, 0xff = mute |
| LED / RGB | W | `uint8 red, green, blue` | Exactly 3 bytes |
| LED / state | W | `uint8 mode` | 0 state indication, 1 custom; callback currently does not validate enum range |
| Button / state | R/N | `uint8 action` | 0 released, 1 pressed |
| Power saving / mode | R/W | `uint8 mode` | 0 Off, 1 Minimal, 2 Balanced, 3 Aggressive |
| Power saving / supported modes | R | `uint8 count`, repeated `(uint8 id, uint8 name_length, bytes name)` | No NUL; encoder buffer 128 bytes |
| Sensor / config | W | `uint8 id, uint8 rate_index, uint8 storage_mask` | Exactly 3 bytes; mask 0x01 BLE, 0x02 SD |
| Sensor / config status | R/N | Consecutive config records | No count prefix; length / 3 is record count |
| Sensor / recording name | R/W | Raw name bytes | Write maximum 63 bytes; no wire length prefix; local NUL added; embedded NUL truncates effective string |
| ParseInfo / list | R | `uint8 count, uint8 ids[count]` | Sensor IDs in declared order |
| ParseInfo / request | W | `uint8 id` | Exactly 1 byte |
| ParseInfo / scheme | R/N | See below | Notification follows successful request |
| Time / RTT | W/N | `uint8 version, uint8 op, uint16 seq, uint64 t1, t2, t3` | Exactly 28 bytes; version 1; request op 0, response op 1; times in microseconds |
| Time / offset | W | `int64 delta_us` | Exactly 8 bytes; **added** to current offset |
| Device / identifier | R | ASCII `0x%08X` plus zeros | Fixed 19 bytes; low 32 bits of boot device ID |
| Device / generation | R | Revision string bytes | `strlen` bytes, no NUL |
| Device / firmware | R | Firmware version string bytes | `strlen` bytes, no NUL |

### Sensor records and batching

Header: `uint8 sensor_id, uint8 payload_size, uint64 timestamp_us` (10 bytes).
One sample follows directly. For multiple samples, append a `uint16 period_us`
after all samples. `payload_size` includes this period; no sample-count prefix
exists. The timestamp belongs to the first sample. The BLE characteristic is
**notify only**. The batch implementation supports these widths:

| ID | Sample bytes |
|---|---:|
| 0 IMU | 36 |
| 1 temperature/barometer | 8 |
| 4 PPG | 16 |
| 6 optical temperature | 4 |
| 7 bone conduction | 6 |

Packets cannot exceed 244 bytes or the caller's smaller limit. A batch requires
the same sensor ID and strictly increasing timestamps. Periods must be equal
and fit uint16. A rejected append signals firmware to flush/retry; this policy
stays outside generated codecs. SDLogger's ordinary record writer stores the
same 10-byte header plus exactly `size` bytes, without fixed-array padding.

### ParseInfo schemes

A scheme is `uint8 id, uint8 name_length, bytes name, uint8 component_count`,
then that many component records, then config options. Firmware groups are
flattened: each component repeats its group name.

A component is `uint8 parse_type`, followed by three independently uint8-length
prefixed byte strings: group name, component name, unit. No terminating NULs.
Types 0..7 are int8, uint8, int16, uint16, int32, uint32, float, double.

Options start with `uint8 available_options` (0x01 streaming, 0x02 storage,
0x10 frequencies). **Only when bit 0x10 is set**, append `uint8 frequency_count,
uint8 default_index, uint8 max_ble_index, float frequencies[frequency_count]`.
Do not insert a union tag or encode an empty frequency header when absent.
Names and flattened counts have uint8 wire widths; legacy serializers do not
consistently reject overflow. Future validation must not silently wrap values.

The storage blob begins with the sensor list; for each ID in that list append
`uint16 scheme_size, bytes scheme[scheme_size]`. The BLE scheme payload itself
has no uint16 size prefix.

### `.oe` files

Current version 3 has a 27-byte fixed header:
`uint16 version, uint64 timestamp_us, uint32 header_size,
uint32 parse_info_size, uint64 device_id, uint8 side`, then the ParseInfo blob.
`header_size = 27 + parse_info_size` points to the first sensor record.
Side is 0 left, 1 right, 255 unknown. Records have no alignment padding.
Version 2's documented historical header is `uint16 version, uint64 timestamp`;
the current writer does not emit it, so historical fixtures require an actual
archived file before claiming verified backwards-reader coverage.

## Documentation discrepancies resolved

`src/ParseInfo/README` previously described sensor data as an ID plus a 32-bit
millisecond timestamp. Current framing has a size byte and a 64-bit microsecond
timestamp, plus the optional batching period above. Its read/notify description
also differs from the notify-only GATT declaration.

The README lists obsolete scheme/config UUIDs ending in `cb8`/`e3bd`. They are
not defined in current services. The JSON inventory freezes the implemented
sensor config `e3be`, ParseInfo list `cb9`, request `cba`, and response `cbb`.
Time-sync documentation describes offset assignment; the callback increments
its offset. Device identity is not an unpadded full 64-bit hexadecimal string.

## Existing submodule protocols

Audio-response already defines its contract in
`protocol/schemas/audio-response/protocol.yml`, including BLE metadata and
stable transfer-control tags 0 start, 1 commit, 2 abort. Keep that schema as its
source of truth. Wireless audio configuration is also schema-defined but has no
firmware service integration here. Neither needs a duplicate legacy schema.
Their existing generator tests remain separate from these legacy fixtures.

## Migration constraints

Preserve valid payload bytes and UUIDs before changing callback validation.
Existing write-offset behavior differs across services; normalizing it is a
separate behavioral change, not an automatic consequence of adopting codecs.
Test buffer capacity, truncated inputs, signed offsets, large 64-bit timestamps,
conditional fields, and single/multiple sensor samples during each migration.

Dart checks skip when its SDK is absent. Set `PROTOCOL_DART` to a Dart executable
when using a direct SDK binary instead of a Flutter wrapper.
