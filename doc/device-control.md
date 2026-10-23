# Device control (firmware 2.3.2)

The tester can remove stored phone bonds and turn off an earphone after saving
its QC results. This operation preserves UICR hardware version, left/right
assignment and CSIP set identity, plus all other application settings.
It clears **all Bluetooth bonds**, not just the current phone.

## GATT protocol v1

All UUIDs share suffix `-4c7a-4d8b-9a61-6c9e2f183a70`:

| UUID prefix | Role | Access |
| --- | --- | --- |
| `8b5d1000` | Service | Discovery |
| `8b5d1001` | Control | Encrypted read / write with response |
| `8b5d1002` | Status | Indicate, encrypted CCC |

Read Control: one byte `01` (protocol version). Subscribe to Status **indications**,
then write `01 01` to Control (version, CLEAR_BONDS_AND_SHUTDOWN opcode).
The device indicates `01 01 00` (version, opcode, accepted). It begins terminal
cleanup only after the client confirms the indication, with a 250 ms grace
period for the ATT write response. This acknowledgement means **accepted**, not
that the subsequent flash erasure or hardware shutdown has completed.

Malformed/unsupported commands, prepared writes, writes without response,
missing indication subscriptions and overlapping requests are rejected. A failed
indication does not erase bonds. Indication parameters remain owned by Bluetooth
until its destroy callback, so another request cannot overwrite an in-flight ACK.

The deferred worker inhibits advertising, drains any in-flight advertising work,
stops all extended advertising sets, erases bonds and verifies the bond count is
zero before calling PowerManager. A failed erase/shutdown is logged, leaves
advertising inhibited and requires a restart. Normal shutdown preserves bonds.

On external power, terminal shutdown sets application-owned nRF5340
`GPREGRET[1]` to `0xA6` before the existing software reboot. PowerManager consumes
this one-shot marker on software reset to enter its pre-Bluetooth charging loop.
A deliberate button action wakes it. Other reset reasons discard the marker.
On battery, the existing system-off path is used.

## Client sequencing and fallback

Send and observe acknowledgement/disconnection for **both** earphones before
removing either Android bond: Android can disconnect an entire coordinated set
when one member is forgotten. Stop automatic reconnect attempts during cleanup.
A service-absent earphone must only be disconnected; retain its Android bond.
Discovery errors, unknown protocol versions and missing ACKs are failures, not
an absent-service fallback. A lost BLE link alone is not proof of bond erasure.

## Validation

`sh tests/host/device_control/run.sh` executes the production GATT callbacks with
transport/workqueue doubles. It checks command validation, missing CCC, indication
failure, concurrent requests, ACK lifetime and deferred execution. It runs in CI.
A clean FOTA build validates the actual Zephyr/Nordic API and memory budget.
Hardware validation must cover both earphones, Android bond removal, preservation
of UICR identity, zero bonds after waking, and charging-only versus battery shutdown.
