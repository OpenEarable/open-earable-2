# syncing Time on OpenEarable

## Data formats

### Time sync packet

The time sync packet is used to calculate the round-trip time (RTT) between the phone and the device, as well as to estimate the clock offset. The packet structure is as follows:

| Field         | Size (bytes) | Description                          |
|---------------|--------------|--------------------------------------|
| version       | 1            | Version of the time sync packet      |
| op            | 1            | 0 = request, 1 = response            |
| seq           | 2            | Sequence number, chosen by the phone |
| t1_phone      | 8            | Phone send time (microseconds)       |
| t2_dev_rx     | 8            | Device receive time (microseconds)   |
| t3_dev_tx     | 8            | Device transmit time (microseconds)  |

### Time offset packet
The time offset packet is used to calculate the offset between the device's internal time and the Unix time. The packet structure is as follows:

| Field          | Size (bytes) | Description                             |
|----------------|--------------|-----------------------------------------|
| offset         | 8            | Device time offset in microseconds      |

## Time syncing workflow

The phone initiates the sync by sending a time sync request packet with the current phone time (t1_phone) and a sequence number (seq). The device records the receive time (t2_dev_rx) upon receiving the packet. It then responds with a time sync response packet, including the recorded receive time (t2_dev_rx) and the transmit time (t3_dev_tx). The phone, upon receiving the response, calculates the round-trip time (RTT) and estimates the clock offset between the phone and the device. An offset between the device time and Unix time is then established using a time offset packet sent from the phone to the device via the Time Offset characteristic.

## GATT service

The `Time Sync` GATT service can be found with the service UUID `2e04cbf7-939d-4be5-823e-271838b75259`.

| Characteristic Name  | Characteristic UUID                    | Capabilities   | Description                               |
|----------------------|----------------------------------------|----------------|-------------------------------------------|
| Time Offset          | `2e04cbf8-939d-4be5-823e-271838b75259` | Write          | Offset between device time and unix time  |
| RTT calculation      | `2e04cbf9-939d-4be5-823e-271838b75259` | Write / Notify | Used to calculate RTT and sync time.      |