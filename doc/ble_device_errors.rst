.. _ble_device_errors:

BLE Device Error Service
########################

The firmware exposes a dedicated Bluetooth LE GATT service for device-side
events that should be visible to user applications. The service is intended for
errors, warnings, and informational events such as SD card recording failures,
sensor acquisition problems, and firmware-level failures.

Service and Characteristic
**************************

.. list-table::
   :header-rows: 1

   * - Item
     - UUID
     - Properties
   * - Device Error Service
     - ``5f9c0001-6f4a-4c6b-9f0d-4f2f3b0a0001``
     -
   * - Device Error Event
     - ``5f9c0002-6f4a-4c6b-9f0d-4f2f3b0a0001``
     - Notify

The application must subscribe to the Device Error Event characteristic to
receive notifications. The current payload is 57 bytes, so an ATT MTU of at
least 60 is required. Client applications should request a larger MTU when the
platform allows it.

Payload Format
**************

All multi-byte values are little-endian.

.. list-table::
   :header-rows: 1

   * - Offset
     - Size
     - Field
     - Description
   * - 0
     - 1
     - ``version``
     - Payload version. Current value: ``1``.
   * - 1
     - 1
     - ``level``
     - Event severity.
   * - 2
     - 2
     - ``error_code``
     - Stable code describing the event.
   * - 4
     - 1
     - ``source_id``
     - Source component or sensor.
   * - 5
     - 4
     - ``timestamp_ms``
     - Device uptime in milliseconds.
   * - 9
     - 48
     - ``message``
     - Null-terminated ASCII text. Truncated when longer than 47 bytes.

Severity Levels
***************

.. list-table::
   :header-rows: 1

   * - Value
     - Level
   * - ``0``
     - Info
   * - ``1``
     - Warning
   * - ``2``
     - Error
   * - ``3``
     - Fatal

Source IDs
**********

.. list-table::
   :header-rows: 1

   * - Value
     - Source
   * - ``0x00``
     - IMU
   * - ``0x01``
     - Temperature/barometer
   * - ``0x02``
     - Microphone
   * - ``0x04``
     - PPG
   * - ``0x05``
     - Pulse oximeter
   * - ``0x06``
     - Optical temperature
   * - ``0x07``
     - Bone conduction
   * - ``0xFF``
     - System

Error Codes
***********

.. list-table::
   :header-rows: 1

   * - Code
     - Meaning
   * - ``0x0001``
     - SD card removed while recording.
   * - ``0x0002``
     - SD card is not mounted.
   * - ``0x0003``
     - SD card mount failed.
   * - ``0x0004``
     - SD card file open failed.
   * - ``0x0005``
     - SD card write failed.
   * - ``0x0006``
     - SD card log header write failed.
   * - ``0x0007``
     - SD card flush failed.
   * - ``0x0008``
     - SD card/ring buffer is full.
   * - ``0x0009``
     - SD card file close failed.
   * - ``0x0101``
     - Sensor queue is full.
   * - ``0x0102``
     - Sensor initialization failed.
   * - ``0x0103``
     - Sensor read failed.
   * - ``0x0104``
     - Invalid sensor sample rate.
   * - ``0x0105``
     - Sensor not found.
   * - ``0x0106``
     - Sensor configuration queue is full.
   * - ``0x0201``
     - BLE notification failed.
   * - ``0x0301``
     - Firmware fatal error.

Firmware Integration
********************

Use the helpers from ``device_error_service.h`` when a log should also be
visible to connected BLE clients:

.. code-block:: c

   device_error_log_err(DEVICE_ERROR_CODE_SD_WRITE_FAILED,
                        DEVICE_ERROR_SOURCE_SYSTEM,
                        "SD write failed: %d", ret);

The helper writes to the firmware log and sends a BLE notification when a client
is connected and subscribed. If no client is subscribed, the normal firmware log
still records the event.
