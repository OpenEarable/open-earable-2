.. _dmic_gain_reference:

DMIC Microphone Gain Control Reference
#######################################

.. contents::
   :local:
   :depth: 2

Overview
********

The DMIC (Digital Microphone) gain on the ADAU1860 codec can now be adjusted over Bluetooth using the Audio Config Service characteristic ``BT_UUID_DMIC_GAIN`` (UUID: ``1410df99-5f68-4ebb-a7c7-5e0fb9ae7557``).

Hardware Register Mapping
**************************

Based on ADAU186x datasheet (register DMIC_VOL0 @ address 0x4000C045):

- ``DMIC_VOL0`` / DMIC channel 0 controls the **external** microphone.
- ``DMIC_VOL1`` / DMIC channel 1 controls the **internal** microphone.

.. list-table::
   :header-rows: 1
   :widths: 20 20 60

   * - Register Value
     - Gain (dB)
     - Description
   * - 0x00
     - +24
     - Maximum positive gain
   * - 0x01
     - +23.625
     -
   * - 0x02
     - +23.25
     -
   * - ...
     - ...
     - Each step = -0.375 dB
   * - 0x3F
     - +0.375
     -
   * - **0x40**
     - **0**
     - **Reset/Default value**
   * - 0x41
     - -0.375
     -
   * - ...
     - ...
     - Each step = -0.375 dB
   * - 0xFD
     - -70.875
     -
   * - 0xFE
     - -71.25
     -
   * - 0xFF
     - ∞
     - Mute (infinite attenuation)

Gain Calculation
*****************

For any register value ``reg`` (0x00–0x3F for positive gains):

- **Gain (dB) = 24 - (reg × 0.375)**

For negative gains (0x41–0xFD):

- **Gain (dB) = -(reg - 0x40) × 0.375**

Examples
========

- ``0x00`` → 24 - (0 × 0.375) = **+24 dB**
- ``0x10`` → 24 - (16 × 0.375) = **+18 dB**
- ``0x20`` → 24 - (32 × 0.375) = **+12 dB** (initial value in this firmware)
- ``0x40`` → **0 dB**
- ``0x50`` → -(80 - 64) × 0.375 = **-6 dB**
- ``0x60`` → -(96 - 64) × 0.375 = **-12 dB**

BLE Protocol
************

Characteristic Details
======================

- **UUID**: ``0x1410df99-5f68-4ebb-a7c7-5e0fb9ae7557`` (under Audio Config Service)
- **Properties**: Read, Write
- **Payload**: 2 bytes

Write Command
=============

Send 2 bytes ``[external_gain_reg, internal_gain_reg]``:

.. code-block:: text

   Byte 0: External microphone gain register (DMIC_VOL0, 0x00–0xFF)
   Byte 1: Internal microphone gain register (DMIC_VOL1, 0x00–0xFF)

Read Response
=============

Returns 2 bytes showing current DMIC gain as ``[external_gain_reg, internal_gain_reg]``.

Usage Examples
**************

Set both microphones to +12 dB
===============================

Send: ``[0x20, 0x20]``

Set external to +18 dB, internal to +6 dB
=========================================

Send: ``[0x10, 0x30]``

(0x10 = 24 - 16×0.375 = +18 dB, 0x30 = 24 - 48×0.375 = +6 dB)

Set to 0 dB (neutral/reset)
============================

Send: ``[0x40, 0x40]``

Mute both microphones
=====================

Send: ``[0xFF, 0xFF]``

Set external to -12 dB (96 = 0x60), internal to 0 dB
=====================================================

Send: ``[0x60, 0x40]``

API Reference (C/C++)
*********************

Functions
=========

.. code-block:: c

   // Set DMIC gain for both microphones
   // gain_external_reg, gain_internal_reg: register values (0x00–0xFF)
   int hw_codec_mic_gain_set(uint8_t gain_external_reg, uint8_t gain_internal_reg);

   // Get current DMIC gain register value for external/internal microphone
   uint8_t hw_codec_mic_gain_get_external(void);
   uint8_t hw_codec_mic_gain_get_internal(void);

Example C Code
==============

.. code-block:: cpp

   // Set external and internal DMIC to +12 dB
   hw_codec_mic_gain_set(0x20, 0x20);

   // Read back the values
   uint8_t external = hw_codec_mic_gain_get_external(); // Should return 0x20
   uint8_t internal = hw_codec_mic_gain_get_internal(); // Should return 0x20

Implementation Details
**********************

- **File**: ``src/modules/hw_codec_adau1860.cpp``
- **Registers Written**: external mic ``DMIC_VOL0`` (0x4000C045), internal mic ``DMIC_VOL1`` (0x4000C046)
- **BLE Handler**: ``src/bluetooth/gatt_services/audio_config_service.c``
- **Logging**: INFO level logs show when gain is changed via BLE

Notes
*****

- Each register value change takes effect immediately on the hardware.
- Default at startup: 0x20 (+12 dB) for both channels (set in ``ADAU1860::begin()``).
- The characteristic is read/write; clients can query current gain and set new values.
- The 0.375 dB step provides fine-grained control with 256 possible values.
