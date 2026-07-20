# OpenEarable Battery Debug Tools

These scripts read and debug the OpenEarable v2 battery path through a J-Link
connection. They are meant for bring-up and recovery when the device may not be
booting far enough to expose logs, shell, BLE, or USB services.

## Does this need special firmware?

No. `battery_debug.py` talks directly to the battery ICs over SWD/J-Link by
briefly halting the nRF5340 application core and borrowing its TWIM1 peripheral.
It does not require the recovery firmware, normal firmware, RTT shell, USB, BLE,
or the application to be alive.

The tool is still hardware-specific: it assumes the OpenEarable v2 wiring:

- Fuel gauge: TI BQ27220 at I2C address `0x55`
- Charger: TI BQ25120A at I2C address `0x6a`
- Bus: app-core `i2c1` / TWIM1
- SDA: P0.21
- SCL: P0.24
- Charger PG: P0.18, active low
- Charger CD: P0.17

Because the tool uses SWD and EasyDMA scratch RAM, it briefly disturbs the
running firmware. By default it resumes the app core afterwards.

## Requirements

- A connected J-Link probe
- Python with `pylink` available
- The device must have enough target power for SWD and the battery ICs

Paste and replace your J-Link serial number where it is required [YOUR_JLINK_SERIAL_NUMBER].

## Voltage Only

Use this when you only want the battery voltage:

```bash
python3 tools/battery/battery_debug.py voltage --snr [YOUR_JLINK_SERIAL_NUMBER]
```

For scripts, print only integer millivolts:

```bash
python3 tools/battery/battery_debug.py voltage --snr [YOUR_JLINK_SERIAL_NUMBER] --raw
```

There is also a convenience wrapper:

```bash
python3 tools/battery/check_voltage.py --snr [YOUR_JLINK_SERIAL_NUMBER]
```

## Full Status

Read fuel-gauge and charger state:

```bash
python3 tools/battery/battery_debug.py status --snr [YOUR_JLINK_SERIAL_NUMBER]
```

This prints voltage, charger state, charger fault bits, power-good state, charge
enable/high-Z state, temperature, state of charge, and related raw registers.

## Low-Battery Recovery

Configure charging if the battery is below the start threshold:

```bash
python3 tools/battery/battery_debug.py recover --snr [YOUR_JLINK_SERIAL_NUMBER]
```

Force charger reset/configuration even if the voltage is already above the
threshold:

```bash
python3 tools/battery/battery_debug.py recover --snr [YOUR_JLINK_SERIAL_NUMBER] --force
```

Monitor continuously and reset the charger again if it enters fault:

```bash
python3 tools/battery/battery_debug.py recover --snr [YOUR_JLINK_SERIAL_NUMBER] --continuous --reset-on-fault
```

The old helper name is kept as a compatibility wrapper:

```bash
python3 tools/battery/recover_low_battery.py --snr [YOUR_JLINK_SERIAL_NUMBER] --continuous --reset-on-fault
```

## Useful Options

- `--speed-khz 1000`: SWD speed.
- `--no-resume`: leave the app core halted after the operation.
- `--start-below-mv 3000`: recovery starts below this voltage.
- `--target-mv 3300`: recovery exits after reaching this voltage unless
  `--continuous` is set.
- `--allow-deep-discharge`: allow recovery below the safety threshold. Use only
  with physical supervision.

## Notes

- Reading voltage/status is non-destructive apart from briefly halting the CPU.
- Recovery writes charger registers and drives the charger `CD` pin low to allow
  charging.
- If SWD cannot connect, the battery may still be too low or the target may not
  have enough power for debug access.
