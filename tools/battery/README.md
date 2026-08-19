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
- J-Link DLL/software version `8.82`
- PyLink version `2.0.1`
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

## Full Status

Read fuel-gauge and charger state:

```bash
python3 tools/battery/battery_debug.py status --snr [YOUR_JLINK_SERIAL_NUMBER]
```

This prints voltage, charger state, decoded charger and temperature faults,
power-good state, charge enable/high-Z state, temperature, state of charge, and
related raw registers.

The charger status register has a few latched bits that are easy to confuse:

- `timer_fault=True` means the BQ25120A safety timer expired. This is reported
  in the status register, so `charger_fault=0x00` can still be valid.
- `reset_fault=True` is the separate reset-event latch. It may be set while the
  charger is still actively charging.
- `charger_ctrl=0x51` decodes as charging with `reset_fault=True` and
  `timer_fault=False`.
- `charger_ctrl=0xd9` decodes as charger fault with `timer_fault=True`; recovery
  should clear it by toggling `CD` and reconfiguring the charger.

The tool also names `VIN_OV`, `VIN_UV`, `BAT_UVLO`, `BAT_OCP`, VINDPM, and the
three charger temperature states. `ts_fault=0x88` means charger-side temperature
monitoring is enabled and currently normal; it is not a temperature fault.

## Low-Battery Recovery

Configure charging if the battery is below the start threshold:

```bash
python3 tools/battery/battery_debug.py recover --snr [YOUR_JLINK_SERIAL_NUMBER]
```

For automatic recovery of resettable charger faults, use:

```bash
python3 tools/battery/battery_debug.py recover --snr [YOUR_JLINK_SERIAL_NUMBER] --reset-on-fault
```

The application core remains halted while recovery is running. Recovery exits
after the battery reaches `--target-mv` (3300 mV by default) with no blocking
fault, then verifies that the application core resumed. It also stops if the
voltage fails to rise by at least 10 mV within 10 minutes by default.

Force charger reset/configuration even if the voltage is already above the
threshold:

```bash
python3 tools/battery/battery_debug.py recover --snr [YOUR_JLINK_SERIAL_NUMBER] --force
```

Monitor continuously and reset the charger again if it enters fault:

```bash
python3 tools/battery/battery_debug.py recover --snr [YOUR_JLINK_SERIAL_NUMBER] --continuous --reset-on-fault
```

During recovery, safety-timer faults are reset even without `--reset-on-fault`.
The reset sequence pulses the charger `CD` pin, verifies that it moved high and
back low, resets the charger registers, restores the charger configuration, and
checks that the timer fault cleared. A failed sequence is retried once. If a
fault returns until `--max-fault-resets` is reached, recovery stops with an
error instead of silently continuing with charging stopped.

`BAT_UVLO`, VINDPM, and the cool/warm temperature derating states do not trigger
repeated resets; recovery monitors them while voltage progresses. Missing input
power, input overvoltage, battery overcurrent, and a hot/cold temperature
suspension stop recovery immediately because software cannot safely clear the
underlying electrical condition.

## Useful Options

- `--speed-khz 1000`: SWD speed.
- `--no-resume`: leave the app core halted after the operation.
- `--start-below-mv 3000`: recovery starts below this voltage.
- `--target-mv 3300`: recovery exits after reaching this voltage unless
  `--continuous` is set.
- `--max-fault-resets 3`: maximum automatic charger resets during monitoring.
- `--stall-minutes 10`: stop after this long without meaningful voltage
  progress; use `0` to disable.
- `--stall-min-rise-mv 10`: voltage increase that resets the stall timer.
- `--allow-deep-discharge`: allow recovery below the safety threshold. Use only
  with physical supervision.

## Notes

- Reading voltage/status is non-destructive apart from briefly halting the CPU.
- Recovery writes charger registers and drives the charger `CD` pin low to allow
  charging.
- `--continuous` keeps the application core halted and continues monitoring
  after the target voltage; stop the command to resume the firmware.
- If SWD cannot connect, the battery may still be too low or the target may not
  have enough power for debug access.

## Tests

Run the battery-tool tests without connecting hardware:

```bash
python3 -m unittest discover -s tools/battery -p 'test_*.py' -v
```
