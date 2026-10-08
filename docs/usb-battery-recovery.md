# USB battery recovery

FOTA builds enable `CONFIG_USB_BATTERY_RECOVERY`. On USB power, before the normal
power manager starts, the application asserts the BQ25120A CD pin to inhibit
charging and checks the fuel gauge using bounded I2C transfers.

It enters USB management mode if the gauge cannot be read, remains uninitialized,
is in configuration update mode, or reports any of these conditions:

- Battery absent or charging inhibited.
- Overtemperature flags, or temperature outside the existing 0–45 °C charging range.
- Voltage below the existing 2.5 V charge-inhibit threshold, or above 4.5 V.

In this mode CD stays high, so USB can supply the system without enabling battery
charging. The application never starts the normal power manager, its charge-enable
workers, Bluetooth, audio or sensors. It serves the existing USB mcumgr interface.
Recovery is latched for this boot; a later improvement in readings does not enable
charging. Unplugging USB puts the processor into system-off with USB insertion as
a wake source. A healthy battery follows the previous boot/charging behavior,
including normal precharging of a low but chargeable cell. Battery-only boots and
non-FOTA builds retain their previous behavior.

The CD/power-path behavior is documented in the
[TI BQ25120A datasheet](https://www.ti.com/lit/ds/symlink/bq25120a.pdf).

The gauge gets up to ten initialization checks, 100 ms apart. An I2C error enters
recovery immediately; the board's 500 ms transfer timeout prevents a stuck bus
from trapping the application before USB initialization. The probe does not read
the charger's read-to-clear reset flag or rewrite the gauge configuration.

## Access

Connect a USB data cable. With `mcumgr` installed, list the serial ports and use
the one belonging to the earphone, for example on macOS:

```sh
ls /dev/cu.usbmodem*
mcumgr --conntype serial --connstring 'dev=/dev/cu.usbmodemXXXXX,baud=115200,mtu=512' image list
```

Replace the example port with the actual one. `image list` only reads image
metadata. The existing image upload/reset commands use the same connection.
There is no Bluetooth connection or normal RGB status indication in recovery.

## Limits and validation

This requires the updated application to be installed already. It cannot rescue
an older application that never exposes USB, or a device that cannot power its
processor. It does not add a USB recovery service to MCUboot.

CD is asserted when the application starts. Charging before that point, during
bootloader execution or a hardware reset, is not prevented by this patch. CD also
does not physically isolate the battery: the PMIC's power path can supplement an
insufficient USB supply from the battery. This is a management/reflash path, not
a battery-reconditioning procedure or a claim of zero battery current.

Automated tests cover normal boot, safe precharging, unsafe/invalid gauge values,
I2C and GPIO failures, USB removal during probing, and latched charge inhibition.
Hardware acceptance still requires checking USB enumeration and image transfers,
measuring the CD pin and battery current with a controlled battery simulator,
and checking unplug/replug and the PMIC watchdog interval. Do not use a damaged
cell to induce these test conditions.
