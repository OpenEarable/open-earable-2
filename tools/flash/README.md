# Flashing tools

Keep the earables powered by USB or a charged battery. J-Link scripts require
SEGGER's J-Link software and Nordic's `nrfjprog` on `PATH`; list probes with
`nrfjprog --ids`. Run the existing single-device scripts from the repository
root.

| Script | Purpose | Images / connection |
| --- | --- | --- |
| `flash_fota.sh` | Flash one device with FOTA support | `build_fota/`, one J-Link |
| `flash_fota.ps1` | PowerShell equivalent of `flash_fota.sh` | `build_fota/`, one J-Link |
| `flash.sh` | Flash one device with the standard build | `build/`, one J-Link |
| `recover.sh` | Erase/unlock both nRF5340 cores; does not install firmware | One J-Link |
| `mcumgr_upload.sh` | Upload and test signed application/network images, then reset | Serial SMP connection; no J-Link |
| `flash_pair.sh` | Flash two devices concurrently and provision a left/right pair | `build_fota/`, two J-Links |

The J-Link flashers require both merged HEX files:
`merged_openearable_v2_nrf5340_cpuapp.hex` and
`merged_openearable_v2_nrf5340_cpunet.hex`. They also accept the older pair of
names, `merged.hex` and `merged_CPUNET.hex`. Build with merged images enabled
(`SB_CONFIG_MERGED_HEX_FILES=y`). A FOTA ZIP alone is not a complete J-Link image.

## One device: `flash_fota.sh`, `flash.sh`, `flash_fota.ps1`

```bash
# Retain the existing UICR side, hardware and pair configuration:
bash tools/flash/flash_fota.sh --snr 123456789
# Configure a left device explicitly:
bash tools/flash/flash_fota.sh --snr 123456789 --left --hw 2.0.1
```

| Bash option | PowerShell option | Meaning |
| --- | --- | --- |
| `--snr SERIAL` | `-Snr SERIAL` | Required J-Link serial |
| `--left` / `--right` | `-Left` / `-Right` | Choose one side; omit both to back up and restore UICR |
| `--hw x.y.z` | `-Hw x.y.z` | Requires a side; defaults to `2.0.0` when a side is supplied |
| `--standalone` | `-Standalone` | Requires a side; configure independent operation |
| — | `-Chip NRF53` | Chip family; default `NRF53` |
| — | `-Clockspeed 8000` | SWD speed in kHz; default `8000` |

All three erase phone bonds. Supplying a side skips UICR restoration, so existing
pair configuration is erased. Omitting both side flags restores UICR, but does
not preserve phone bonds. The single-device scripts share
`tools/flash/uicr_backup.hex`; do not run them concurrently. The FOTA scripts use
a pin reset, a five-second wait and an application reset; `flash.sh` uses a
single system reset.

## Recovery and serial updates

`bash tools/flash/recover.sh --snr SERIAL` accepts only `--snr`. It recovers both
cores and erases their firmware, UICR configuration and bonds. Reflash afterwards
with the desired side and hardware revision. Inspect both recovery command
results; this helper does not stop automatically if the first command fails.

`bash tools/flash/mcumgr_upload.sh` takes no command-line options. It requires
`mcumgr` and firmware exposing serial SMP. Edit its hard-coded serial port,
connection name, baud rate/MTU and signed-image paths before use. It uploads
image 0 and image 1, extracts hashes from `image list`, marks both for a test
boot, then resets. Check the hash selection against your device's image list;
this helper assumes a particular output order and does not stop on errors.

## Two devices: `flash_pair.sh`

This script can run from any directory and requires Bash 3.2 or later
(macOS/Linux), with no Python dependency.

```bash
bash tools/flash/flash_pair.sh --left-snr 123456789 --right-snr 987654321 --hw 2.0.1
```

| Option | Meaning |
| --- | --- |
| `--left-snr SERIAL` | Required J-Link serial for the left device |
| `--right-snr SERIAL` | Required, different J-Link serial for the right device |
| `--hw x.y.z` | Hardware revision for both devices; default `2.0.1`, each component 0–255 |
| `--build-dir DIR` | Directory containing the merged images; default is the repository's `build_fota/` |
| `--dry-run` | Validate arguments/image paths and print the plan; no hardware access |
| `-h`, `--help` | Print usage |

Set `NRFJPROG=/path/to/nrfjprog` to select a particular executable or wrapper.
The script programs the files in the selected directory; it does not download
firmware, compile it, or change its embedded firmware version.

Both devices are fully erased, including phone bonds and previous pairing
configuration. Separate original UICR backups are saved first. The new pair ID
is the XOR of the low 32 bits of the two factory chip IDs, matching normal
earable discovery. The script writes left/right, that shared ID and the hardware
revision; standalone mode remains disabled. This provisions the coordinated-set
identity, not phone bonding keys. Stock 2.3.0 ignores this stored identity for
CSIP; use the 2.3.1 fix in [#369](https://github.com/OpenEarable/open-earable-2/pull/369)
or firmware that honors the UICR pair identity.

Each run saves per-device logs, UICR backups, chip IDs, `pair.conf` and successful
verification stages under the selected build directory's `pair-logs/`. Both
devices must pass image verification and configuration readback before final
resets. A specific Nordic worker-process crash is retried once; other failures
stop the operation. A successful exit confirms programming and reset commands,
not runtime Bluetooth pairing or charging behavior.

Per-probe lock directories in `${TMPDIR:-/tmp}` block overlapping pair-script
runs by the same user. On interruption, the script waits for the current workers
before releasing locks. If a force-killed process leaves a lock, check its `pid`
file and ensure no flashing process is running before removing it. Do not run
other programming tools on these probes concurrently.
