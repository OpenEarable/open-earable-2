# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

Firmware for the OpenEarable v2 earbud: a Zephyr / nRF Connect SDK application for the dual-core
nRF5340, forked from Nordic's `nrf5340_audio` sample. It is built as an LE Audio unicast server
(headset) with a large amount of application code layered on top: multi-sensor acquisition, SD-card
logging, battery/charge management, and custom GATT services.

- SDK: `sdk-nrf` **v3.0.1**, pinned in [west.yml](west.yml). Toolchain v3.0.1.
- Board target: `openearable_v2/nrf5340/cpuapp`.
- Language: mixed C and C++17 (`CONFIG_CPP=y`, `CONFIG_STD_CPP17=y`). Most application subsystems
  are C++ with `extern "C"` shims so Nordic's C code can call them.

## Build, Flash, Debug

Day-to-day development uses the **nRF Connect for VS Code** extension (see [README.md](README.md)
for full setup). Two build configurations exist and their *build directory names matter* because the
flash scripts hardcode them:

| Config | Build dir | How |
|---|---|---|
| FOTA (default, what CI builds) | `build_fota` | extra CMake arg `-DFILE_SUFFIX="fota"`, no base Kconfig fragment |
| Non-FOTA | `build` | base config `prj.conf`, no `FILE_SUFFIX` |

`FILE_SUFFIX=fota` is Zephyr's file-suffix mechanism: it selects [prj_fota.conf](prj_fota.conf),
[sysbuild_fota.conf](sysbuild_fota.conf), [pm_static_fota.yml](pm_static_fota.yml), and
`boards/openearable_v2_nrf5340_cpuapp_fota.overlay` instead of the plain variants.

Command line (from a west workspace whose manifest repo is this directory):

```bash
west build --board openearable_v2/nrf5340/cpuapp --pristine=always . -- -DFILE_SUFFIX="fota"
```

Flashing requires a genuine J-Link and `nrfjprog`; run from the repo root:

```bash
./tools/flash/flash_fota.sh --snr <jlink-snr> [--left|--right] [--standalone] [--hw 2.0.1]
./tools/flash/flash.sh      --snr <jlink-snr> [--left|--right]   # non-FOTA, uses ./build
./tools/flash/recover.sh    --snr <jlink-snr>                    # recover both cores
```

Omitting `--left`/`--right` makes the script back up and restore UICR, preserving the existing
left/right role, SIRK, and hardware version. Passing `--left`/`--right` **chip-erases and rewrites**
those UICR words, which breaks the existing left/right bond.

Debug output: RTT is the shell backend; the log backend is UART at 115200 over the J-Link VCOM.

### Testing

There is no unit-test suite. [sample.yaml](sample.yaml) declares build-only Twister configurations.
The de-facto validation gate — matching [.github/workflows/build_firmware.yaml](.github/workflows/build_firmware.yaml) —
is a clean FOTA build; anything touching sensors, BLE, power, storage, or audio should additionally
be smoke-tested on hardware. See [CONTRIBUTING.md](CONTRIBUTING.md).

## Architecture

### Entry point and startup order

[unicast_server/main.cpp](unicast_server/main.cpp) is `main()`. Ordering there is load-bearing:

1. `power_manager.begin()` — must run first; it enables the 1.8 V / 3.3 V / SD load switches. Any
   peripheral touched before this sees dead rails. This is why USB mass storage uses the *next* USB
   stack (`CONFIG_USB_DEVICE_STACK_NEXT`) in the FOTA config: the legacy MSC stack probes the disk
   in a pre-`main()` init hook, before the SD rail is up.
2. USB device stack + `sd_mass_storage_init()`.
3. `streamctrl_start()` — brings up BLE and the LE Audio unicast server.
4. LED/pairing state derived from UICR SIRK and bond count.
5. `init_sensor_manager()`, then each GATT service, parse-info service, connection-interval policy,
   and time sync.

### zbus is the spine

Subsystems are decoupled through Zephyr zbus channels rather than direct calls. The channel is the
integration point to look for when tracing a feature end to end:

| Channel | Defined in | Carries |
|---|---|---|
| `sensor_chan` | [src/SensorManager/SensorManager.cpp](src/SensorManager/SensorManager.cpp) | `struct sensor_msg` (sample + `sd`/`stream` routing flags) |
| `battery_chan` | [src/Battery/PowerManager.cpp](src/Battery/PowerManager.cpp) | `struct battery_data` |
| `sd_card_chan` | [src/SD_Card/SD_Card_Manager/SD_Card_Manager.cpp](src/SD_Card/SD_Card_Manager/SD_Card_Manager.cpp) | card insertion/removal |
| `button_chan` | [src/buttons/button_manager.c](src/buttons/button_manager.c) | `struct button_msg` |
| `bt_mgmt_chan`, `le_audio_chan`, `volume_chan`, `cont_media_chan`, `sdu_ref_chan` | `src/bluetooth/**` | Nordic's BLE/audio state machine events |
| `audio_channel` | [src/audio/audio_datapath.c](src/audio/audio_datapath.c) | audio blocks |

Shared message types live in [include/openearable_common.h](include/openearable_common.h) (OpenEarable-specific:
`sensor_data`, `earable_state`, `charging_state`, `sensor_id`, …) and
[include/zbus_common.h](include/zbus_common.h) (inherited from Nordic).

### Sensors

[src/SensorManager](src/SensorManager) owns acquisition. `EdgeMlSensor` ([EdgeMLSensor.h](src/SensorManager/EdgeMLSensor.h))
is the abstract base — `init/start/stop` plus per-sensor `_sd_logging` / `_ble_stream` flags. Concrete
sensors (`IMU`, `Baro`, `PPG`, `Temp`, `BoneConduction`, `Microphone`) drive vendor drivers in the
sibling directories (`BMX160`, `BMA580`, `BMP388`, `MAXM86161`, `MLX90632`).

Each sensor pushes into `sensor_queue` (a `k_msgq`) from its own work item on `sensor_work_q`; a
dedicated publisher thread drains the queue onto `sensor_chan`. Consumers are the BLE sensor service
and the SD logger. Sample-rate options and byte layouts are *not* hardcoded in the app — they are
described by the parse-info scheme (below) so hosts can decode generically.

### Parse-info scheme

[src/ParseInfo](src/ParseInfo) publishes a machine-readable description of every sensor's packet
layout and available sample rates over GATT, and the same blob is embedded into SD log file headers.
[src/ParseInfo/README](src/ParseInfo/README) is the authoritative BLE protocol spec (service and
characteristic UUIDs, config packet format, data packet format). **When you add or change a sensor's
data layout you must update its scheme in [DefaultSensors.h](src/ParseInfo/DefaultSensors.h)** —
otherwise host-side parsing silently breaks.

### SD card

Three layers under [src/SD_Card](src/SD_Card):

- `SD_Card_Manager` — FAT/exFAT mount lifecycle, directory/file primitives, card-presence handling.
- `SDLogger` — the `.oe` binary recording format: a packed `FileHeader` (version, timestamp,
  device id, side, parse-info size) followed by the serialized parse-info blob and sensor records,
  buffered in `SD_BLOCK_SIZE`-aligned blocks.
- `MassStorage` — exposes the card as a USB MSC LUN so the host can pull recordings, plus an
  `sd_msc` shell command.

`.oe` files are decoded by the Colab notebook linked from the README.

### Audio

Nordic's datapath, modified. [src/audio/audio_datapath.c](src/audio/audio_datapath.c) is the hub: I2S
RX/TX blocks, presentation-delay handling, tone generation, and hooks into
`decimation_filter.cpp` / `audio_datapath_decimator.cpp` (downsampling mic audio for the sensor
stream) and `sdlogger_wrapper.cpp` (recording audio to SD). `streamctrl.c` is the stream state
machine; `sw_codec_select.c` wraps LC3.

Hardware codec gotcha: `CONFIG_NRF5340_AUDIO_CS47L63_DRIVER` is a leftover Nordic symbol name — in
this repo it compiles [src/modules/hw_codec_adau1860.cpp](src/modules/hw_codec_adau1860.cpp) driving
an **ADAU1860**, not a CS47L63. The low-level driver and DSP/FDSP program are in
[src/drivers](src/drivers).

### Bluetooth

[src/bluetooth](src/bluetooth) keeps Nordic's structure (`bt_management`, `bt_stream`,
`bt_content_control`, `bt_rendering_and_capture`) and adds `gatt_services/` with the OpenEarable-specific
services: `sensor_service`, `battery_service`, `button_service`, `led_service`, `audio_config_service`,
`device_info`. `bt_mgmt_conn_interval` + `conn_interval/` implement an adaptive connection-interval
policy installed from `main()`.

### Power, state, and device identity

- [src/Battery](src/Battery) — `PowerManager` (C++ singleton `power_manager`) coordinates the
  BQ25120A charger and BQ27220 fuel gauge, load switches, power-button debounce, and shutdown.
- [src/utils/StateIndicator.h](src/utils/StateIndicator.h) — single owner of the RGB LED. Charging
  states override connection states; BLE can override both via `led_service`. The state tables in the
  README are the spec.
- [src/utils/uicr.h](src/utils/uicr.h) — per-device identity persisted in UICR at
  `NRF_UICR_S_BASE + 0xF0`: left/right channel, SIRK, standalone flag, hardware revision. Written by
  the flash scripts via `nrfjprog --memwr`; read at boot to decide pairing behavior.

### Board definition

The board files live in [boards/teco/openearable_v2](boards/teco/openearable_v2) (devicetree,
`Kconfig.defconfig`, board init, MCUboot hook). They are not automatically visible to Zephyr — CI
copies `boards/teco/*` into `zephyr/boards/arm/` before building
(see [build_firmware.yaml](.github/workflows/build_firmware.yaml)); the VS Code extension setup
relies on the board being registered in the SDK's `boards/teco` directory. Custom devicetree bindings
are in [dts/bindings](dts/bindings).

## Conventions

Full rules in [CONTRIBUTING.md](CONTRIBUTING.md); the ones that most affect day-to-day edits:

- **Conventional commits** are required: `<type>(<scope>): <summary>` with type in
  `feat|fix|refactor|docs|test|chore|build|ci|perf`.
- **Rebase, never merge `main`** into a feature branch. Push with `--force-with-lease`.
- Public classes, functions, and headers are expected to carry doc comments covering parameters,
  ownership, failure modes, and hardware assumptions.
- Extend an existing subsystem under `src/` rather than adding a new top-level concept; if a new
  abstraction is unavoidable, justify it.
- Keep [README.md](README.md) and [CONTRIBUTING.md](CONTRIBUTING.md) in sync when changing build
  behavior, flash scripts, board targets, or public firmware APIs.

## Gotchas

- Many `build_*` directories exist at the repo root from past experiments. They are stale artifacts,
  not configurations — do not treat them as sources of truth.
- Zephyr cannot remount a FAT volume freely, so historically the device had to be powered off before
  inserting/removing the SD card (README says as much). SD hot-plug handling is active work on the
  current branch — check `SD_Card_Manager` and `MassStorage` before assuming either behavior.
- Application-level singletons are declared as `extern` globals in their headers (`power_manager`,
  `state_indicator`, `sdlogger`, `sdcard_manager`). Use those instances; do not construct new ones.
- `prj.conf` and `prj_fota.conf` diverge in more than DFU settings — notably the USB stack
  (`USB_DEVICE_STACK` vs `USB_DEVICE_STACK_NEXT`) and MSC. A change that works in one config may not
  compile in the other; CI only builds the FOTA config.
- Debug/diagnostic shell commands are registered across subsystems (`battery`, `hw_codec`, `dsp`,
  `audio_system`, `test`, `sd_msc`, `power`, `sd_card_playback`) and reach the device over RTT.
