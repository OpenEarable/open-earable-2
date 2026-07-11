# AGENTS.md — Build & dev notes for agents

## Build command

From the repo root, inside the NCS shell (v3.0.1 toolchain in
`~/ncs/toolchains/7cbc0036f4/`). This is exactly what the user's editor
tooling runs, verbatim — run this same command when building by hand so the
two invocations share a cache:

```
west build --build-dir /home/than/code/subvocal/open-earable-2/build \
  /home/than/code/subvocal/open-earable-2 \
  --pristine --board openearable_v2/nrf5340/cpuapp \
  -- \
  -DDEBUG_THREAD_INFO=Off \
  -Dopen-earable-2_DEBUG_THREAD_INFO=Off \
  -Dmcuboot_DEBUG_THREAD_INFO=Off \
  -Dipc_radio_DEBUG_THREAD_INFO=Off \
  -Db0n_DEBUG_THREAD_INFO=Off \
  -DBOARD_ROOT=/home/than/code/subvocal/open-earable-2
```

Incremental re-build once the cache exists (no `--pristine`):

```
west build --build-dir /home/than/code/subvocal/open-earable-2/build
```

### Why the flags look like this

- Build directory is `build/` (flat) — **not** `build/open-earable-2/`.
  Sysbuild then puts each sub-image under `build/<image-name>/`.
- **Sysbuild is auto-detected** from the repo's top-level `sysbuild.conf`;
  `--sysbuild` doesn't need to be passed explicitly. `sysbuild.conf` enables
  MCUboot for cpuapp + cpunet and external-flash DFU, so a non-sysbuild
  build would **silently skip mcuboot and ipc_radio** while the main app
  links fine. If you ever see only one image linking, you're in that trap.
- `--pristine` wipes the cache every time; the editor tooling does this
  so per-image sysbuild defines are applied cleanly. For hand-run
  incremental builds, drop `--pristine` (see the incremental form above).
- `-D<image>_DEBUG_THREAD_INFO=Off` silences per-image thread-info debug
  across all sysbuild sub-targets (`open-earable-2`, `mcuboot`,
  `ipc_radio`, `b0n`).
- The custom board `openearable_v2` lives at `boards/teco/openearable_v2/`
  (not in the NCS tree), so `-DBOARD_ROOT=<repo-root>` is always required.

### Board qualifier

`openearable_v2/nrf5340/cpuapp`. The `_ns` variant exists in `board.yml`
(TF-M non-secure) but the firmware targets the secure cpuapp core.

### Output artifacts

Under `build/`:
- `open-earable-2/zephyr/zephyr.elf` — main app
- `mcuboot/zephyr/zephyr.elf` — bootloader
- `ipc_radio/zephyr/zephyr.elf` — net-core image
- `merged.hex`, `merged_CPUNET.hex` — flashable combined images
- `dfu_application.zip` — MCUmgr / FOTA bundle
- `signed_by_mcuboot_and_b0_ipc_radio.hex` — signed net-core for DFU

### Flashing / DFU

`tools/flash/flash.sh` for J-Link; `tools/flash/mcu-manager_upload.sh` for
MCUmgr-over-BLE DFU. See `tools/flash/` for the canonical invocations.

## DTS binding locations

Board-specific bindings (those referenced by the board's `_common.dts`) must
live under `boards/teco/openearable_v2/dts/bindings/`, not the top-level
`dts/bindings/`. Reason: sub-images under sysbuild (mcuboot, ipc_radio) don't
search the main app's `dts/bindings/`, but they **do** search `BOARD_DIR/dts/
bindings/`. A binding that only lives at the top level will compile the main
app fine and then fail the mcuboot sub-image with `lacks binding` or
`lacks #power-domain-cells` — which is easy to miss if you only tail the
first image's output.

Current board-local bindings:
- `load-switch.yaml` — GPIO-gated power rail; acts as a power-domain
  controller for devices that reference it via `power-domains`.

## Power management (see POWER_DESCRIPTION.md, SYSTEM_DESCRIPTION.md)

- Three load switches: `ls_1_8` (gpio1.11), `ls_3_3` (BQ25120A LDO,
  gpio0.14), `ls_sd` (gpio1.12). All three have
  `zephyr,pm-device-runtime-auto` in DTS.
- `mx25r64` has `power-domains = <&load_switch>;` so the `spi_nor`
  driver's `pm_device_driver_init` sees the parent as unpowered and skips
  chip init at boot — the fix for the classic `spi_nor_wait_until_ready`
  hang when `ls_1_8` is off.
- DFU picks up `ls_1_8` via the MCUmgr hook in
  `src/utils/StateIndicator.cpp` (`MGMT_EVT_OP_IMG_MGMT_DFU_STARTED` →
  `pm_device_runtime_get(mx25r64)`).
- `mcuboot_hook.c::init_load_switch` runs at `SYS_INIT(PRE_KERNEL_2, 80)`
  inside mcuboot and drives `gpio1.11` + `gpio0.14` HIGH before mcuboot
  touches the flash. This keeps mcuboot working without needing
  `CONFIG_PM_DEVICE_POWER_DOMAIN` in the mcuboot config.

## User preferences (from memory)

- Run the user-given command verbatim from repo root; don't `cd` or rewrite
  paths.
- Don't speculate about root causes; read the current state before editing.
- The user provides the NCS shell for builds — but we can run the build
  ourselves when asked, using the command above.
