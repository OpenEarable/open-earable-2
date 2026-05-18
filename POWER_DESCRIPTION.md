# Power Management — OpenEarable v2

## Context

Description of how power management works today, plus a list of still-open
issues.

Reference reading:
- Zephyr PM: https://docs.zephyrproject.org/latest/services/pm/index.html
- Zephyr Device PM: https://docs.zephyrproject.org/latest/services/pm/device.html
- Zephyr Device Runtime PM: https://docs.zephyrproject.org/latest/services/pm/device_runtime.html

---

## 0. Boot sequence and load-switch lifecycle

### Hardware power rails (from schematic)

| Rail | Device | GPIO | Powers |
|------|--------|------|--------|
| **V_LS** (`ls_1_8`) | AP22118CA4-7 | `gpio1.11` | I2C2/I2C3 pull-ups, MX25R6435F flash VCC, SD card SPI level-shifter low side, sensors (1.8 V supply via connector) |
| **LS_LDO** (`ls_3_3`) | BQ25120A internal | `gpio0.14` | KTD2026 LED controller VIN, PPG LDO, sensors (3.3 V supply via connector), audio codec |
| **V_SD** (`ls_sd`) | AP22118CA4-7 | `gpio1.12` | SD card VDD, SD card SPI level-shifter high side |

**i2c1 pull-ups are on a permanent supply** (not V_LS). This is why
`PowerManager::begin` can talk to BQ25120A / BQ27220 / KTD2026 before any load
switch is up.

### Which rails each consumer needs

| Consumer | `ls_1_8` | `ls_3_3` | `ls_sd` | Notes |
|----------|:--------:|:--------:|:-------:|-------|
| Sensors (I2C3: BMA580, BMX160, BMP388, MLX90632) | **yes** | some | — | I2C3 pull-ups on V_LS; MLX90632/BMA580 also need 3.3 V |
| PPG (MAXM86161, I2C2) | **yes** | **yes** | — | I2C2 pull-ups on V_LS; PPG LDO on 3.3 V |
| Audio codec (ADAU1860, I2C2) | **yes** | — | — | I2C2 pull-ups on V_LS; codec enable via separate GPIO |
| LED controller (KTD2026, I2C1) | — | **yes** | — | I2C1 pull-ups on permanent supply; VIN needs 3.3 V |
| SPI NOR flash (MX25R6435F) | **yes** | — | — | VCC on V_LS; DFU only in the app |
| SD card (SDHC on SPI4) | **yes** | — | **yes** | Level-shifter needs V_LS (low) + V_SD (high) |
| BQ25120A / BQ27220 (I2C1) | — | — | — | Permanent supply |

### Idle profile (as implemented)

"Device on, BLE advertising, no sensors, no SD logging, no audio, no DFU" plus
a status LED flashing to indicate state:

- `ls_1_8`: **OFF** (unless an SD card is physically present → ON for level-shifter)
- `ls_3_3`: **ON** while LED is active (KTD2026 owns via `_get`/`_put`)
- `ls_sd`: **OFF** (unless an SD card is physically present → ON)

### Boot sequence

```
T=0   Reset
      └→ mcuboot starts
         └→ SYS_INIT init_load_switch (PRE_KERNEL_2, prio 80)
            ├→ gpio1.11 = HIGH  (ls_1_8 ON — needed for SPI NOR flash)
            └→ gpio0.14 = HIGH  (ls_3_3 ON — needed for status LED)
         └→ mcuboot reads/verifies image, blinks LED
         └→ mcuboot jumps to app  (GPIO state preserved: ls_1_8 HIGH, ls_3_3 HIGH)

T=?   Application kernel init  (mcuboot is gone; no reset; GPIOs intact)
      └→ POST_KERNEL prio 80: board_init.c device init
         ├→ init_pm_device(ls_1_8): gpio → INACTIVE (HIGH→LOW transient)
         ├→ init_pm_device(ls_3_3): gpio → INACTIVE (HIGH→LOW transient)
         └→ init_pm_device(ls_sd):  gpio → INACTIVE (was already off)
         Framework state: all three SUSPENDED, refcount 0
      └→ spi_nor driver init (mx25r6435f)
         pm_device_driver_init checks pm_device_is_powered(dev):
           - power-domains = <&load_switch> → parent is SUSPENDED
           - pm_device_is_powered returns FALSE
           → pm_device_init_off(dev); return 0;
           NO SPI transactions. NO infinite-loop hang.
      └→ sdhc_spi driver init (sdhc0)
         Only checks device_is_ready(spi_dev). No SPI I/O. No hang.

T=?   main() → power_manager.begin()
      ├→ Talks to BQ25120A/BQ27220 on i2c1 (permanent supply — no load switch)
      ├→ Reads reset reason, logs RESETREAS + BQ25120A fault / BQ27220 status
      ├→ Claims ZERO load switches
      ├→ check_battery() — the ONLY gate on booting
      │  └→ fail: return power_down()  (rails stay off, sys_poweroff)
      ├→ power_on = true, state_indicator.init → KTD2026::begin
      │  └→ _get(ls_3_3)  → RESUME → gpio HIGH → LED on
      ├→ Starts power_button_watch_work (see §6)
      └→ main() continues:
         ├→ sdcard_manager.init() → acquires ls_1_8 + ls_sd briefly, probes
         │    sd_state pin (card-detect needs rails up on this board), keeps
         │    rails if card present, releases otherwise
         ├→ disk_access_init("SD") — SD disk initialised while rails are up
         ├→ usbd_enable — USB MSC LUN points at "SD"
         ├→ Sensors started later (BLE client configures)
         │  └→ sensor.init → _get(ls_1_8) [+ _get(ls_3_3) for some]
         ├→ Audio started later (streaming begins)
         │  └→ dac.begin → _get(ls_1_8)
         └→ DFU (MCUmgr upload over BLE)
            └→ StateIndicator DFU hook: MGMT_EVT_OP_IMG_MGMT_DFU_STARTED
                → _get(mx25r64) → cascades via power-domains to _get(ls_1_8)
                → TURN_ON: spi_nor_configure (JEDEC ID, SFDP, mxicy config)
                → RESUME: exit DPD → flash ready
            └→ MGMT_EVT_OP_IMG_MGMT_DFU_STOPPED
                → _put(mx25r64) → SUSPEND (enter DPD)
                → cascades _put(ls_1_8) → if refcount=0, SUSPEND → gpio LOW
```

### Key configuration

- **DTS** (`boards/openearable_v2_nrf5340_cpuapp.overlay`,
  `.../openearable_v2_nrf5340_cpuapp_common.dts`):
  - `mx25r64` has `power-domains = <&load_switch>` + `zephyr,pm-device-runtime-auto`.
  - All three load-switch nodes carry `zephyr,pm-device-runtime-auto`.
  - The repo-local load-switch binding at
    `boards/teco/openearable_v2/dts/bindings/load-switch.yaml` declares
    `#power-domain-cells = <0>` so other devices can use it as their power domain.
- **Kconfig** (`prj.conf`): `CONFIG_PM_DEVICE_POWER_DOMAIN=y`.
- **`KTD2026.cpp`**: claims only `ls_3_3`.
- **`PowerManager::begin`**: claims no load switches. `check_battery()` is the
  only boot gate (see §2); reset reason is logged but not gating.
- **`SDCardManager::init`**: presence probe — briefly acquires `ls_1_8` + `ls_sd`,
  waits 1 ms, checks `sd_state` pin, keeps rails if a card is present, releases
  otherwise. Card-detect can't read the card unless rails are up (project memory:
  `project_sd_detect_needs_rails`). `acquire_ls` / `release_ls` only manage
  `ls_1_8` + `ls_sd`.
- **`main.cpp`**: calls `sdcard_manager.init()` before `disk_access_init("SD")`
  so the SD disk probe sees an already-powered card.
- **MCUmgr DFU hook** (`StateIndicator.cpp`): `MGMT_EVT_OP_IMG_MGMT_DFU_STARTED`
  calls `pm_device_runtime_get(mx25r64_dev)` and DFU_STOPPED calls the matching
  `_put`, cascading through power-domains to `ls_1_8`.

### Why the boot doesn't hang on SPI NOR

The Zephyr `spi_nor` driver's `pm_device_driver_init` checks
`pm_device_is_powered(dev)`: with `CONFIG_PM_DEVICE_POWER_DOMAIN=y` and a
`power-domains` parent that's SUSPENDED, it returns false → `pm_device_init_off`
short-circuits the init before any SPI transaction. Without the power-domain
linkage the driver would read 0xFF from the unpowered chip and spin forever in
`spi_nor_wait_until_ready` (unbounded `while (true)` in `spi_nor.c:479`
because the WIP bit looks set in garbage).

---

## 1. Zephyr PM background (one-paragraph refresher)

Zephyr separates **System PM** (CPU sleep states, set by a policy callback during the
idle thread) from **Device PM** (per-driver action callbacks: `TURN_ON`, `RESUME`,
`SUSPEND`, `TURN_OFF`). Device Runtime PM (`CONFIG_PM_DEVICE_RUNTIME`) layers a
reference counter on top: drivers call `pm_device_runtime_get()` before using a device
and `pm_device_runtime_put()` after, and the framework calls the action callback to
flip ACTIVE↔SUSPENDED automatically. Devices can be tied together via the
`power-domains` DT property, so getting a child also brings up its supply, and the
`zephyr,pm-device-runtime-auto` DT property makes the framework enable runtime PM
without an explicit `pm_device_runtime_enable()` in code.

---

## 2. What OpenEarable v2 actually does

**Configuration** (`prj.conf:125-128`):
```
CONFIG_PM=y
CONFIG_PM_DEVICE=y
CONFIG_PM_DEVICE_RUNTIME=y
CONFIG_POWEROFF=y
```
System PM is *enabled* but no PM policy hooks or sleep states are exercised — the
application never participates in system suspend. The MCU only ever runs at full
clock or fully off (`sys_poweroff()`).

**Architecture in three layers:**

1. **Application policy — `PowerManager` (`src/Battery/PowerManager.{h,cpp}`)**
   - Custom 10-state battery/charging state machine fed by BQ27220 (fuel gauge) +
     BQ25120A (charger) over I²C1, driven by two work items
     (`charge_ctrl_delayable`, `fuel_gauge_work`) and three GPIO callbacks
     (`power_good_callback`, `fuel_gauge_callback`, `battery_controller_callback`).
     Classification is factored into a pure `classify_charging()` helper (§5).
   - Owns the boot/power-on/power-off sequence: `begin()` and `power_down()`.
   - Publishes battery state on the `battery_chan` ZBUS channel.

2. **Power gating — three custom load-switch "devices"**
   `boards/teco/openearable_v2/board_init.c` defines three `DEVICE_DT_DEFINE`s
   (`ls_1_8`, `ls_3_3`, `ls_sd`) using a generic PM control callback that just
   toggles a GPIO and waits `power-delay-us`:
   ```c
   case PM_DEVICE_ACTION_SUSPEND: gpio_pin_set_dt(&data->ctrl_pin, 0); break;
   case PM_DEVICE_ACTION_RESUME : gpio_pin_set_dt(&data->ctrl_pin, 1);
                                  k_usleep(data->delay_us); break;
   ```
   The load switches use `pm_device_runtime_*` for refcounting. `ls_3_3` is the
   3.3 V LDO inside the BQ25120A (DT child of `bq25120a@6a`); `ls_1_8` and `ls_sd`
   are GPIO load switches on `gpio1.11` / `gpio1.12`. All three have
   `zephyr,pm-device-runtime-auto` so runtime PM is enabled automatically and
   `PowerManager::begin()` doesn't have to touch them — each rail comes up on
   the first `_get` and drops on the last `_put`.

3. **Application-level sensor/peripheral wrappers**
   Every sensor driver (`IMU.cpp`, `Baro.cpp`, `BoneConduction.cpp`, `PPG.cpp`,
   `Temp.cpp`), the LED driver (`KTD2026.cpp`), the audio codec (`ADAU1860.cpp`),
   and the SD card manager (`SD_Card_Manager.cpp`) call
   `pm_device_runtime_get(ls_*)` in `init()`/`begin()` and
   `pm_device_runtime_put(ls_*)` in `stop()`/`power_off()`. None of the sensors
   themselves are real Zephyr devices — they're declared as
   `compatible = "i2c-device"` and accessed through C++ wrappers, so they have no
   PM action callbacks of their own.

**Power-down sequence (`PowerManager::power_down`):**

`reboot()` and `power_down()` share a `shutdown_subsystems()` helper that:
disconnects BLE peers, stops external advertising, stops the sensor manager,
stops the BT watchdog, and ends the DAC. From there:

1. `shutdown_subsystems()` (shared with `reboot()`)
2. `led_controller.power_off()`
3. If not charging: arm BQ25120A PG as the System-OFF wake source, high-impedance the charger
4. `k_msleep(200)` to let BT disconnects propagate, then `bt_disable()` (on nRF5340
   this also forces the netcore off via `bt_hci_transport_teardown`)
5. Clear error LED
6. If charging: `sys_reboot(COLD)` and return
7. Otherwise: disable BQ25120A `EN_LS_LDO`, then `pm_device_action_run(SUSPEND)` on
   `ls_sd`, `ls_3_3`, `ls_1_8`, and `cons` (the UART console)
8. `poweroff audit` log line (rail pins, `EN_LS_LDO`, `pmic_cd`, `netcore_off`)
9. `sys_poweroff()`; fallback `k_msleep(1000); sys_reboot(COLD)`

**Sensor pattern (typical):**
```cpp
bool Sensor::init() {
    pm_device_runtime_get(ls_1_8);   // and ls_3_3 if needed
    if (!hw.begin()) { pm_device_runtime_put(ls_1_8); return false; }
    ...
}
void Sensor::stop() {
    if (!_active) return;
    _active = false; _running = false;
    k_timer_stop(...); k_work_cancel(...);
    hw.softReset()/stop()/sleepMode();   // varies per sensor; all chips are
                                         // driven into a sleep/suspend state
                                         // before the rail is released
    pm_device_runtime_put(ls_1_8);
}
```

---

## 3. Open issues

### High

#### 3.1 No mid-power "BLE advertising only" state
During normal "idle" operation (advertising or connected) the BLE controller
(network core + RF) is the dominant power consumer. There is no
advertising-only / sensors-off mid-power state — the device is "everything on"
or "off".

#### 3.2 No system PM is actually used
`CONFIG_PM=y` is set but the application never enters a low-power sleep state.
Zephyr's tickless idle reduces wakeups, but the SoC stays in CONSTLAT, the HF clock is
likely running, and the I²C peripheral blocks (see 3.3) are powered. There is no PM
policy callback, no `pm_state_force()`, no constraint API usage. The "low power" path
is jump-straight-to-`sys_poweroff()`.

#### 3.3 I²C buses are never gated
`i2c1`, `i2c2`, `i2c3` all have `status = "okay"` and define both `default` and
`sleep` pinctrl states (`*.dts:95-183`). But nothing ever drives the bus into the
sleep state — `nordic,nrf-twim` only flips pinctrl when its own PM action callback
runs, and nobody calls `pm_device_action_run` on the I²C controllers. So the TWIM
peripheral blocks are clocked continuously.

#### 3.4 No `power-domains` linkage between sensors and load switches
Sensor DT nodes (`bmp388@76`, `bma580@18`, `bmx160@68`, `mlx90632@3a`,
`maxm86161@62`) declare no `power-domains` property. Combined with the fact that
they're `compatible = "i2c-device"` and have no driver/PM callback, the load-switch
refcount and the sensor's "active" state are coordinated only by the C++ wrappers.
Easy to leak and easy to introduce ordering bugs.

### Medium

#### 3.5 `power_down` ordering: sensors are stopped before BLE is disconnected
`shutdown_subsystems()` runs disconnect → ext-adv-stop →
`stop_sensor_manager()` → stop-watchdog → `dac.end()` without waiting for the
async disconnect events. The 200 ms sleep before `bt_disable()` is the
existing buffer; re-ordering so audio/sensor teardown happens after BLE
teardown is a latent improvement.

#### 3.6 `DEBOUNCE_POWER_MS = K_MSEC(1000)` is unused at the place that needs it
`PowerManager.h` defines it, but `power_good_callback` schedules
`power_down_work` with `K_NO_WAIT`. So unplugging USB while powered-off triggers
an immediate `power_down`, with no debounce. Not catastrophic but the named
constant is misleading.

#### 3.7 No fault-recovery path for non-undervoltage faults
In `classify_charging`, only BAT_UVLO (bit 5) has a recovery transition (→
PRECHARGING when power is connected and current is flowing). OVP / TS / input-UV
faults fall through to `FAULT` and re-enter it on the next poll with no exit.
The work handler does re-run `setup_pmic()` on a TS fault, which might un-stick
the TS path, but everything else requires a user power-cycle.

### Low / code-quality

#### 3.8 Synchronous `pm_device_runtime_put` everywhere
Some shutdown paths (especially PPG, which talks to the chip) could use
`pm_device_runtime_put_async` so the suspend completes off the calling thread.
Probably doesn't matter for the rare power-down case but might smooth sensor
reconfiguration.

#### 3.9 Sensors aren't real Zephyr devices
Compatible = `"i2c-device"` means: no `init_fn`, no `pm_action_cb`, no
`PM_DEVICE_DT_INST_DEFINE`. We're paying the price (manual coordination, no
`power-domains`, no auto-runtime, can't piggy-back on system PM) without the
benefit (smaller code). This is a structural choice — converting to real
drivers is a larger refactor — but it's the root cause of 3.4, 3.8, and would
simplify most of the manual wrapper refcounting.

---

## 4. Next steps

- **Root-cause USB MSC mid-run failure** — the `reboot()` in §7 is a pragmatic
  fix; the underlying race between running system state and USB MSC attaching
  to the SD disk is not understood (see §8).
- **Consider converting sensors to real Zephyr drivers** with PM action
  callbacks and `power-domains` (§3.9). Big refactor; would eliminate several
  categories of manual refcount bugs.
- **Gate the I²C controllers** (§3.3) when no sensor/LED/codec is active.
- **Add a BLE-only idle state** (§3.1) to cut average draw while advertising.

---

## 5. Charging code: structure

The charging-related code lives in `src/Battery/PowerManager.{h,cpp}` and
`src/Battery/BQ25120a.{h,cpp}`. A few patterns are load-bearing; grepping for
these names will orient most changes.

### `BQ25120a::ChargePhase` — typed enum for the 2-bit charging-phase field

`enum class ChargePhase : uint8_t { Discharge, Charging, Done, Fault }`. The
register-0x00 `>> 6` shift lives in a single accessor,
`BQ25120a::read_charge_phase()`. Everything that cares about phase (the work
handler's big switch, `get_battery_status` GATT encoding, `cmd_battery_info`)
uses the enum. One diagnostic (`cmd_sensor_diag`) still prints the raw byte
inline — deliberate, because it also dumps the full register for debugging.

### `BQ25120a::ActiveScope` — internal RAII guard for I2C access

I2C reads/writes to the BQ25120A require the chip to be out of high-impedance
mode (CD pin low) while the transaction happens. `ActiveScope` is a private,
refcounted RAII guard with a `k_mutex`: the 0→1 transition calls
`exit_high_impedance()`, the 1→0 transition calls `enter_high_impedance()`, and
nested/concurrent scopes compose correctly.

**Every public I2C method self-wraps in an `ActiveScope`.** Callers just call
`battery_controller.read_charge_phase()` and the hi-Z dance happens inside.
Refcounting means a public method that calls another public method
(`setup()` → the various `write_*` helpers) only toggles the CD pin on the
outermost entry. `enter_high_impedance()` / `exit_high_impedance()` are also
private; `ActiveScope` is the only thing that calls them.

The single exception is `enter_ship_mode()`, which drives CD high directly as
part of the fire-and-die ship-mode entry sequence (§9.3.1.1 of the datasheet) —
the chip is going to cut SYS in a few ms anyway, so there's nothing to restore.

### `charger_init_pending` — ISR → work-queue handoff for charger re-init

`power_good_callback` runs in GPIO ISR context on USB plug-in/plug-out. I2C can't
run there, so it can't call `setup_pmic()` directly. Instead, on plug-in it sets
`charger_init_pending = true` and schedules the `charge_ctrl_delayable` work
item. `charge_task` (work context) sees the flag, clears it, runs `setup_pmic()`
+ `enable_charge()`, then submits `fuel_gauge_work`. Boot doesn't need to seed
the flag — `begin()` calls `setup_pmic()` once up front.

### `PowerManager::setup_pmic()` — single entry point for charger configuration

All charger-configure paths (`begin()`, `charge_task()` on plug-in,
`fuel_gauge_work_handler` on TS-fault recovery, the `sensor_diag` shell command)
route through `setup_pmic()`. `_battery_settings` is referenced in exactly one
place outside the member declaration. `BQ25120a::setup()` self-wraps in
`ActiveScope`, so `setup_pmic()` just calls it plainly.

### `BQ25120a::fault_bits[]` — single source of truth for fault-register labels

A constexpr array of `{mask, name}` pairs for register 0x01 (faults). The work
handler's `LOG_WRN` loop and the shell diag's `shell_print` loop both iterate
this array. Labels follow BQ25120A datasheet Table 13 exactly:
B4=BAT_OCP (cleared on read), B5=BAT_UVLO (persistent), B6=VIN_UV (cleared on
read), B7=VIN_OV (persistent). Heads-up: upstream commit `91d4081` swapped the
B4/B7 names in 2025-05; the swap is fixed here but still lives in upstream.

### `classify_charging()` — pure state classifier

`fuel_gauge_work_handler` separates three concerns:

1. **Read** hardware state into `struct charging_snapshot` (phase, `bat_status`,
   `gauge_status`, fault byte, voltage, current, target current, `power_connected`).
2. **Classify** via file-static pure `classify_charging(snapshot, cfg) → enum
   charging_state`. Zero I/O, no logging, no side effects — unit-testable in
   isolation (no tests written yet).
3. **Side effects** on the result: logging by phase, TS-fault recovery
   (`setup_pmic()`), zbus publish, polling-interval adjustment.

The undervoltage-recovery edge case (BAT_UVLO + power connected + current
flowing → PRECHARGING) is a single branch inside the classifier rather than a
side-effecting hairball in the switch.

### `cmd_battery_info` vs. work-handler LOG_DBG dump

Both print overlapping battery snapshots in slightly different formats. Not
consolidated; a shared formatter taking a `printk`/`shell_print` callback is the
natural next step if fields need to be added. `cmd_battery_info` also prints
the four load-switch pin states (`ls_1_8`, `ls_3_3`, `ls_sd`, PPG LDO) plus
the BQ25120A `EN_LS_LDO` bit.

---

## 6. Power-button state machine

`PowerManager::begin()` no longer gates boot on BQ25120A button/wake register
bits. The only boot gate is `check_battery()`. The button state machine is a
post-boot polling loop:

```
member: first_release_seen = false      // set once the button is first released

begin() schedules power_button_watch_work with K_MSEC(100)
power_button_watch_handler (k_work_delayable):
    if earable_btn.getState() == BUTTON_RELEASED:
        first_release_seen = true        // consume the power-on press
        return                           // stops polling
    else:
        reschedule(K_MSEC(100))

battery_controller_work_handler (BQ25120A INT → work):
    read button register
    if state.wake_2 && first_release_seen:
        power_on = !power_on
        if (!power_on) power_down()
```

Result: the user can hold the button until they see the desired state change,
release, and that's it. The toggle on WAKE_2 fires immediately (state change is
visible while the button is still held); the `first_release_seen` flag makes
sure the initial power-on press isn't mis-read as an immediate power-off
toggle.

---

## 7. Reset-reason handling and USB-plug reboot

`begin()` reads `RESETREAS` for **diagnostics only** — it logs the bits and
records `oe_boot_state.timer_reset` on `RESETPIN` wakes (the BQ25120A timer
reset vs a programmer nRESET is useful downstream) — but the reset reason
does not gate boot.

USB-plug while the device is already running is handled by triggering a
graceful reboot instead of trying to bring USB MSC up against running app
state:

```cpp
// power_good_callback (GPIO ISR on BQ25120A PG):
if (power_good && power_manager.power_on) {
    k_work_submit(&usb_plug_reboot_work);   // handler calls power_manager.reboot()
}
```

This ensures the device always comes up via the cold-boot-with-USB path, which
is known to cleanly enumerate the SD card over MSC. On VBUS-wake from
System OFF, the new boot just proceeds normally (no special handling needed).

---

## 8. Known limitation: USB MSC while device is running

`sd_bench` (firmware-side disk access) works reliably with the rails up, but
host-initiated READ(10) requests over USB MSC time out with `DID_ERROR
cmd_age=3s` when USB is plugged into a running device. INQUIRY / READ_CAPACITY
(served from cached `sd_card` fields, no SPI transaction) respond fine; the
failure is specifically on data reads. Root cause not nailed down — suspected
thread/priority contention between the USBD stack and whatever the app is
doing when the host tries to read. Worked around by rebooting on USB plug (§7),
which puts us into the cold-boot path that works end-to-end.

---

## 9. Files referenced

| File | Notes |
|------|-------|
| `src/Battery/PowerManager.h` | Class definition; power-button / USB-plug work items; `first_release_seen` flag |
| `src/Battery/PowerManager.cpp` | `begin`, `power_down`/`reboot` sharing `shutdown_subsystems`, `power_button_watch_handler`, `usb_plug_reboot_handler`, fault-log with CTRL/FAULT/TS_FAULT raw bytes |
| `src/Battery/BQ25120a.{cpp,h}` | Charger; HiZ + `ActiveScope`, `ChargePhase`, `fault_bits`. Push-button Control register (0x08) is left at reset defaults (MRRESET=9s, MRWAKE2=1500ms) — firmware never writes it. |
| `src/Battery/BQ27220.{cpp,h}` | Fuel gauge; wakeup interrupts |
| `boards/teco/openearable_v2/board_init.c` | Custom load-switch PM devices |
| `boards/teco/openearable_v2/openearable_v2_nrf5340_cpuapp_common.dts` | I²C/load-switch nodes; all three load switches carry `zephyr,pm-device-runtime-auto` |
| `boards/openearable_v2_nrf5340_cpuapp.overlay` | `mx25r64` carries `power-domains = <&load_switch>` + `zephyr,pm-device-runtime-auto` |
| `boards/teco/openearable_v2/dts/bindings/load-switch.yaml` | Repo-local load-switch binding; `#power-domain-cells = <0>` |
| `src/SensorManager/IMU.cpp` | `imu.stop()` parks accel+gyro in BMI160 suspend mode before `_put(ls_1_8)` |
| `src/SensorManager/Baro.cpp` | `bmp.sleep()` before pm_device_runtime_put |
| `src/SensorManager/BoneConduction.cpp` | OK; calls bma580.stop() |
| `src/SensorManager/PPG.cpp` | GPIO LDO released in `stop()` and error path |
| `src/SensorManager/Temp.cpp` | OK; calls sleepMode() |
| `src/SensorManager/SensorManager.cpp` | `stop_sensor_manager` calls every `.stop()` |
| `src/drivers/LED_Controller/KTD2026.cpp` | claims only `ls_3_3` |
| `src/drivers/ADAU1860.cpp` | Audio codec PM |
| `src/SD_Card/SD_Card_Manager/SD_Card_Manager.cpp` | Presence probe in `init()`; `acquire_ls`/`release_ls` manage only `ls_1_8` + `ls_sd` |
| `src/main.cpp` | `sdcard_manager.init()` before `disk_access_init("SD")` |
| `src/utils/StateIndicator.cpp` | MCUmgr DFU hook: `get`/`put` on `mx25r64` |
| `prj.conf` | CONFIG_PM*, CONFIG_PM_DEVICE_POWER_DOMAIN, CONFIG_POWEROFF |
| `SYSTEM_DESCRIPTION.md` | I²C topology, sensor/codec overview |
