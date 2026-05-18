# OpenEarable v2 Firmware — Code Quality Assessment

## Summary

The project-specific code is functional but still has meaningful quality issues across thread safety, error handling, memory safety, and architectural cohesion. Several concerns from the prior pass have been addressed by recent work (major power refactor, sensor_sink introduction, Bosch BMI160 replacing the DFRobot BMX160 driver, card-detect rails fix), but a core set of thread-safety and memory-safety issues remains. Below is a prioritized breakdown reflecting the current state of the tree.

---

## Recent Progress (context for this revision)

- **Major power refactor** — `PowerManager.cpp` shrank substantially (~880 lines now); control flow around mounting, load switches, and charging is cleaner. The BQ25120a interrupt handling has been documented.
- **sensor_sink** — new `include/sensor_sink.h` + `src/SensorManager/sensor_sink.cpp` centralize how sensors deliver data to both BLE and SD sinks, removing the prior `audio_datapath` wrapper and trimming `SensorManager` by ~70 lines.
- **BMX160 → Bosch BMI160** — the DFRobot driver was replaced with the official Bosch reference driver plus a thin `BMX160_Sensor` wrapper. BMX160 suspend mode is now implemented.
- **SD card-detect** — pre-rails card detection issue understood; `sd_state` GPIO requires `ls_1_8` + `ls_sd` to be powered before it can read.
- **Duplicate include fixed** — `sensor_service.h` is now included only once in `SensorManager.cpp`.
- **Dead placeholders cleaned** — e.g. `Baro.cpp`'s commented-out `bmp.start()` and the "pulse oximeter" copy-paste in `BoneConduction.cpp::reset()` are gone.
- **Header guard fixed** — `led_service.h` no longer uses `AUDIO_PLAYER_H`.

---

## Critical Issues

### 1. Thread Safety — Pervasive Race Conditions

Nearly every module still has shared mutable state accessed from ISR callbacks, work queues, and threads without synchronization:

- **`Wire.cpp`** — mutex locking is **still commented out** for the primary I2C transaction paths (lines 81, 93, 98, 104, 107, 113). A second lock pair at lines 156/160 *is* active, so protection is inconsistent within the same file.
- **`SensorManager.cpp`** — `active_sensors` counter is modified from multiple contexts. The recent sensor_sink refactor simplified data delivery but did not introduce an atomic or mutex around this counter.
- **`PowerManager.cpp`** — `charging_disabled`, `last_charging_state`, and related flags are touched from ISR callbacks without atomics or mutexes. The ZBUS adoption for some paths reduces exposure but does not eliminate it.
- **`time_sync.c`** — `time_offset_us` is written by BLE callback (~line 144) and read by `get_current_time_us()` (~lines 160–173); no synchronization.
- **Sensor classes** — `_active`, `_running`, `_ble_stream` booleans are still accessed from timer interrupts and main thread without atomics.
- **Sensors still set `_active = false` *before* calling `k_timer_stop()`** — confirmed in `Baro.cpp` (line 112 vs 121) and `IMU.cpp` (line 152 vs 165). The timer can fire on a deactivated sensor. Stop the timer first.
- **`sensor_service.c`** — `connection_complete` modified in ISR context (~line 53) but read in thread context (~line 206) without synchronization.
- **`led.c`** — `on_phase` static variable in `led_blink_work_handler` and `led_units` array modified without synchronization.
- **`Button.cpp`** — `button_cb_data` is static (line 13) but written per-instance; cross-instance ISR interference if a second button is ever registered.

### 2. Silent Error Swallowing

Return codes are logged but not propagated; the system continues in degraded states without callers knowing:

- **`BQ25120a.cpp` / `BQ27220.cpp`** — I2C read failures still return stale data (e.g. `voltage()` returns 0.0 on I2C failure, indistinguishable from a dead battery).
- **`BQ25120a.cpp` (`readReg`)** — returns success even when `i2c_burst_read()` fails.
- **`main.cpp:~87`** — USB init failure still returns 0 (success).
- **`PowerManager.cpp`** — multiple `pm_device_action_run()` calls have ignored return values (~lines 580–582, 607, 835, 839).
- **`sensor_service.c`** — still uses a busy-wait loop `while(notify_count >= MAX_NOTIFIES_IN_FLIGHT) { k_yield(); }` instead of a semaphore.
- **`PowerManager.cpp`** — load switch enable errors are logged as warnings and execution continues regardless.

### 3. Memory Safety

- **`uicr.c`** — **known memory collision**: `MEM_ADDR_UICR_SNR` is 8 bytes wide but `MEM_ADDR_UICR_CH` is placed only 4 bytes after it, causing overlap. Comment acknowledging this is still present (~line 80).
- **`SensorScheme.cpp:351`** — `sensorSchemesMap[id]` still uses `operator[]`, which **creates** entries for missing keys. Should use `.find()`.
- **`BQ27220.cpp:343`** — `k_malloc()` with no null check.
- **`BQ27220.cpp:315`** — VLA `uint8_t buf[len]` is non-standard C++ and allocates an unknown size on the stack.
- **`streamctrl.c:~575`** — `memcpy(chip_id, data, sizeof(chip_id))` still assumes `data` has at least 8 bytes, no bounds check.
- **`Equalizer.cpp:~39`** — `data[n]` loop increments by 2 with no odd-length guard.
- **`sensor_service.c`** — `active_sensor_configs` allocated with `k_malloc()`/`k_realloc()` (~lines 237–278) but never freed.
- **`Wire.cpp:~118–124`** — TX buffer writes with no overflow check; silent truncation.

### 4. Unsafe Enum / Type Casts

- **`SensorManager.cpp:~134`** — raw cast `(enum sensor_id) config.sensorId` from untrusted BLE data, no bounds check.
- **`Button.cpp:33`** — `gpio_pin_get_dt()` return (int) cast directly to `button_action` enum.
- **`time_sync.c:~168`** — integer overflow check happens **after** the overflow.

> The `hw_codec_adau1860.cpp` `#ifdef`-split if/else chain flagged previously was not found in the current code and is treated as resolved pending final verification.

---

## Architectural Issues

### 5. PowerManager is still doing a lot

Even after the refactor, `PowerManager.cpp` (~880 lines) continues to manage charging, fuel-gauge reads, GPIO callbacks, LED signalling, BLE disconnect, reboot logic, battery reporting, and sleep/wake. It is noticeably better organized than before, but the single-file god-object shape remains. A further split into (a) charging manager, (b) battery monitor, and (c) power/state controller would reduce coupling.

### 6. No Sensor Registry — Hardcoded Sensor Lists

Confirmed still present at `SensorManager.cpp:88–93`:
```cpp
Baro::sensor.stop();
IMU::sensor.stop();
PPG::sensor.stop();
Temp::sensor.stop();
BoneConduction::sensor.stop();
Microphone::sensor.stop();
```
Adding a sensor still requires touching multiple files. `sensor_sink` helped on the data-plane side but did not replace the control-plane list. A registry pattern (or a vector of `EdgeMLSensor*`) would eliminate this.

### 7. Hardcoded GATT Attribute Indices

Throughout the BLE services: `sensor_service.attrs[4]`, `button_service.attrs[2]`, `sensor_service.attrs[7]`, `parseInfo_service.attrs[5]`. If the `BT_GATT_SERVICE_DEFINE` macro structure changes, these silently break.

### 8. C / C++ Style Mixing

The codebase still mixes C and C++ patterns — `memcpy` on C++ objects (`StateIndicator.cpp:~152`), C-style casts, raw pointers with no ownership model, `k_malloc` without RAII. The `Wire` class wraps Zephyr's C I2C API in an Arduino-style C++ interface but leaves some callbacks unimplemented.

---

## Code Quality Issues

### 9. Magic Numbers

- **`PowerManager.cpp:~177`** — `0.8 * target_current - 2 * power_manager._battery_settings.i_term` (why 0.8? why 2x?).
- **`PowerManager.h:~64–76`** — battery settings hardcoded, still with German comment `Spannungen [V]`.
- **`BQ27220.cpp:~447–456`** — raw RAM addresses (`0x9240`, `0x9243`, `0x9282`) with no named constants.
- **`DefaultSensors.h`** — `maxBleFrequencyIndex` values (2, 1, 12, 7, 17, 6) with no explanation.
- **`SensorScheme.cpp:100`** — `&parseInfo_service.attrs[5]` for notification attribute.
- **`led.c:21`** — `BLINK_FREQ_MS 1000` used as `K_MSEC(BLINK_FREQ_MS / 2)` without explaining why half.
- **`PPG.cpp:~26–28`** — GPIO pin 6 hardcoded (now in a named static, but still not from device tree).

### 10. Remaining Dead / Commented-Out Code

- **`Wire.cpp`** — commented-out `k_mutex_lock/unlock` lines as noted above.
- **`PowerManager.cpp`** — still carries ~70 lines of commented diagnostic/debug code despite recent cleanups.
- **`button_pressed.cpp`** — large block of commented-out multi-button handling.
- **`Button.cpp:80–83`** — `gpio_remove_callback()` commented out, potentially leaving dangling ISR callbacks after `end()`.
- **`battery_service.cpp:~47–48, 141–144`** — commented-out characteristics still present.

### 11. TODOs in Production Paths

- **`streamctrl.c:~606–607`** — `"TODO: check if the device wants to pair"`.
- **`PowerManager.cpp`** — multiple remaining `TODO` / `check power on condition` / `check states of load switch` comments.
- **`time_sync.c:~151`** — `can_sync_time()` always returns true with TODO.
- **`uicr.h:12`** — `"TODO: Discuss better alternative for UICR storage"`.
- **`hw_codec_adau1860.cpp:~64`** — `"TODO: make writing to bank work"` with 200ms sleep workaround.
- **`BQ25120a.cpp:~65`** — `"TODO: check value"` for an unjustified 1ms sleep.
- **`SensorManager.cpp:~242`** — `"TODO: if (ble_sensors.empty()) ..."`.

### 12. Additional Issues

- **`ParseType.h`** — `parseTypeSizes[]` array still depends on exact enum ordering with no `static_assert`. Adding an enum entry will silently break the mapping.
- **`SensorScheme.cpp:~203–206`** — `strlen()` called multiple times on the same string instead of caching the result.
- **`SensorManager.cpp:~199–202`** — `active_sensors` counter still goes negative under some error paths (symptom of a deeper double-stop bug).
- **`PowerManager.cpp:~356–359`** — `while(!power_on && battery_controller.power_connected())` still has no timeout.
- **`PPG.cpp:~84`** — floating-point comparison without epsilon.
- **`uicr.c:~38–40`** — NVM write verification reads immediately after async write, may see stale value.
- **`uicr.c:~96–100`** — `uicr_hw_revision_get()` takes `char*` with hardcoded `snprintf` size 16, no way for caller to specify buffer size.
- **`SDLogger.h`** — `is_open` is a public bool accessed from multiple files without synchronization; should be private with atomic access.

---

## Resolved Since Last Revision

- `SensorManager.cpp` double-include of `sensor_service.h` — **fixed**.
- `led_service.h` `AUDIO_PLAYER_H` header guard — **fixed** (now `OPEN_EARABLE_LED_SERVICE_H`).
- `Baro.cpp` commented-out `bmp.start()` — **removed**.
- `BoneConduction.cpp::reset()` "pulse oximeter" copy-paste — **removed**.
- BMX160 suspend mode was missing — **implemented** as part of the BMI160 port.
- `EdgeMLSensor.h` commented-out macro block — **removed** during the sensor_sink refactor.
- `SensorScheme.cpp` inconsistent `-1` / `-ENOMEM` error codes — **normalized** to `-1`.
- `aquire()` misspelling on `MbedI2C` / `TWIM` and all I2C callers — **renamed** to `acquire()`.
- `audio_datapath_aquire` and `SDCardManager::aquire_ls` / `ls_aquired` — **renamed** to `acquire` variants.
- `DefaultSensors.h` `microComponenents` typo — **renamed** to `microComponents`.
- `DefaultSensors.h` microphone channel name casing (`"INNER"` vs `"Outer"`) — **normalized** to uppercase.
- `PowerManager.cpp:365` `abs()` on a float — **replaced** with `fabsf()` (added `<math.h>`).

---

## What's Done Well

- **ZBUS adoption** for inter-module messaging remains a good architectural choice and has been extended in the power refactor.
- **sensor_sink** is a nice new seam: sensors no longer need to know about BLE vs SD destinations, which made the BMI160 swap and the audio_datapath removal much cleaner.
- **Device tree integration** for hardware configuration is proper Zephyr practice, and the load-switch binding addition is a good example.
- **Sensor abstraction** via `EdgeMlSensor` base class is a reasonable pattern (though underutilized by the SensorManager control path).
- **Build configuration** split (debug/FOTA/release) is clean.
- **SD card logging** with ring buffers and background flush is a reasonable design; robust mount/unmount handling was added recently.
- **Board definition structure** follows Zephyr conventions well.

---

## Recommended Fix Priority

1. **Fix the UICR memory collision** — still a data corruption bug in production.
2. **Re-enable Wire mutex locking** — concurrent I2C access can still corrupt transactions (protection is already inconsistent within the same file).
3. **Stop timer before clearing `_active`** in all sensor `stop()` methods — confirmed still backwards in `Baro.cpp` and `IMU.cpp`.
4. **Add atomic/mutex protection** to shared state in sensors, PowerManager, and `time_sync`.
5. **Propagate errors** from I2C reads in BQ25120a/BQ27220 instead of returning stale data.
6. **Replace `sensorSchemesMap[id]` with `.find()`** to prevent silent map entry creation.
7. **Null-check `k_malloc` results** in BQ27220 and fix the VLA on stack.
8. **Free `active_sensor_configs`** in `sensor_service.c` or switch to a static buffer.
9. **Fix the post-facto overflow check in `time_sync.c`**.
10. **Extract magic numbers** into named constants, especially battery parameters and register addresses.
11. **Introduce a sensor registry** to replace the hardcoded stop/start list in `SensorManager.cpp`.
12. **Split PowerManager** further into focused classes with single responsibilities.
13. **Remove remaining dead/commented code** and resolve or delete stale TODOs.
