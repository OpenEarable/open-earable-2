#include "PowerManager.h"

#include "macros_common.h"

#include <math.h>
#include <stdio.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/shell/shell.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include <zephyr/zbus/zbus.h>

#ifdef CONFIG_BOOTLOADER_MCUBOOT
#include <zephyr/dfu/mcuboot.h>
#endif

#include <hal/nrf_ficr.h>
#include <hal/nrf_gpio.h>

#include "../drivers/LED_Controller/KTD2026.h"
#include "../drivers/ADAU1860.h"
#include "../buttons/Button.h"
#include "../SensorManager/SensorManager.h"

#include "../utils/StateIndicator.h"

#include "bt_mgmt.h"
#include "bt_mgmt_ctlr_cfg_internal.h"

#include <zephyr/logging/log_ctrl.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(power_manager, LOG_LEVEL_DBG);

K_WORK_DELAYABLE_DEFINE(PowerManager::charge_ctrl_delayable, PowerManager::charge_ctrl_work_handler);
K_WORK_DELAYABLE_DEFINE(PowerManager::power_down_work, PowerManager::power_down_work_handler);
K_WORK_DELAYABLE_DEFINE(PowerManager::power_button_watch_work, PowerManager::power_button_watch_handler);

K_WORK_DEFINE(PowerManager::fuel_gauge_work, PowerManager::fuel_gauge_work_handler);
K_WORK_DEFINE(PowerManager::battery_controller_work, PowerManager::battery_controller_work_handler);
K_WORK_DEFINE(PowerManager::usb_plug_reboot_work, PowerManager::usb_plug_reboot_handler);

ZBUS_CHAN_DEFINE(battery_chan, struct battery_data, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0));

static struct battery_data msg;

void PowerManager::fuel_gauge_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    LOG_DBG("Fuel Gauge GPOUT Interrupt");
    k_work_submit(&fuel_gauge_work);
}

void PowerManager::battery_controller_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_work_submit(&battery_controller_work);
}

void PowerManager::power_good_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    bool power_good = battery_controller.power_connected();

    k_work_submit(&fuel_gauge_work);

    if (power_good) {
        power_manager.charger_init_pending = true;
        k_work_schedule(&charge_ctrl_delayable, K_NO_WAIT);
        // USB MSC can't come up cleanly against a running system — the SD /
        // USB stacks race with whatever the app is doing. Take the reliable
        // path: reboot so we come back up through the cold-boot-with-USB
        // path, which is known to work end-to-end.
        if (power_manager.power_on) k_work_submit(&usb_plug_reboot_work);
    } else {
        k_work_cancel_delayable(&charge_ctrl_delayable);
        if (!power_manager.power_on) k_work_reschedule(&power_manager.power_down_work, K_NO_WAIT);
    }
}

void PowerManager::usb_plug_reboot_handler(struct k_work * work) {
    power_manager.reboot();
}

void PowerManager::power_down_work_handler(struct k_work * work) {
	power_manager.power_down();
}

void PowerManager::charge_ctrl_work_handler(struct k_work * work) {
	power_manager.charge_task();
    // Schedule next execution
    k_work_schedule(&charge_ctrl_delayable, power_manager.chrg_interval);
}

void PowerManager::battery_controller_work_handler(struct k_work * work) {
    button_state state = battery_controller.read_button_state();

    // WAKE event = long-press threshold reached. Toggle power immediately so
    // the user sees the state change while the button is still held. Events
    // that latched from the power-on press itself are suppressed until the
    // user has released the button at least once (first_release_seen).
    if (state.wake_2 && power_manager.first_release_seen) {
        power_manager.power_on = !power_manager.power_on;
        if (!power_manager.power_on) power_manager.power_down();
    }
}

void PowerManager::power_button_watch_handler(struct k_work * work) {
    if (earable_btn.getState() == BUTTON_RELEASED) {
        power_manager.first_release_seen = true;
        return;
    }
    k_work_reschedule(&power_button_watch_work, K_MSEC(100));
}

struct charging_snapshot {
    BQ25120a::ChargePhase phase;
    bat_status bat;
    gauge_status gs;
    uint8_t fault;
    float voltage_v;
    float current_ma;
    float target_current_ma;
    bool power_connected;
};

// Pure classifier: hardware snapshot + config → application-level charging_state.
// No I2C, no logging, no side effects. Unit-testable by construction.
static enum charging_state classify_charging(const charging_snapshot &s,
                                             const battery_settings &cfg) {
    switch (s.phase) {
        case BQ25120a::ChargePhase::Discharge:
            if (s.gs.edv1) return BATTERY_CRITICAL;
#ifdef CONFIG_BATTERY_ENABLE_LOW_STATE
            if (s.gs.edv2) return BATTERY_LOW;
#endif
            return DISCHARGING;

        case BQ25120a::ChargePhase::Charging:
            if (s.bat.SYSDWN) return PRECHARGING;
            if (s.current_ma > 0.8f * s.target_current_ma - 2.0f * cfg.i_term) {
                return CHARGING;
            }
            if (s.voltage_v > cfg.u_term - 0.02f) {
#ifdef CONFIG_BATTERY_ENABLE_TRICKLE_CHARGE
                return TRICKLE_CHARGING;
#else
                return CHARGING;
#endif
            }
            return POWER_CONNECTED;

        case BQ25120a::ChargePhase::Done:
            return FULLY_CHARGED;

        case BQ25120a::ChargePhase::Fault:
            // Undervoltage fault with power connected + current flowing = recovery.
            if ((s.fault & (1 << 5)) && s.power_connected &&
                s.current_ma > 0.5f * cfg.i_term) {
                return PRECHARGING;
            }
            return FAULT;
    }
    return DISCHARGING;
}

void PowerManager::fuel_gauge_work_handler(struct k_work * work) {
    int ret;
    battery_level_status status;

    msg.battery_level = fuel_gauge.state_of_charge();

    power_manager.get_battery_status(status);

    charging_snapshot snap;
    snap.bat = fuel_gauge.battery_status();
    snap.gs = fuel_gauge.gauging_state();
    snap.voltage_v = fuel_gauge.voltage();
    snap.current_ma = fuel_gauge.current();
    snap.target_current_ma = fuel_gauge.charge_current();

    if (power_manager.power_on && snap.bat.SYSDWN) {
        LOG_WRN("Battery reached system down voltage.");
        power_manager.power_down();
        return;
    }

    if (snap.bat.CHGINH) {
        power_manager.charging_disabled = true;
        battery_controller.disable_charge();
    } else if (power_manager.charging_disabled) {
        battery_controller.enable_charge();
    }

    uint8_t ts_fault = 0;
    uint8_t ctrl_raw = battery_controller.read_charging_state();
    snap.phase = static_cast<BQ25120a::ChargePhase>(ctrl_raw >> 6);
    snap.power_connected = battery_controller.power_connected();
    if (snap.phase == BQ25120a::ChargePhase::Fault) {
        snap.fault = battery_controller.read_fault();
        ts_fault = battery_controller.read_ts_fault();
    } else {
        snap.fault = 0;
    }

    msg.charging_state = classify_charging(snap, power_manager._battery_settings);

    switch (snap.phase) {
        case BQ25120a::ChargePhase::Discharge:
            LOG_INF("charging state: discharge");
            break;
        case BQ25120a::ChargePhase::Charging:
            LOG_INF("charging state: charging");
            LOG_DBG("Voltage: %.3f V", snap.voltage_v);
            LOG_DBG("Charging current: %.3f mA", snap.current_ma);
            LOG_DBG("Target current: %.3f mA", snap.target_current_ma);
            LOG_DBG("State of charge: %.3f %%", fuel_gauge.state_of_charge());
            break;
        case BQ25120a::ChargePhase::Done:
            LOG_INF("charging state: done");
            break;
        case BQ25120a::ChargePhase::Fault:
            LOG_WRN("charging state: fault (CTRL=0x%02x FAULT=0x%02x TS_FAULT=0x%02x)",
                    ctrl_raw, snap.fault, ts_fault);
            for (const auto &b : BQ25120a::fault_bits) {
                if (snap.fault & b.mask) LOG_WRN("%s", b.name);
            }
            if (snap.fault & (1 << 5)) {
                LOG_WRN("Battery voltage at under-voltage fault: %.3f V", snap.voltage_v);
            }
            if ((ts_fault >> 5) & 0x7) {
                LOG_WRN("TS_ENABLED: %i, TS FAULT: %i", ts_fault >> 7, (ts_fault >> 5) & 0x3);
                power_manager.setup_pmic();
            }
            LOG_DBG("------------------ Battery Info ------------------");
            LOG_DBG("Battery Status:");
            LOG_DBG("  Present: %d, Full Charge: %d, Full Discharge: %d",
                    snap.bat.BATTPRES, snap.bat.FC, snap.bat.FD);
            LOG_DBG("Basic Measurements:");
            LOG_DBG("  Voltage: %.3f V", snap.voltage_v);
            LOG_DBG("  Current: %.3f mA", snap.current_ma);
            break;
    }

    // Adjust interval based on state
    if (msg.charging_state == FAULT || msg.charging_state == POWER_CONNECTED) {
        power_manager.chrg_interval = K_SECONDS(CONFIG_BATTERY_CHARGE_CONTROLLER_FAST_INTERVAL_SECONDS);
    } else {
        power_manager.chrg_interval = K_SECONDS(CONFIG_BATTERY_CHARGE_CONTROLLER_NORMAL_INTERVAL_SECONDS);
    }

    ret = zbus_chan_pub(&battery_chan, &msg, K_FOREVER);
    if (ret) {
        LOG_WRN("power manager msg queue full");
    }
}

int PowerManager::begin() {
    earable_state oe_state;

    oe_state.charging_state = DISCHARGING;
    oe_state.pairing_state = PAIRED;

    battery_controller.begin();
    fuel_gauge.begin();
    earable_btn.begin();

    uint8_t bat_state = battery_controller.read_charging_state();

    // get reset reason
    uint32_t reset_reas = NRF_RESET->RESETREAS;

    // Boot diagnostics: decode RESETREAS and snapshot PMIC fault / fuel-gauge
    // status so spurious wakes from System OFF can be traced to a specific
    // line (BQ25120A PG/INT or BQ27220 GPOUT).
    {
        uint8_t bq_fault    = battery_controller.read_fault();
        uint8_t bq_ts_fault = battery_controller.read_ts_fault();
        bat_status fg       = fuel_gauge.battery_status();
        LOG_WRN("boot: RESETREAS=0x%08x [%s%s%s%s%s%s%s%s%s%s%s]",
                reset_reas,
                (reset_reas & RESET_RESETREAS_RESETPIN_Msk) ? "PIN "  : "",
                (reset_reas & RESET_RESETREAS_DOG0_Msk)     ? "DOG0 " : "",
                (reset_reas & RESET_RESETREAS_CTRLAP_Msk)   ? "CAP "  : "",
                (reset_reas & RESET_RESETREAS_SREQ_Msk)     ? "SREQ " : "",
                (reset_reas & RESET_RESETREAS_LOCKUP_Msk)   ? "LOCK " : "",
                (reset_reas & RESET_RESETREAS_OFF_Msk)      ? "OFF "  : "",
                (reset_reas & RESET_RESETREAS_LPCOMP_Msk)   ? "LPC "  : "",
                (reset_reas & RESET_RESETREAS_DIF_Msk)      ? "DIF "  : "",
                (reset_reas & RESET_RESETREAS_NFC_Msk)      ? "NFC "  : "",
                (reset_reas & RESET_RESETREAS_DOG1_Msk)     ? "DOG1 " : "",
                (reset_reas & RESET_RESETREAS_VBUS_Msk)     ? "VBUS " : "");
        LOG_WRN("boot: BQ25120A fault=0x%02x ts_fault=0x%02x  "
                "BQ27220 DSG=%u SYSDWN=%u BATTPRES=%u FC=%u FD=%u "
                "OTC=%u OTD=%u TCA=%u TDA=%u",
                bq_fault, bq_ts_fault,
                fg.DSG, fg.SYSDWN, fg.BATTPRES, fg.FC, fg.FD,
                fg.OTC, fg.OTD, fg.TCA, fg.TDA);
    }

    // Record the BQ25120A timer_reset bit on RESETPIN wake (long-hold button
    // vs. programmer-driven nRESET) for downstream policy decisions.
    if (reset_reas & RESET_RESETREAS_RESETPIN_Msk) {
        oe_boot_state.timer_reset = bat_state & (1 << 4);
    }

    // reset the reset reason
    NRF_RESET->RESETREAS = 0xFFFFFFFF;

    setup_pmic();
    battery_controller.set_int_callback(battery_controller_callback);

    op_state state = fuel_gauge.operation_state();
    if (state.SEC != BQ27220::SEALED) {
        fuel_gauge.setup(_battery_settings);
    }

    // If we woke from System OFF via a GPIO event (INT or PG), check if it was intended.
    // INT also pulses on PMIC faults and timer resets. If the user isn't holding the 
    // button and power isn't connected, this is an undesired wake.
    if (reset_reas & RESET_RESETREAS_OFF_Msk) {
        if (!battery_controller.power_connected() && earable_btn.getState() == BUTTON_RELEASED) {
            LOG_WRN("Undesired wake from System OFF detected. Powering down.");
            return power_down(false);
        }
    }

    // Battery health is the only gate on booting: if we can't run safely,
    // power down and rely on the charger wake path to bring us back.
    if (!check_battery()) {
        LOG_WRN("Battery check failed — powering down.");
        return power_down(false);
    }

    power_on = true;

    // Start watching the button GPIO so a long-press + release toggles power.
    // The first release consumes the power-on press so it doesn't immediately
    // toggle us back off.
    k_work_reschedule(&power_button_watch_work, K_MSEC(100));

    oe_state.charging_state = battery_controller.power_connected() ? POWER_CONNECTED
                                                                   : DISCHARGING;
    if (oe_state.charging_state == POWER_CONNECTED) {
        k_work_schedule(&charge_ctrl_delayable, K_NO_WAIT);
    }

    battery_controller.set_power_connect_callback(power_good_callback);
    fuel_gauge.set_int_callback(fuel_gauge_callback);

    int ret = device_is_ready(error_led.port);
    if (!ret) {
        LOG_WRN("Error LED not ready.");
    }

    ret = gpio_pin_configure_dt(&error_led, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        LOG_INF("Failed to set Error LED as output: ERROR -%i.", ret);
    }

    float capacity = fuel_gauge.capacity();
    if (fabsf(capacity - _battery_settings.capacity) > 1e-4f) {
        fuel_gauge.setup(_battery_settings);
        set_error_led();
    }

#ifdef CONFIG_BOOTLOADER_MCUBOOT
    bool img_confirmed = boot_is_img_confirmed();

	if (!img_confirmed) {
		ret = boot_write_img_confirmed();
		if (ret) {
			LOG_ERR("Failed to confirm image");
			// reboot and revert to last confirmed image
			sys_reboot(SYS_REBOOT_COLD);
		}
        LOG_INF("Image confirmed");
        #ifdef CONFIG_SETUP_FUEL_GAUGE
        fuel_gauge.setup(_battery_settings);
        #endif
	}
#endif

    state_indicator.init(oe_state);

    uint32_t device_id[2];

    // Read the device ID
    device_id[0] = nrf_ficr_deviceid_get(NRF_FICR, 0);
    device_id[1] = nrf_ficr_deviceid_get(NRF_FICR, 1);

    oe_boot_state.device_id = (((uint64_t) device_id[1]) << 32) | device_id[0];

    return 0;
}

void PowerManager::set_error_led(int val) {
    gpio_pin_set_dt(&error_led, val > 0 ? 1 : 0);
}

bool PowerManager::check_battery() {
    bool charging = battery_controller.power_connected();

    if (charging) {
        float voltage = fuel_gauge.voltage();

        if (voltage < _battery_settings.u_charge_prevent) {
            battery_controller.disable_charge();
            return false;
        }

        float temp = fuel_gauge.temperature();
        
        if (temp < _battery_settings.temp_min || temp > _battery_settings.temp_max) {
            battery_controller.disable_charge();
            return false;
        } else if (temp < _battery_settings.temp_fast_min || temp > _battery_settings.temp_fast_max) {
            battery_controller.write_charging_control(_battery_settings.i_charge / 2);
            battery_controller.enable_charge();
        } else {
            battery_controller.write_charging_control(_battery_settings.i_charge);
            battery_controller.enable_charge();
        }
    }

    bat_status bs = fuel_gauge.battery_status();
    if (bs.SYSDWN) return false;

    return true;
}

void PowerManager::get_battery_status(battery_level_status &status) {
    BQ25120a::ChargePhase phase = battery_controller.read_charge_phase();

    status.flags = 0;
    status.power_state = 0x1; // battery_present

    // charging state
    if (battery_controller.power_connected())  {
        status.power_state |= (0x1 << 1); // external source wired (wireless = 3-4),
        if (phase == BQ25120a::ChargePhase::Charging) {
            status.power_state |= (0x1 << 5); // charging
            status.power_state |= (0x1 << 9); // const current
        }
        else if (phase == BQ25120a::ChargePhase::Done) status.power_state |= (0x3 << 5); // inactive discharge
    } else {
        status.power_state |= (0x2 << 5); // active discharge
    }

    // battery level
    gauge_status gs = fuel_gauge.gauging_state();

    // charge level
    if (gs.edv1) status.power_state |= (0x3 << 7); // critical
    else if (gs.edv2) status.power_state |= (0x2 << 7); // low
    else status.power_state |= (0x1 << 7); // good
}

void PowerManager::get_energy_status(battery_energy_status &status) {
    float voltage = fuel_gauge.voltage();
    float current_mA = fuel_gauge.current();
    float capacity = fuel_gauge.capacity(); 

    status.flags = 0b00011010; // presence of fields
    status.voltage = sfloat_from_float(voltage);
    status.charge_rate = sfloat_from_float(voltage * current_mA / 1000);
    status.available_capacity = sfloat_from_float(3.7f * capacity / 1000);
}

void PowerManager::get_health_status(battery_health_status &status) {
    float state_of_health = fuel_gauge.state_of_health();
    int cycle_count = fuel_gauge.cycle_count();
    float temp = fuel_gauge.temperature(); 

    status.flags = 0b00000111; // presence of fields
    status.battery_health_summary = state_of_health;
    status.cycle_count = cycle_count;
    status.current_temperature = round(CLAMP(temp,-127,128));
}

void bt_disconnect_handler(struct bt_conn *conn, void * data) {
    int ret;
    struct bt_conn_info info;

    ret = bt_conn_get_info(conn, &info);
    if (ret != 0) return;
    
    if (info.state == BT_CONN_STATE_CONNECTED) {
        ret = bt_mgmt_conn_disconnect(conn, *((uint8_t*)data));
    }
}

void PowerManager::setup_pmic() {
    battery_controller.setup(_battery_settings);
}

void PowerManager::shutdown_subsystems() {
    int ret;

    uint8_t data = BT_HCI_ERR_REMOTE_USER_TERM_CONN;
    bt_conn_foreach(BT_CONN_TYPE_ALL, bt_disconnect_handler, &data);

    ret = bt_mgmt_ext_adv_stop(0);
    if (ret) LOG_WRN("Failed to stop ext adv: %d", ret);

    stop_sensor_manager();

    ret = bt_mgmt_stop_watchdog();
    if (ret) LOG_WRN("Failed to stop watchdog: %d", ret);

    dac.end();
}

void PowerManager::reboot() {
    shutdown_subsystems();
    sys_reboot(SYS_REBOOT_COLD);
}

int PowerManager::power_down(bool fault) {
    int ret;

    shutdown_subsystems();

    led_controller.power_off();

    bool charging = battery_controller.power_connected();

    // Let BT disconnect events from bt_conn_disconnect above propagate, then
    // tear down the host. Side effect: on nRF5340 this also writes
    // NETWORK.FORCEOFF=Hold via the bt_hci_transport_teardown → nrf53_cpunet_enable(false)
    // path (zephyr/drivers/bluetooth/hci/nrf53_support.c + nrf53_cpunet_mgmt.c),
    // so no separate net-core shutdown is needed here.
    k_msleep(200);
    int bt_ret = bt_disable();
    if (bt_ret) LOG_WRN("bt_disable() failed: %d", bt_ret);

    if (fault) {
        LOG_WRN("Power off due to fault");
    } else {
        LOG_INF("Power off");
    }
    LOG_PANIC();

    gpio_pin_set_dt(&error_led, 0);

    if (charging) {
        sys_reboot(SYS_REBOOT_COLD);
        return 0;
    }

    // Disable EN_LS_LDO in the BQ25120A so the 3.3V LDO is fully off
    battery_controller.write_LS_control(false);

    ret = pm_device_action_run(ls_sd,  PM_DEVICE_ACTION_SUSPEND);
    ret = pm_device_action_run(ls_3_3, PM_DEVICE_ACTION_SUSPEND);
    ret = pm_device_action_run(ls_1_8, PM_DEVICE_ACTION_SUSPEND);

    // Pre-poweroff audit. Expected: load-switch pins LOW, EN_LS_LDO=0.
    // pmic_cd reflects the CD line state *before* we drive it high for
    // Ship Mode entry below. netcore_off reflects whatever bt_disable()
    // left behind. Anything else is a possible leak.
    {
        uint8_t ls_1_8_pin = nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(1, 11));
        uint8_t ls_3_3_pin = nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(0, 14));
        uint8_t ls_sd_pin  = nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(1, 12));
        uint8_t ppg_ldo    = nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(0, 6));
        uint8_t pmic_cd    = nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(0, 17));
        uint8_t on_btn     = nrf_gpio_pin_read(NRF_GPIO_PIN_MAP(1, 5));
        uint8_t netcore_off = NRF_RESET->NETWORK.FORCEOFF & 1;
        uint8_t en_ls_ldo = (battery_controller.read_ls_ldo_ctrl_raw() >> 7) & 1;
        LOG_WRN("poweroff audit: ls_1_8=%u ls_3_3=%u ls_sd=%u ppg_ldo=%u "
                "EN_LS_LDO=%u pmic_cd=%u on_btn=%u netcore_off=%u",
                ls_1_8_pin, ls_3_3_pin, ls_sd_pin, ppg_ldo,
                en_ls_ldo, pmic_cd, on_btn, netcore_off);
        LOG_PANIC();
    }

    ret = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);

    // Ship Mode entry (BQ25120A datasheet §9.3.1.1 Figure 15) requires MR
    // high. If we got here via the long-press WAKE_2 path, the user is still
    // holding the button (MR low); wait for release. Bound the wait so a
    // stuck button can't block shutdown forever — after the timeout we fall
    // through to sys_poweroff() which gives System OFF (~µA) rather than
    // Ship Mode (~nA), but at least we don't spin.
    //
    // Poll the raw ON+BTN pin (gpio1.5, high when button pressed) directly
    // rather than earable_btn.getState(): the Button class updates its
    // cached state from a k_work on the system work queue, which is the
    // same queue we may be executing on — so getState() can be stale.
    const uint32_t BTN_PIN = NRF_GPIO_PIN_MAP(1, 5);
    for (int i = 0; i < 50 && nrf_gpio_pin_read(BTN_PIN); i++) {
        k_msleep(100);
    }

    // Fire-and-die: drives CD high and writes EN_SHIPMODE=1. The BQ25120A
    // latches BAT FET off within tQUIET (< 1 ms per datasheet timing
    // diagrams), SYS collapses, the nRF5340 loses power. Quiescent drain in
    // Ship Mode is ~2 nA (datasheet I_BAT_SHIP) vs ~0.7–4 µA in Hi-Z — the
    // ~1000× factor is what gives months-of-shelf-life behaviour. Wake path:
    // long MR press (>= MRRESET time configured in setup()) asserts the
    // BQ25120A RESET output, which is wired to nRF5340 nRESET → cold boot.
    battery_controller.enter_ship_mode();

    // Fallbacks. If Ship Mode entry was declined (e.g. VIN re-appeared in
    // the last millisecond, invalidating the VIN<VUVLO precondition), drop
    // to System OFF via sys_poweroff(); if even that returns, cold-reboot.
    k_msleep(100);
    sys_poweroff();

    k_msleep(1000);
    sys_reboot(SYS_REBOOT_COLD);
}


void PowerManager::charge_task() {
    if (charger_init_pending) {
        charger_init_pending = false;
        LOG_INF("Setting up charge controller ........");
        setup_pmic();
        battery_controller.enable_charge();
    }
    k_work_submit(&fuel_gauge_work);
}

int cmd_setup_fuel_gauge(const struct shell *shell, size_t argc, const char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    fuel_gauge.setup(power_manager._battery_settings);

    power_manager.reboot();

    return 0;
}

static int cmd_battery_info(const struct shell *shell, size_t argc, const char **argv) {
    ARG_UNUSED(argc);
	ARG_UNUSED(argv);

    shell_print(shell, "------------------ Battery Info ------------------");
    // Battery fuel gauge status
    bat_status status = fuel_gauge.battery_status();
    shell_print(shell, "Battery Status:");
    shell_print(shell, "  Present: %i, Full Charge: %i, Full Discharge: %i", 
            status.BATTPRES, status.FC, status.FD);

    // Basic measurements
    shell_print(shell, "Basic Measurements:");
    shell_print(shell, "  Voltage: %.3f V", fuel_gauge.voltage());
    shell_print(shell, "  Temperature: %.1f °C", fuel_gauge.temperature());
    shell_print(shell, "  Current: %.1f mA (avg: %.1f mA)", 
            fuel_gauge.current(), fuel_gauge.average_current());
    shell_print(shell, "  State of Charge: %.1f%%", fuel_gauge.state_of_charge());

    // Capacity info
    shell_print(shell, "Capacity Information:");
    shell_print(shell, "  Design Capacity: %.1f mAh", fuel_gauge.design_cap());
    shell_print(shell, "  Full Charge Capacity: %.1f mAh", fuel_gauge.capacity());
    shell_print(shell, "  Remaining Capacity: %.1f mAh", fuel_gauge.remaining_cap());
    
    // Time estimates
    float ttf = fuel_gauge.time_to_full();
    float tte = fuel_gauge.time_to_empty();
    shell_print(shell, "Time Estimates:");
    shell_print(shell, "  Time to Full: %ih %02dmin", (int)ttf / 60, (int)ttf % 60);
    shell_print(shell, "  Time to Empty: %ih %02dmin", (int)tte / 60, (int)tte % 60);

    // Battery controller status
    shell_print(shell, "Charging Information:");
    BQ25120a::ChargePhase phase = battery_controller.read_charge_phase();
    shell_print(shell, "  Charging State: %i", static_cast<int>(phase));
    shell_print(shell, "  Power Good: %i", battery_controller.power_connected());

    struct chrg_state charge_ctrl = battery_controller.read_charging_control();
    shell_print(shell, "  Charge Control: enabled=%i, current=%.1f mA",
            charge_ctrl.enabled, charge_ctrl.mAh);

    uint8_t en_ls_ldo = (battery_controller.read_ls_ldo_ctrl_raw() >> 7) & 1;

    // Load switch / LDO rail state
    shell_print(shell, "Load Switches:");
    shell_print(shell, "  ls_1_8 (P1.11): %d", nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(1, 11)));
    shell_print(shell, "  ls_3_3 (P0.14): %d", nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(0, 14)));
    shell_print(shell, "  ls_sd  (P1.12): %d", nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(1, 12)));
    shell_print(shell, "  PPG LDO (P0.06): %d", nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(0, 6)));
    shell_print(shell, "  BQ25120A EN_LS_LDO: %u", en_ls_ldo);

    return 0;
}

static void i2c_manual_recover(uint32_t pin_scl, uint32_t pin_sda) {
    // Configure SCL and SDA as standard push-pull
    nrf_gpio_cfg_output(pin_scl);
    nrf_gpio_pin_set(pin_scl);
    nrf_gpio_cfg_output(pin_sda);
    nrf_gpio_pin_set(pin_sda);
    k_usleep(100);

    // Generate 9 SCL clock pulses to clock out any stuck transmitting slave
    for (int i = 0; i < 10; i++) {
        nrf_gpio_pin_clear(pin_scl);
        k_usleep(10);
        nrf_gpio_pin_set(pin_scl);
        k_usleep(10);
    }

    // Generate a STOP condition (SDA LOW to HIGH while SCL is HIGH)
    nrf_gpio_pin_clear(pin_scl);
    k_usleep(10);
    nrf_gpio_pin_clear(pin_sda); // SDA LOW
    k_usleep(10);
    nrf_gpio_pin_set(pin_scl); // SCL HIGH
    k_usleep(10);
    nrf_gpio_pin_set(pin_sda); // SDA HIGH
    k_usleep(10);

    // Restore to Input (High-Impedance)
    nrf_gpio_cfg_input(pin_scl, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(pin_sda, NRF_GPIO_PIN_NOPULL);
}

static int cmd_sensor_diag(const struct shell *shell, size_t argc, const char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(shell, "=== Sensor Bus Diagnostic ===");

    // 1. Load switch GPIO states (output pins need nrf_gpio_pin_out_read)
    shell_print(shell, "\n-- Load Switch GPIO States --");
    shell_print(shell, "  ls_1_8 (P1.11): %d", nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(1, 11)));
    shell_print(shell, "  ls_3_3 (P0.14): %d", nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(0, 14)));
    shell_print(shell, "  ls_sd  (P1.12): %d", nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(1, 12)));
    shell_print(shell, "  PPG LDO (P0.06): %d", nrf_gpio_pin_out_read(NRF_GPIO_PIN_MAP(0, 6)));

    // 2. I2C bus line levels (HIGH = idle/OK, LOW = stuck)
    shell_print(shell, "\n-- I2C Bus Line Levels --");
    shell_print(shell, "  I2C2: SCL(P1.00)=%d  SDA(P1.15)=%d",
        nrf_gpio_pin_read(NRF_GPIO_PIN_MAP(1, 0)),
        nrf_gpio_pin_read(NRF_GPIO_PIN_MAP(1, 15)));
    shell_print(shell, "  I2C3: SCL(P1.02)=%d  SDA(P1.03)=%d",
        nrf_gpio_pin_read(NRF_GPIO_PIN_MAP(1, 2)),
        nrf_gpio_pin_read(NRF_GPIO_PIN_MAP(1, 3)));

    // 3. BQ25120a PMIC registers
    shell_print(shell, "\n-- BQ25120a PMIC --");
    uint8_t charging_state = battery_controller.read_charging_state();
    uint8_t fault = battery_controller.read_fault();
    uint8_t ts_fault = battery_controller.read_ts_fault();
    uint8_t ls_ldo_raw = battery_controller.read_ls_ldo_ctrl_raw();
    float ldo_voltage = battery_controller.read_ldo_voltage();

    shell_print(shell, "  Charging state reg: 0x%02x (state=%d)", charging_state, charging_state >> 6);
    shell_print(shell, "  Fault reg: 0x%02x", fault);
    for (const auto &b : BQ25120a::fault_bits) {
        if (fault & b.mask) shell_print(shell, "    %s", b.name);
    }
    shell_print(shell, "  TS fault reg: 0x%02x", ts_fault);
    shell_print(shell, "  LS_LDO_CTRL raw: 0x%02x", ls_ldo_raw);
    shell_print(shell, "    EN_LS_LDO (bit7): %d", (ls_ldo_raw >> 7) & 1);
    shell_print(shell, "    LDO voltage: %.1f V", (double)ldo_voltage);
    shell_print(shell, "  Power Good (PG): %d", battery_controller.power_connected());

    // 3b. If LDO voltage is wrong, try to fix it and report
    if ((ls_ldo_raw >> 7) == 0) {
        shell_print(shell, "\n  EN_LS_LDO is OFF — re-running PMIC setup...");
        power_manager.setup_pmic();
        uint8_t after = battery_controller.read_ls_ldo_ctrl_raw();
        shell_print(shell, "  After setup: LS_LDO_CTRL=0x%02x (EN=%d)", after, (after >> 7) & 1);
    }

    // 4. I2C3 sensor probes
    shell_print(shell, "\n-- I2C3 Sensor Probes --");
    const struct device *i2c3_dev = DEVICE_DT_GET(DT_NODELABEL(i2c3));
    struct { const char *name; uint8_t addr; } sensors[] = {
        {"BMX160 (IMU)",  0x68},
        {"BMP388 (Baro)", 0x76},
        {"BMA580 (Bone)", 0x18},
        {"MLX90632 (Temp)", 0x3A},
    };
    for (int i = 0; i < 4; i++) {
        uint8_t dummy;
        int ret = i2c_burst_read(i2c3_dev, sensors[i].addr, 0x00, &dummy, 1);
        shell_print(shell, "  %s @0x%02x: %s (ret=%d)",
            sensors[i].name, sensors[i].addr,
            ret == 0 ? "OK" : "FAIL", ret);
    }

    // 5. I2C2 sensor probe
    shell_print(shell, "\n-- I2C2 Sensor Probes --");
    const struct device *i2c2_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));
    {
        uint8_t dummy;
        int ret = i2c_burst_read(i2c2_dev, 0x62, 0x00, &dummy, 1);
        shell_print(shell, "  MAXM86161 (PPG) @0x62: %s (ret=%d)",
            ret == 0 ? "OK" : "FAIL", ret);
    }

    shell_print(shell, "\n=== Diagnostic Complete ===");
    return 0;
}

static int cmd_sensor_bus_reset(const struct shell *shell, size_t argc, const char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    // 1. Stop all sensors to halt ongoing I2C traffic
    shell_print(shell, "Stopping all sensors...");
    stop_sensor_manager();
    k_msleep(100);

    // 2. Disable EN_LS_LDO in PMIC register BEFORE pulling GPIOs
    //    With EN_LS_LDO=1 persisting in the PMIC, the 3.3V LDO may leak
    //    even with LSCTRL low, preventing sensors from fully de-powering.
    shell_print(shell, "Disabling EN_LS_LDO in PMIC...");
    battery_controller.write_LS_control(false);

    // 3. Suspend I2C peripherals before GPIO takeover
    shell_print(shell, "Suspending I2C peripherals...");
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2), okay)
    const struct device *i2c2_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));
    pm_device_action_run(i2c2_dev, PM_DEVICE_ACTION_SUSPEND);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c3), okay)
    const struct device *i2c3_dev = DEVICE_DT_GET(DT_NODELABEL(i2c3));
    pm_device_action_run(i2c3_dev, PM_DEVICE_ACTION_SUSPEND);
#endif

    // 4. Turn off ALL load switches via raw GPIO (bypass PM refcounts)
    shell_print(shell, "Turning off all load switches...");
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 11));   // ls_1_8
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 11));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0, 14));   // ls_3_3 / LSCTRL
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(0, 14));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 12));   // ls_sd
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 12));

    // 5. Force I2C lines LOW to break back-powering through pull-ups/ESD diodes
    shell_print(shell, "Forcing I2C lines LOW to break back-power path...");
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 0));    // I2C2 SCL
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 0));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 15));   // I2C2 SDA
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 15));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 2));    // I2C3 SCL
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 2));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1, 3));    // I2C3 SDA
    nrf_gpio_pin_clear(NRF_GPIO_PIN_MAP(1, 3));

    // 6. Power off so EN_LS_LDO=0 persists and sensors stay de-powered.
    //    Leave off as long as needed, then press button to restart.
    //    setup() will re-enable the LDO on boot.
    shell_print(shell, "Powering off. Press button to restart.");
    k_msleep(20);
    sys_poweroff();

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(battery_cmd,
    SHELL_COND_CMD(CONFIG_SHELL, info, NULL, "Print battery info", cmd_battery_info),
    SHELL_COND_CMD(CONFIG_SHELL, setup, NULL, "Setup fuel gauge", cmd_setup_fuel_gauge),
    SHELL_COND_CMD(CONFIG_SHELL, sensor_diag, NULL, "Diagnose sensor bus and power rail state", cmd_sensor_diag),
    SHELL_COND_CMD(CONFIG_SHELL, sensor_bus_reset, NULL, "Hard reset I2C sensor bus via GPIO sink", cmd_sensor_bus_reset),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(battery, &battery_cmd, "Power Manager Commands", NULL);

PowerManager power_manager;