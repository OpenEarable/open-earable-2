#include "PowerManager.h"

#include "macros_common.h"

#include <stdio.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/shell/shell.h>

#include <zephyr/pm/pm.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include <zephyr/zbus/zbus.h>

#ifdef CONFIG_BOOTLOADER_MCUBOOT
#include <zephyr/dfu/mcuboot.h>
#endif

#include <hal/nrf_ficr.h>
#include <hal/nrf_reset.h>
#include "BatteryPolicy.h"
#include "../audio/audio_datapath.h"

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

// Charger servicing must not wait behind Bluetooth, storage, or sensor work.
K_THREAD_STACK_DEFINE(battery_work_stack, 4096);
static struct k_work_q battery_work_queue;
K_WORK_DELAYABLE_DEFINE(PowerManager::charge_ctrl_delayable, PowerManager::charge_ctrl_work_handler);
K_WORK_DELAYABLE_DEFINE(PowerManager::power_down_work, PowerManager::power_down_work_handler);
K_WORK_DEFINE(PowerManager::fuel_gauge_work, PowerManager::fuel_gauge_work_handler);
K_WORK_DEFINE(PowerManager::battery_controller_work, PowerManager::battery_controller_work_handler);

ZBUS_CHAN_DEFINE(battery_chan, struct battery_data, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0));

void PowerManager::fuel_gauge_callback(const struct device *, struct gpio_callback *, uint32_t) {
    if (!power_manager.stopping) {
        k_work_submit_to_queue(&battery_work_queue, &fuel_gauge_work);
    }
}

void PowerManager::battery_controller_callback(const struct device *, struct gpio_callback *, uint32_t) {
    if (!power_manager.stopping) {
        k_work_submit_to_queue(&battery_work_queue, &battery_controller_work);
    }
}

void PowerManager::power_good_callback(const struct device *, struct gpio_callback *, uint32_t) {
    if (!power_manager.stopping) {
        // Both edges matter: unplugging must keep low-battery monitoring alive.
        k_work_reschedule_for_queue(&battery_work_queue, &charge_ctrl_delayable, K_NO_WAIT);
    }
}

void PowerManager::power_down_work_handler(struct k_work *) {
    power_manager.finish_power_down();
}

void PowerManager::charge_ctrl_work_handler(struct k_work *) {
    if (power_manager.stopping) return;
    power_manager.charge_task();
    if (!power_manager.stopping) {
        k_work_schedule_for_queue(&battery_work_queue, &charge_ctrl_delayable,
                                 power_manager.chrg_interval);
    }
}

void PowerManager::battery_controller_work_handler(struct k_work *) {
    if (power_manager.stopping) return;
    battery_controller.exit_high_impedance();
    button_state state = battery_controller.read_button_state();
    battery_controller.enter_high_impedance();
    if (!state.wake_2) return;
    if (power_manager.power_on) {
        power_manager.power_down();
    } else if (power_manager.charger_configured && power_manager.check_battery()) {
        power_manager.power_on = true;
    } else {
        LOG_WRN("Ignoring start request until battery has recovered");
    }
}

void PowerManager::fuel_gauge_work_handler(struct k_work *) {
    if (!power_manager.stopping) power_manager.charge_task();
}

int PowerManager::begin() {
    earable_state oe_state = {};

    oe_state.charging_state = DISCHARGING;
    oe_state.pairing_state = PAIRED;

    k_work_queue_start(&battery_work_queue, battery_work_stack,
                       K_THREAD_STACK_SIZEOF(battery_work_stack), 5, NULL);
    k_thread_name_set(&battery_work_queue.thread, "battery");
    int controller_ret = battery_controller.begin();
    int gauge_ret = fuel_gauge.begin();
    earable_btn.begin();

    battery_controller.exit_high_impedance();

    uint8_t bat_state = battery_controller.read_charging_state();

    button_state btn = battery_controller.read_button_state();

    uint8_t retained = oe_boot_flags(NRF_POWER->GPREGRET[1]);
    bool remain_off = retained & OE_BOOT_FLAG_CHARGE_ONLY;
    // Keep the USB-session safety state even when boot consumes the off request.
    NRF_POWER->GPREGRET[1] = oe_boot_encode(retained & OE_BOOT_SAFETY_FLAGS);
    timer_recovery_used = retained & OE_BOOT_FLAG_TIMER_USED;
    charger_fault_latched = retained & OE_BOOT_FLAG_CHARGER_FAULT;
    if (!battery_controller.power_connected()) set_charger_session(false, false);
    power_on = btn.wake_2;

    // get reset reason
    uint32_t reset_reas = NRF_RESET->RESETREAS;

    // reset the reset reason
    NRF_RESET->RESETREAS = 0xFFFFFFFF;
    
    if (reset_reas & RESET_RESETREAS_RESETPIN_Msk) {
        oe_boot_state.timer_reset = bat_state & (1 << 4);
        power_on = power_on || oe_boot_state.timer_reset;
    }

    /*if (reset_reas & RESET_RESETREAS_DOG1_Msk) {
        printk("Reset durch Watchdog-Timer\n");
    }*/

    if (reset_reas & RESET_RESETREAS_SREQ_Msk) {
        LOG_INF("Rebooting ...");
        power_on = !remain_off;
    }
    // A retained button/reset indication must not undo an explicit off request.
    if (remain_off) power_on = false;

    /*if (reset_reas & RESET_RESETREAS_LOCKUP_Msk) {
        printk("Reset durch CPU Lockup\n");
    }*/

    charger_configured = controller_ret == 0 &&
                         battery_controller.setup_boot(_battery_settings) == 0;
    if (controller_ret || gauge_ret || !charger_configured) {
        LOG_ERR("Battery hardware initialization failed");
        power_on = false;
    }

    bool battery_condition = charger_configured && gauge_ret == 0 && check_battery();

    // Gauge programming is never a prerequisite for depleted-cell recovery.
    // Do it only on a healthy boot before any battery work can run. FCC is a
    // learned value; use design capacity to identify an unconfigured profile.
    if (battery_condition && power_on) {
        op_state state = fuel_gauge.operation_state();
        if (state.SEC != BQ27220::SEALED ||
            fabsf(fuel_gauge.design_cap() - _battery_settings.capacity) > 0.5f ||
            IS_ENABLED(CONFIG_SETUP_FUEL_GAUGE)) {
            if (fuel_gauge.setup(_battery_settings) != 0) battery_condition = false;
        }
    }

    if (!battery_condition) LOG_WRN("Battery check failed.");

    // check charging state
    bool charging = battery_controller.power_connected();
    // Setup above already covered this USB state. Only later edges should
    // invalidate it; publishing callbacks must not race a redundant setup.
    usb_connected = charging;

    if (!battery_condition) {
        power_on = false;
        // LOG_ERR("Bad battery condition.");
        if (!charging){
            //TODO: Flash red LED once
            power_down(false);
            k_sleep(K_FOREVER);
        }
    }

    if (charging) {

        int ret = pm_device_runtime_enable(ls_1_8);
        if (ret != 0) {
            LOG_WRN("Error setting up load switch 1.8V.");
        }

        ret = pm_device_runtime_enable(ls_3_3);
        if (ret != 0) {
            LOG_WRN("Error setting up load switch 3.3V.");
        }

        //battery_level_status bat_status;
        //get_battery_status(&bat_status);

        oe_state.charging_state = POWER_CONNECTED;

        // A reset PMIC defaults to LDO pass-through. Do not energize the LED
        // rail until its 3.3 V configuration has been verified.
        if (charger_configured) {
            state_indicator.init(oe_state);
            indicator_ready = true;
        }

        // Keep the network core off during battery recovery/charge-only boot.
        nrf_reset_network_force_off(NRF_RESET, true);
        battery_controller.set_int_callback(battery_controller_callback);
        battery_controller.set_power_connect_callback(power_good_callback);
        fuel_gauge.set_int_callback(fuel_gauge_callback);
        k_work_schedule_for_queue(&battery_work_queue, &charge_ctrl_delayable, K_NO_WAIT);

        while(!power_on && battery_controller.power_connected()) {
            //__WFE();
            k_sleep(K_SECONDS(1));
        }
    } else {
        oe_state.charging_state = DISCHARGING;
    }

    if (!power_on || !check_battery() || !charger_configured || stopping) {
        power_down();
        k_sleep(K_FOREVER);
    }
    nrf_reset_network_force_off(NRF_RESET, false);

    //TODO: check power on condition
    // either not charging and edv1 or charging and edv0 and temperature
    
    if (!charging) {
        battery_controller.set_power_connect_callback(power_good_callback);
        fuel_gauge.set_int_callback(fuel_gauge_callback);
        battery_controller.set_int_callback(battery_controller_callback);
    }

    //float voltage = battery_controller.read_ldo_voltage();
    //if (voltage != 3.3) battery_controller.write_LDO_voltage_control(3.3);

    battery_controller.enter_high_impedance();

    int ret = pm_device_runtime_enable(ls_1_8);
    if (ret != 0) {
        LOG_WRN("Error setting up load switch 1.8V.");
    }

    ret = pm_device_runtime_enable(ls_3_3);
    if (ret != 0) {
        LOG_WRN("Error setting up load switch 3.3V.");
    }

    ret = pm_device_runtime_enable(ls_sd);
    if (ret != 0) {
        LOG_WRN("Error setting up load switch SD.");
    }

    ret = device_is_ready(error_led.port); //bool
    if (!ret) {
        LOG_WRN("Error LED not ready.");
        //return -1;
    }

    ret = gpio_pin_configure_dt(&error_led, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        LOG_INF("Failed to set Error LED as output: ERROR -%i.", ret);
        //return ret;
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
	}
#endif

    state_indicator.init(oe_state);
    indicator_ready = true;

    uint32_t device_id[2];

    // Lesen der DEVICEID
    device_id[0] = nrf_ficr_deviceid_get(NRF_FICR, 0);
    device_id[1] = nrf_ficr_deviceid_get(NRF_FICR, 1);

    oe_boot_state.device_id = (((uint64_t) device_id[1]) << 32) | device_id[0];

    // Poll on battery as well as USB: gauge IRQs are an optimization, not
    // the only protection against discharging below the shutdown threshold.
    k_work_schedule_for_queue(&battery_work_queue, &charge_ctrl_delayable, K_NO_WAIT);
    return 0;
}

void PowerManager::set_error_led(int val) {
    gpio_pin_set_dt(&error_led, val > 0 ? 1 : 0);
}

bool PowerManager::check_battery() {
    bat_status status = {};
    float voltage, temperature;
    if (!fuel_gauge.read_safety_status(status, voltage, temperature)) return false;
    return battery_policy::can_start(voltage, temperature, status.SYSDWN,
                                    _battery_settings);
}

void PowerManager::get_battery_status(battery_level_status &status) {
    battery_controller.exit_high_impedance();
    uint8_t charging_state = battery_controller.read_charging_state() >> 6;

    status.flags = 0;
    status.power_state = 0x1; // battery_present

    // charging state
    if (battery_controller.power_connected())  {
        status.power_state |= (0x1 << 1); // external source wired (wireless = 3-4),
        if (charging_state == 0x1) {
            status.power_state |= (0x1 << 5); // charging
            status.power_state |= (0x1 << 9); // const current
        }
        else if (charging_state == 0x2) status.power_state |= (0x3 << 5); // inactive discharge
    } else {
        status.power_state |= (0x2 << 5); // active discharge
    }
    battery_controller.enter_high_impedance();

    // battery level
    gauge_status gs = fuel_gauge.gauging_state();

    // charge level
    if (gs.edv1) status.power_state |= (0x3 << 7); // critical
    else if (gs.edv2) status.power_state |= (0x2 << 7); // low
    else status.power_state |= (0x1 << 7); // good
	//	status.power_state |= (0x1 << 12); // fault reason
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

void PowerManager::reboot() {
    int ret;
    
    // disconnect devices
    uint8_t data = BT_HCI_ERR_REMOTE_USER_TERM_CONN;
    bt_conn_foreach(BT_CONN_TYPE_ALL, bt_disconnect_handler, &data);

    ret = bt_le_adv_stop();

    stop_sensor_manager();

    ret = bt_mgmt_stop_watchdog();
    ERR_CHK(ret);

    dac.end();

    sys_reboot(SYS_REBOOT_COLD);
}

int PowerManager::power_down(bool fault) {
    // Serialize all shutdown paths with the charger worker. Never return to
    // normal operation after partially shutting down the peripherals.
    if (stopping.exchange(true)) return 0;
    power_on = false;
    shutdown_fault = fault;
    k_work_cancel_delayable(&charge_ctrl_delayable);
    k_work_reschedule_for_queue(&battery_work_queue, &power_down_work, K_NO_WAIT);
    return 0;
}

void PowerManager::set_charger_session(bool recovery_used, bool fault_latched) {
    unsigned int key = irq_lock();
    timer_recovery_used = recovery_used;
    charger_fault_latched = fault_latched;
    uint8_t flags = oe_boot_flags(NRF_POWER->GPREGRET[1]) & OE_BOOT_FLAG_CHARGE_ONLY;
    if (recovery_used) flags |= OE_BOOT_FLAG_TIMER_USED;
    if (fault_latched) flags |= OE_BOOT_FLAG_CHARGER_FAULT;
    NRF_POWER->GPREGRET[1] = oe_boot_encode(flags);
    irq_unlock(key);
}

void PowerManager::finish_power_down() {
    uint8_t reason = BT_HCI_ERR_REMOTE_USER_TERM_CONN;
    bt_conn_foreach(BT_CONN_TYPE_ALL, bt_disconnect_handler, &reason);
    (void)bt_le_adv_stop();
    stop_sensor_manager();
    (void)audio_datapath_stop();
    (void)bt_mgmt_stop_watchdog();
    dac.end();
    if (indicator_ready) led_controller.power_off();
    gpio_pin_set_dt(&error_led, 0);

    // A low battery alert can stay asserted. It must never wake an off device.
    (void)fuel_gauge.disable_wakeup_int();
    LOG_INF("Power off%s", shutdown_fault ? " due to fault" : "");

    if (battery_controller.power_connected()) {
        NRF_POWER->GPREGRET[1] = oe_boot_request_charge_only(NRF_POWER->GPREGRET[1]);
        sys_reboot(SYS_REBOOT_COLD);
        CODE_UNREACHABLE;
    }

    set_charger_session(false, false); // USB absence ends the retained session.
    (void)pm_device_action_run(ls_sd, PM_DEVICE_ACTION_SUSPEND);
    (void)pm_device_action_run(ls_3_3, PM_DEVICE_ACTION_SUSPEND);
    (void)pm_device_action_run(ls_1_8, PM_DEVICE_ACTION_SUSPEND);

    // Ship mode removes SYS/PMID loads rather than just sleeping the CPU.
    // MR must be released for the PMIC to complete this transition.
    int ret = battery_controller.enter_ship_mode();
    if (ret == -EBUSY || battery_controller.power_connected()) {
        NRF_POWER->GPREGRET[1] = oe_boot_request_charge_only(NRF_POWER->GPREGRET[1]);
        sys_reboot(SYS_REBOOT_COLD);
        CODE_UNREACHABLE;
    }
    if (ret) {
        LOG_ERR("Ship mode failed (%d); using SYSTEMOFF", ret);
        battery_controller.enter_high_impedance();
    }
    (void)pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
    nrf_reset_network_force_off(NRF_RESET, true);
    // Level wake can already be asserted. Arm it only after blocking cleanup,
    // with CPU interrupts masked, then enter SYSTEMOFF without another wait.
    (void)irq_lock();
    (void)battery_controller.set_wakeup_int();
    sys_poweroff();
    CODE_UNREACHABLE;
}

void PowerManager::charge_task() {
    if (stopping) return;
    bool usb = battery_controller.power_connected();
    if (usb != usb_connected) {
        usb_connected = usb;
        charger_configured = false;
        if (!usb) set_charger_session(false, false);
        charge_inhibited = true;
        requested_current = 0;
    }
    if (!usb && !power_on) {
        power_down();
        return;
    }

    bat_status battery = {};
    float voltage = 0, temperature = 0;
    bool valid = fuel_gauge.read_safety_status(battery, voltage, temperature);
    bool safe = valid && battery_policy::can_charge(voltage, temperature,
                                                    battery.CHGINH || battery.OTC,
                                                    _battery_settings);
    if (!usb && (!valid || battery.SYSDWN ||
                 voltage <= battery_policy::shutdown_voltage(_battery_settings))) {
        LOG_WRN("Battery shutdown: valid=%d voltage=%.3f", valid, voltage);
        power_down(true);
        return;
    }
    // A running application must also stop drawing a heavy load during USB
    // recovery; a weak input must not let it drain the battery indefinitely.
    if (usb && power_on && (!valid || battery.SYSDWN ||
                           voltage <= battery_policy::shutdown_voltage(_battery_settings))) {
        battery_controller.disable_charge();
        charge_inhibited = true;
        power_down(true);
        return;
    }

    if (usb && !charger_configured && !charger_fault_latched) {
        charger_configured = (indicator_ready ? battery_controller.setup(_battery_settings)
                                             : battery_controller.setup_boot(_battery_settings)) == 0;
        charge_inhibited = true;
        requested_current = 0;
        if (!charger_configured && indicator_ready) {
            // USB edges invalidate our configuration cache. If a PMIC reset
            // also changed the LDO, setup cannot repair it with the live rail
            // enabled. Retry through boot, where both enable sources are off.
            // Persistent bus failures then stay in the boot retry path.
            LOG_ERR("Runtime charger setup failed; restarting into charge-only mode");
            power_down(true);
            return;
        }
    }
    if (usb && charger_configured && !indicator_ready) {
        earable_state state = {};
        state.pairing_state = PAIRED;
        state.charging_state = POWER_CONNECTED;
        state_indicator.init(state);
        indicator_ready = true;
    }
    uint8_t ctrl = 0, fault = 0, ts = 0;
    bool read_ok = battery_controller.read_status(ctrl, fault, ts);
    if (usb && charger_configured && read_ok && !charger_fault_latched &&
        !battery_controller.configuration_valid(_battery_settings,
            requested_current > 0 ? requested_current : _battery_settings.i_charge)) {
        // A PMIC watchdog reset also restores the LDO to pass-through. Its
        // voltage cannot be changed while LSCTRL is high. Inhibit charging and
        // restart in charge-only mode so all rails are off before reconfiguration.
        LOG_ERR("Charger configuration lost; restarting into charge-only mode");
        charger_configured = false;
        charge_inhibited = true;
        power_down(true);
        return;
    }
    // VIN_OV, VIN_UV, BAT_OCP and hot/cold are electrical/thermal faults.
    // BAT_UVLO and warm/cool derating alone are expected during recovery.
    bool blocked = charger_fault_latched || !read_ok || (fault & 0xD0) ||
                   ((ts & BIT(7)) && ((ts >> 5) & 3) == 1);
    if (usb && read_ok && (ctrl & BIT(3))) {
        if (safe && !blocked && !timer_recovery_used) {
            set_charger_session(true, false);
            LOG_WRN("Recovering charger safety timer once for this USB session");
            charger_configured = battery_controller.recover_charging(_battery_settings) == 0;
            if (!charger_configured) set_charger_session(true, true);
            charge_inhibited = true;
            requested_current = 0;
            read_ok = battery_controller.read_status(ctrl, fault, ts);
            blocked = charger_fault_latched || !read_ok || (fault & 0xD0) ||
                      ((ts & BIT(7)) && ((ts >> 5) & 3) == 1);
        } else {
            // CD inhibition itself clears TIMER: remember exhaustion in software
            // so a later poll cannot silently restart the same stalled battery.
            set_charger_session(timer_recovery_used, true);
        }
        if (ctrl & BIT(3)) set_charger_session(timer_recovery_used, true);
        blocked |= charger_fault_latched;
    }
    bool allow = usb && safe && charger_configured && !blocked;
    if (allow) {
        float current = (temperature < _battery_settings.temp_fast_min ||
                         temperature > _battery_settings.temp_fast_max)
                            ? _battery_settings.i_charge / 2 : _battery_settings.i_charge;
        if (requested_current != current) {
            if (battery_controller.write_charging_control(current) < 0) {
                allow = false;
            } else {
                requested_current = current;
            }
        }
    }
    if (allow && charge_inhibited) {
        charge_inhibited = battery_controller.enable_charge() != 0;
    } else if (!allow && !charge_inhibited) {
        battery_controller.disable_charge();
        charge_inhibited = true;
    }
    battery_controller.enter_high_impedance();

    struct battery_data message = {};
    message.battery_level = valid ? fuel_gauge.state_of_charge() : 0;
    if (!usb) {
        message.charging_state = DISCHARGING;
        if (voltage < battery_policy::start_voltage(_battery_settings)) {
            message.charging_state = BATTERY_CRITICAL;
        }
    } else if (!allow || charge_inhibited) {
        message.charging_state = FAULT;
    } else if (battery.SYSDWN || voltage < battery_policy::start_voltage(_battery_settings)) {
        message.charging_state = PRECHARGING;
    } else if ((ctrl >> 6) == 2) {
        message.charging_state = FULLY_CHARGED;
    } else if ((ctrl >> 6) == 1) {
        float current = fuel_gauge.current();
        message.charging_state = current > 0.8f * requested_current - 2 * _battery_settings.i_term
                                     ? CHARGING : POWER_CONNECTED;
#ifdef CONFIG_BATTERY_ENABLE_TRICKLE_CHARGE
        if (voltage > _battery_settings.u_term - 0.02f) {
            message.charging_state = TRICKLE_CHARGING;
        }
#endif
    } else {
        message.charging_state = (ctrl >> 6) == 3 ? FAULT : POWER_CONNECTED;
    }
    chrg_interval = K_SECONDS(message.charging_state == FAULT ?
        CONFIG_BATTERY_CHARGE_CONTROLLER_FAST_INTERVAL_SECONDS :
        CONFIG_BATTERY_CHARGE_CONTROLLER_NORMAL_INTERVAL_SECONDS);
    // Bound channel-lock/subscriber waits; synchronous listeners must also
    // keep their I2C transactions bounded by the bus driver's transfer timeout.
    int ret = zbus_chan_pub(&battery_chan, &message, K_MSEC(100));
    if (ret) LOG_WRN("Battery state publication failed: %d", ret);
}

int cmd_setup_fuel_gauge(const struct shell *shell, size_t argc, const char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int ret = fuel_gauge.setup(power_manager._battery_settings);
    if (ret) {
        shell_error(shell, "Fuel gauge setup failed: %d", ret);
        return ret;
    }
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
    battery_controller.exit_high_impedance();
    
    shell_print(shell, "Charging Information:");
    uint16_t charging_state = battery_controller.read_charging_state() >> 6;
    shell_print(shell, "  Charging State: %i", charging_state);
    shell_print(shell, "  Power Good: %i", battery_controller.power_connected());
    
    struct chrg_state charge_ctrl = battery_controller.read_charging_control();
    shell_print(shell, "  Charge Control: enabled=%i, current=%.1f mA", 
            charge_ctrl.enabled, charge_ctrl.mAh);

    chrg_state preterm = battery_controller.read_termination_control();
    ilim_uvlo limits = battery_controller.read_uvlo_ilim();
    shell_print(shell, "  Precharge/termination: %.1f mA, UVLO: %.3f V, input limit: %.1f mA",
                preterm.mAh, limits.uvlo_v, limits.lim_mA);
    shell_print(shell, "  Charge voltage: %.3f V, LDO voltage: %.3f V",
                battery_controller.read_battery_voltage_control(), battery_controller.read_ldo_voltage());
    battery_controller.enter_high_impedance();
    uint8_t control, fault, ts;
    if (battery_controller.read_status(control, fault, ts)) {
        shell_print(shell, "  Raw status: CTRL=0x%02x FAULT=0x%02x TS=0x%02x",
                    control, fault, ts);
    } else {
        shell_error(shell, "  Charger status unavailable");
    }
    shell_print(shell, "  Gauge: SYSDWN=%i CHGINH=%i OTC=%i",
                status.SYSDWN, status.CHGINH, status.OTC);

    return 0;
}

static int cmd_battery_off(const struct shell *shell, size_t argc, const char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    shell_print(shell, "Powering off; USB retains charging-only operation");
    return power_manager.power_down();
}

SHELL_STATIC_SUBCMD_SET_CREATE(battery_cmd,
    SHELL_COND_CMD(CONFIG_SHELL, info, NULL, "Print battery info", cmd_battery_info),
    SHELL_COND_CMD(CONFIG_SHELL, off, NULL, "Power off (charge-only on USB)", cmd_battery_off),
    SHELL_COND_CMD(CONFIG_SHELL, setup, NULL, "Setup fuel gauge", cmd_setup_fuel_gauge),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(battery, &battery_cmd, "Power Manager Commands", NULL);

PowerManager power_manager;
