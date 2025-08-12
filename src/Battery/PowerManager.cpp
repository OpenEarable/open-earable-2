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

//K_TIMER_DEFINE(PowerManager::charge_timer, PowerManager::charge_timer_handler, NULL);

K_WORK_DELAYABLE_DEFINE(PowerManager::charge_ctrl_delayable, PowerManager::charge_ctrl_work_handler);

K_WORK_DELAYABLE_DEFINE(PowerManager::power_down_work, PowerManager::power_down_work_handler);

//K_WORK_DEFINE(PowerManager::power_down_work, PowerManager::power_down_work_handler);
//K_WORK_DEFINE(PowerManager::charge_ctrl_work, PowerManager::charge_ctrl_work_handler);
K_WORK_DEFINE(PowerManager::fuel_gauge_work, PowerManager::fuel_gauge_work_handler);
K_WORK_DEFINE(PowerManager::battery_controller_work, PowerManager::battery_controller_work_handler);

ZBUS_CHAN_DEFINE(battery_chan, struct battery_data, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0));

static struct battery_data msg;

//LoadSwitch PowerManager::v1_8_switch(GPIO_DT_SPEC_GET(DT_NODELABEL(load_switch), gpios));

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
        power_manager.last_charging_state = 0;
        k_work_schedule(&charge_ctrl_delayable, K_NO_WAIT);
    } else {
        k_work_cancel_delayable(&charge_ctrl_delayable);
        if (!power_manager.power_on) k_work_reschedule(&power_manager.power_down_work, K_NO_WAIT);
    }
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
    button_state state;

    //uint8_t val = gpio_pin_get_dt(&power_manager.error_led);
    //gpio_pin_set_dt(&power_manager.error_led, 1 - val);

    battery_controller.exit_high_impedance();
    state = battery_controller.read_button_state();
    battery_controller.enter_high_impedance();

    if (state.wake_2) {
        power_manager.power_on = !power_manager.power_on;
        //LOG_INF("Power on: %i", power_manager.power_on);

        if (!power_manager.power_on) power_manager.power_down();
    }

}

void PowerManager::fuel_gauge_work_handler(struct k_work * work) {
    int ret;
    battery_level_status status;

    msg.battery_level = fuel_gauge.state_of_charge();

    bat_status bat = fuel_gauge.battery_status();

    power_manager.get_battery_status(status);

    // full discharge
    //if (bat.FD) k_work_reschedule(&power_manager.power_down_work, K_NO_WAIT);
    if (power_manager.power_on && bat.SYSDWN) {
        LOG_WRN("Battery reached system down voltage.");
        k_work_reschedule(&power_manager.power_down_work, K_NO_WAIT);
    }

    if (bat.CHGINH) {
        power_manager.charging_disabled = true;
        battery_controller.disable_charge();
    } else if (power_manager.charging_disabled) {
        battery_controller.enable_charge();
    }

    float current;
    float target_current;
    float voltage;

    battery_controller.exit_high_impedance();

    uint16_t charging_state = battery_controller.read_charging_state() >> 6;
    gauge_status gs;

    switch (charging_state) {
        case 0:
            LOG_INF("charging state: discharge");
            msg.charging_state = DISCHARGING;

            gs = fuel_gauge.gauging_state();

            if (gs.edv2) {
                #ifdef CONFIG_BATTERY_ENABLE_LOW_STATE
                msg.charging_state = BATTERY_LOW;
                #endif
            }
            if (gs.edv1) {
                msg.charging_state = BATTERY_CRITICAL;
            }
            break;
        case 1:
            LOG_INF("charging state: charging");

            if (bat.SYSDWN) {
                msg.charging_state = PRECHARGING;
                break;
            }

            current = fuel_gauge.current();
            target_current = fuel_gauge.charge_current();
            voltage = fuel_gauge.voltage();

            msg.charging_state = POWER_CONNECTED;

            LOG_DBG("Voltage: %.3f V", voltage);
            LOG_DBG("Charging current: %.3f mA", current);
            LOG_DBG("Target current: %.3f mA", target_current);
            LOG_DBG("State of charge: %.3f %%", fuel_gauge.state_of_charge());

            // check if target current is met (if not tapering)
            if (current > 0.8 * target_current - 2 * power_manager._battery_settings.i_term) {
                msg.charging_state = CHARGING;
            } 
            else if (voltage > power_manager._battery_settings.u_term - 0.02) {
                #ifdef CONFIG_BATTERY_ENABLE_TRICKLE_CHARGE
                msg.charging_state = TRICKLE_CHARGING;
                #else
                msg.charging_state = CHARGING;
                #endif
            }
            
            break;
        case 2:
            LOG_INF("charging state: done");
            msg.charging_state = FULLY_CHARGED;
            break;
        case 3:
            LOG_WRN("charging state: fault");
            msg.charging_state = FAULT;

            uint8_t fault = battery_controller.read_fault();
            // Battery fuel gauge status
            bat_status status = fuel_gauge.battery_status();
            voltage = fuel_gauge.voltage();
            current = fuel_gauge.current();

            // cleared after read
            if (fault & (1 << 4)) {
                LOG_WRN("Input over voltage.");
            }

            // as long as fault exists
            if (fault & (1 << 5)) {
                bool power_connected = battery_controller.power_connected();
                if (power_connected && current > 0.5 * power_manager._battery_settings.i_term) {
                    msg.charging_state = PRECHARGING;
                }
                LOG_WRN("Battery under voltage: %.3f V", voltage);
            }

            // cleared after read
            if (fault & (1 << 6)) {
                LOG_WRN("Input under voltage");
            }

            // as long as fault exists
            if (fault & (1 << 7)) {
                LOG_WRN("Battery over voltage");
            }

            uint8_t ts_fault = battery_controller.read_ts_fault();

            if ((ts_fault >> 5) & 0x7) {
                LOG_WRN("TS_ENABLED: %i, TS FAULT: %i", ts_fault >> 7, (ts_fault >> 5) & 0x3);
                battery_controller.setup(power_manager._battery_settings);
            }

            LOG_DBG("------------------ Battery Info ------------------");
            LOG_DBG("Battery Status:");
            LOG_DBG("  Present: %d, Full Charge: %d, Full Discharge: %d", 
                    status.BATTPRES, status.FC, status.FD);

            // Basic measurements
            LOG_DBG("Basic Measurements:");
            LOG_DBG("  Voltage: %.3f V", voltage);
            LOG_DBG("  Current: %.3f mA", current);
            break;
    }

    battery_controller.enter_high_impedance();

    power_manager.last_charging_msg_state = msg.charging_state;
    
    // Adjust interval based on state
    if (msg.charging_state == FAULT || msg.charging_state == POWER_CONNECTED) {
        power_manager.chrg_interval = K_SECONDS(CONFIG_BATTERY_CHARGE_CONTROLLER_FAST_INTERVAL_SECONDS);
    } else {
        power_manager.chrg_interval = K_SECONDS(CONFIG_BATTERY_CHARGE_CONTROLLER_NORMAL_INTERVAL_SECONDS);
    }

	//ret = k_msgq_put(&battery_queue, &msg, K_NO_WAIT);
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

    battery_controller.exit_high_impedance();

    uint8_t bat_state = battery_controller.read_charging_state();

    button_state btn = battery_controller.read_button_state();

    power_on = btn.wake_2;

    // get reset reason
    uint32_t reset_reas = NRF_RESET->RESETREAS;

    // reset the reset reason
    NRF_RESET->RESETREAS = 0xFFFFFFFF;
    
    if (reset_reas & RESET_RESETREAS_RESETPIN_Msk) {
        oe_boot_state.timer_reset = bat_state & (1 << 4);
        power_on |= oe_boot_state.timer_reset;
    }

    /*if (reset_reas & RESET_RESETREAS_DOG1_Msk) {
        printk("Reset durch Watchdog-Timer\n");
    }*/

    if (reset_reas & RESET_RESETREAS_SREQ_Msk) {
        LOG_INF("Rebooting ...");
        power_on = true;
    }

    /*if (reset_reas & RESET_RESETREAS_LOCKUP_Msk) {
        printk("Reset durch CPU Lockup\n");
    }*/

    battery_controller.setup(_battery_settings);
    battery_controller.set_int_callback(battery_controller_callback);

    // check setup
    op_state state = fuel_gauge.operation_state();
    if (state.SEC != BQ27220::SEALED) {
        //battery_controller.setup();
        fuel_gauge.setup(_battery_settings);
    }

    //k_timer_init(&charge_timer, charge_timer_handler, NULL);

    bool battery_condition = check_battery();

    if (!battery_condition) LOG_WRN("Battery check failed.");

    // check charging state
    bool charging = battery_controller.power_connected();

    if (!battery_condition) {
        power_on = false;
        // LOG_ERR("Bad battery condition.");
        if (!charging){
            //TODO: Flash red LED once
            return power_down(false);
        }
    }

    if (charging) {
        power_manager.last_charging_state = 0;
        
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

        state_indicator.init(oe_state);

        k_work_schedule(&charge_ctrl_delayable, K_NO_WAIT);

        while(!power_on && battery_controller.power_connected()) {
            //__WFE();
            k_sleep(K_SECONDS(1));
        }
    } else {
        oe_state.charging_state = DISCHARGING;
    }

    if (!power_on) return power_down();

    //TODO: check power on condition
    // either not charging and edv1 or charging and edv0 and temperature
    
    battery_controller.set_power_connect_callback(power_good_callback);
    fuel_gauge.set_int_callback(fuel_gauge_callback);
    //battery_controller.set_int_callback(battery_controller_callback);

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

    // check if fuel gauge has wrong value
    float capacity = fuel_gauge.capacity();
    if (abs(capacity - _battery_settings.capacity) > 1e-4) {
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

    // Lesen der DEVICEID
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
            // set params
            battery_controller.disable_charge();
            return false;
        } else if (temp < _battery_settings.temp_fast_min || temp > _battery_settings.temp_fast_max) {
            // set params
            battery_controller.write_charging_control(_battery_settings.i_charge / 2);
            battery_controller.enable_charge();
        } else {
            // normal params
            battery_controller.write_charging_control(_battery_settings.i_charge);
            battery_controller.enable_charge();
        }
    }

    bat_status bs = fuel_gauge.battery_status();
    if (bs.SYSDWN) return false;

    //gauge_status gs = fuel_gauge.gauging_state();
    //if (gs.edv1) return false; // critical battery state

    return true;
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
    int ret;

    // disconnect devices
    uint8_t data = BT_HCI_ERR_REMOTE_USER_TERM_CONN;
    bt_conn_foreach(BT_CONN_TYPE_ALL, bt_disconnect_handler, &data);

    ret = bt_le_adv_stop();

    // power disonnected
    // prepare interrupts

    led_controller.begin();
    led_controller.power_off();

    stop_sensor_manager();

    bool charging = battery_controller.power_connected();

    if (!charging) {
        ret = battery_controller.set_wakeup_int();
        if (ret != 0) return ret;

        ret = fuel_gauge.set_wakeup_int();
        if (ret != 0) return ret;
        
        // check battery good
        //if (!fault) ret = power_switch.set_wakeup_int();
        //if (ret != 0) return ret;

        battery_controller.enter_high_impedance();
    }

    //TODO: prevent crashing with bt_disable (does not wake up)
    /*ret = bt_disable();

    if (ret != 0) {
        NVIC_SystemReset();
        sys_reboot(SYS_REBOOT_COLD);
    }*/

    if (fault) {
        LOG_WRN("Power off due to fault");
    } else {
        LOG_INF("Power off");
    }
    LOG_PANIC();

    ret = bt_mgmt_stop_watchdog();
    //ERR_CHK(ret);

    dac.end();

    // TODO: check states of load switch (should already be suspended
    // if all devieses have been terminated correctly)

    // turn off error led
	gpio_pin_set_dt(&error_led, 0);

    if (charging) {
        //NVIC_SystemReset();
        sys_reboot(SYS_REBOOT_COLD);
        return 0;
    }

    ret = pm_device_action_run(ls_sd,  PM_DEVICE_ACTION_SUSPEND);
    ret = pm_device_action_run(ls_3_3, PM_DEVICE_ACTION_SUSPEND);
    ret = pm_device_action_run(ls_1_8, PM_DEVICE_ACTION_SUSPEND);
    ret = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);

    /*const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    ret = pm_device_action_run(i2c, PM_DEVICE_ACTION_SUSPEND);
    ERR_CHK(ret);*/

    /*const struct device *const watch_dog = DEVICE_DT_GET(DT_CHOSEN(zephyr_bt_hci_rpmsg_ipc));
    ret = pm_device_action_run(watch_dog, PM_DEVICE_ACTION_SUSPEND);
    ERR_CHK(ret);*/

    /*const struct device *const watch_dog = DEVICE_DT_GET(DT_ALIAS(watchdog0));
    ret = pm_device_action_run(watch_dog, PM_DEVICE_ACTION_SUSPEND);
    ERR_CHK(ret);*/

    sys_poweroff();

    // safety if poweroff failed
    k_msleep(1000);

    //NVIC_SystemReset();
    sys_reboot(SYS_REBOOT_COLD);
}


void PowerManager::charge_task() {
    uint16_t charging_state = battery_controller.read_charging_state() >> 6;

    //LOG_INF("Charger Watchdog ...................");

    if (last_charging_state == 0) {
        LOG_INF("Setting up charge controller ........");
        battery_controller.setup(_battery_settings);
        battery_controller.enable_charge();
    }

    //if (last_charging_state != charging_state ||  ) {
        k_work_submit(&fuel_gauge_work);
        //state_inidicator.set_state()
        /*switch (charging_state) {
        case 0:
            LOG_INF("charging state: ready");
            break;
        case 1:
            LOG_INF("charging state: charging");
            break;
        case 2:
            LOG_INF("charging state: done");
            break;
        case 3:
            LOG_WRN("charging state: fault");

            //battery_controller.setup(_battery_settings);
            
            break;
        }*/
    //}

    last_charging_state = charging_state;
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
    battery_controller.exit_high_impedance();
    
    shell_print(shell, "Charging Information:");
    uint16_t charging_state = battery_controller.read_charging_state() >> 6;
    shell_print(shell, "  Charging State: %i", charging_state);
    shell_print(shell, "  Power Good: %i", battery_controller.power_connected());
    
    struct chrg_state charge_ctrl = battery_controller.read_charging_control();
    shell_print(shell, "  Charge Control: enabled=%i, current=%.1f mA", 
            charge_ctrl.enabled, charge_ctrl.mAh);

    battery_controller.enter_high_impedance();

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(battery_cmd,
    SHELL_COND_CMD(CONFIG_SHELL, info, NULL, "Print battery info", cmd_battery_info),
    SHELL_COND_CMD(CONFIG_SHELL, setup, NULL, "Setup fuel gauge", cmd_setup_fuel_gauge),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(battery, &battery_cmd, "Power Manager Commands", NULL);

PowerManager power_manager;