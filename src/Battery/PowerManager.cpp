#include "PowerManager.h"

#include "macros_common.h"

#include <zephyr/sys/poweroff.h>

#include <zephyr/pm/pm.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include "../drivers/LED_Controller/KTD2026.h"
#include "../drivers/SSM6515.h"

#include "bt_mgmt.h"
#include "bt_mgmt_ctlr_cfg_internal.h"

//#include <zephyr/task_wdt/task_wdt.h>

#include <zephyr/logging/log_ctrl.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(power_manager, CONFIG_MAIN_LOG_LEVEL);

K_TIMER_DEFINE(PowerManager::charge_timer, PowerManager::charge_timer_handler, NULL);

K_WORK_DELAYABLE_DEFINE(PowerManager::power_down_work, PowerManager::power_down_work_handler);

//K_WORK_DEFINE(PowerManager::power_down_work, PowerManager::power_down_work_handler);
K_WORK_DEFINE(PowerManager::charge_ctrl_work, PowerManager::charge_ctrl_work_handler);
K_WORK_DEFINE(PowerManager::fuel_gauge_work, PowerManager::fuel_gauge_work_handler);

extern struct k_msgq battery_queue;

battery_data PowerManager::msg;

LoadSwitch PowerManager::v1_8_switch(GPIO_DT_SPEC_GET(DT_NODELABEL(load_switch), gpios));

void PowerManager::charge_timer_handler(struct k_timer * timer) {
	k_work_submit(&charge_ctrl_work);
}

void PowerManager::power_switch_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	bool power_on = power_switch.is_on();

    if (!power_on) {
        k_work_reschedule(&power_manager.power_down_work, K_MSEC(DEBOUNCE_POWER_MS));
        //k_work_submit(&power_manager.power_down_work);
    } else {
        k_work_cancel_delayable(&power_manager.power_down_work);
    }
}

void PowerManager::fuel_gauge_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    k_work_submit(&fuel_gauge_work);
}

void PowerManager::power_good_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	bool power_good = battery_controller.power_connected();

    k_work_submit(&fuel_gauge_work);

    if (power_good) {
        power_manager.last_charging_state = 0;
        // start charge control timer
        k_timer_start(&charge_timer, K_NO_WAIT, power_manager.chrg_interval);
    } else {
        k_timer_stop(&charge_timer);
        //if (!power_switch.is_on()) k_work_submit(&power_manager.power_down_work);
        if (!power_switch.is_on()) k_work_reschedule(&power_manager.power_down_work, K_NO_WAIT);
    }
}

void PowerManager::power_down_work_handler(struct k_work * work) {
	power_manager.power_down();
}

void PowerManager::charge_ctrl_work_handler(struct k_work * work) {
	power_manager.charge_task();
}

void PowerManager::fuel_gauge_work_handler(struct k_work * work) {
    int ret;

    LOG_INF("Fuel Gauge GPOUT Interrupt");
    msg.battery_level = fuel_gauge.state_of_charge();

    battery_level_status status;

    power_manager.get_battery_status(status);

    msg.charging_state = status.power_state;

	ret = k_msgq_put(&battery_queue, &msg, K_NO_WAIT);
	if (ret == -EAGAIN) {
		LOG_WRN("power manager msg queue full");
	}
}

int PowerManager::begin() {
    //int ret;

    battery_controller.begin();
    fuel_gauge.begin();
    power_switch.begin();

    battery_controller.setup();

    // check setup
    op_state state = fuel_gauge.operation_state();
    if (state.SEC != BQ27220::SEALED) {
        //battery_controller.setup();
        fuel_gauge.setup();
    }

    k_timer_init(&charge_timer, charge_timer_handler, NULL);

    // check charging state
    bool charging = battery_controller.power_connected();

    if (charging) {
        float temp = fuel_gauge.temperature();
        
        if (temp < temp_min || temp > temp_max) {
            // set params
            battery_controller.disable_charge();
        } else if (temp < temp_fast_min || temp > temp_fast_max) {
            // set params
            battery_controller.write_charging_control(55);
            battery_controller.enable_charge();
        } else {
            // normal params
            battery_controller.write_charging_control(110);
            battery_controller.enable_charge();
        }

        power_manager.last_charging_state = 0;
        //k_timer_start(&charge_timer, K_NO_WAIT, power_manager.chrg_interval);
        k_timer_start(&charge_timer, power_manager.chrg_interval, power_manager.chrg_interval);

        while(!power_switch.is_on() && battery_controller.power_connected()) {
            __WFE();
        }
    }

    if (power_switch.is_on()) {
        //TODO: check power on condition
        
        battery_controller.set_power_connect_callback(power_good_callback);
        fuel_gauge.set_int_callback(fuel_gauge_callback);
        power_switch.set_power_off_callback(power_switch_callback);

        //float voltage = battery_controller.read_ldo_voltage();
        //if (voltage != 3.3) battery_controller.write_LDO_voltage_control(3.3);

        battery_controller.enter_high_impedance();
        //v1_8_switch.begin();

        int ret = pm_device_runtime_enable(ls_1_8);
        if (ret != 0) {
            LOG_WRN("Error setting up load switch.");
        }

        ret = pm_device_runtime_enable(ls_3_3);
        if (ret != 0) {
            LOG_WRN("Error setting up load switch.");
        }

        //pm_device_runtime_get(ls_3_3);

        //enum pm_device_state state;
        //pm_device_state_get(DEVICE_DT_GET(DT_NODELABEL(load_switch)), &state);

        /*ret = pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(bq25120a)));
        if (ret == 0) {
                //printk("Sucessful getting device.\n");
                LOG_INF("Sucessful getting device.\n");
        } else {
                //printk("Error getting device.\n");
                LOG_WRN("Error getting device.\n");
        }*/

        return 0;
    }

    return power_down();
}

void PowerManager::get_battery_status(battery_level_status &status) {
    battery_controller.exit_high_impedance();
    uint8_t charging_state = battery_controller.read_charging_state() >> 6;
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
    else if (gs.edv2) status.power_state |= (0x2 << 7); //low
    else status.power_state |= (0x1 << 7); // good
	//	| (0x1 << 12); // fault reason
}

void PowerManager::get_energy_status(battery_energy_status &status) {
    float voltage = fuel_gauge.voltage();
    float current_mA = fuel_gauge.current();
    float capacity = fuel_gauge.capacity(); 

    status.flags = 0b00011010; // presence of fields
    status.voltage = voltage;
    status.charge_rate = voltage * current_mA / 1000;
    status.available_capacity = 3.7 * capacity / 1000;
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

void PowerManager::set_1_8(bool on) {
    v1_8_switch.set(on);
}

void PowerManager::set_3_3(bool on) {
    float voltage = battery_controller.read_ldo_voltage();
    if (voltage != 3.3) battery_controller.write_LDO_voltage_control(3.3);
    //k_usleep(10);
    battery_controller.load_switch.set(on);
}

int PowerManager::power_down(bool fault) {
    int ret;
    // if charging do not power off
    // uint16_t last_charging_state = 0;
    /*if (!fault && battery_controller.power_connected()) {
        return -1;
    }*/


    // disconnect devices
    uint8_t data = BT_HCI_ERR_REMOTE_USER_TERM_CONN;
    bt_conn_foreach(BT_CONN_TYPE_ALL, bt_disconnect_handler, &data);

    ret = bt_le_adv_stop();

    // power disonnected
    // prepare interrupts

    led_controller.power_off();

    //power_manager.set_1_8(false);
    //power_manager.set_3_3(false);

    bool charging = battery_controller.power_connected();

    if (!charging) {
        ret = battery_controller.set_wakeup_int();
        if (ret != 0) return ret;
        ret = fuel_gauge.set_wakeup_int();
        if (ret != 0) return ret;
        
        // check battery good
        if (!fault) ret = power_switch.set_wakeup_int();
        if (ret != 0) return ret;

        battery_controller.enter_high_impedance();
    }

    //TODO: prevent crashing with bt_disable (does not wake up)
    /*ret = bt_disable();

    if (ret != 0) {
        NVIC_SystemReset();
    }*/

    LOG_INF("Power off");
    LOG_PANIC();

    ret = bt_mgmt_stop_watchdog();
    ERR_CHK(ret);

    dac.end();

    // TODO: check states of load switch (should already be suspended
    // if all devieses have been terminated correctly)

    ret = pm_device_action_run(ls_1_8, PM_DEVICE_ACTION_SUSPEND);
    ret = pm_device_action_run(ls_3_3, PM_DEVICE_ACTION_SUSPEND);

    if (charging) {
        NVIC_SystemReset();
        return 0;
    }

    ret = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
    /*ERR_CHK(ret);

    const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c1));
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

    NVIC_SystemReset();
}


void PowerManager::charge_task() {
    uint16_t charging_state = battery_controller.read_charging_state() >> 6;

    LOG_INF("Charger Watchdog ...................");

    if (last_charging_state == 0) {
        LOG_INF("Setting up charge controller ........");
        battery_controller.setup();
        battery_controller.enable_charge();
    }

    if (last_charging_state != charging_state) {
            switch (charging_state) {
            case 0:
                LOG_INF("charging state: ready");
                break;
            case 1:
                LOG_INF("charging state: charing");
                break;
            case 2:
                LOG_INF("charging state: done");
                break;
            case 3:
                LOG_WRN("charging state: fault");

                uint16_t ts_fault = battery_controller.read_ts_fault();
                LOG_WRN("TS_ENABLED: %i, TS FAULT: %i", ts_fault >> 7, (ts_fault >> 5) & 0x3);

                battery_controller.setup();
                
                break;
            }
    }

    last_charging_state = charging_state;
}

PowerManager power_manager;