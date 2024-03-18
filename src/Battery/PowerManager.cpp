#include "PowerManager.h"

#include "macros_common.h"

#include <zephyr/sys/poweroff.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device_runtime.h>

#include "led.h"

#include "bt_mgmt.h"
#include "bt_mgmt_ctlr_cfg_internal.h"

#include <zephyr/task_wdt/task_wdt.h>

#include <zephyr/logging/log_ctrl.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(power_manager, CONFIG_MAIN_LOG_LEVEL);

K_TIMER_DEFINE(PowerManager::charge_timer, PowerManager::charge_timer_handler, NULL);

K_WORK_DEFINE(PowerManager::power_down_work, PowerManager::power_down_work_handler);
K_WORK_DEFINE(PowerManager::charge_ctrl_work, PowerManager::charge_ctrl_work_handler);
K_WORK_DEFINE(PowerManager::fuel_gauge_work, PowerManager::fuel_gauge_work_handler);

void PowerManager::charge_timer_handler(struct k_timer * timer)
{
	k_work_submit(&charge_ctrl_work);
}

void PowerManager::power_switch_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	bool power_on = power_switch.is_on();

    if (!power_on) {
        k_work_submit(&power_manager.power_down_work);
    }
}

void PowerManager::fuel_gauge_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	
}

void PowerManager::power_good_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	bool power_good = battery_controller.power_connected();

        if (power_good) {
            // start charge control timer
            k_timer_start(&charge_timer, power_manager.chrg_interval, power_manager.chrg_interval);
        } else {
	        k_timer_stop(&charge_timer);
            if (!power_switch.is_on()) k_work_submit(&power_manager.power_down_work);
        }
}

void PowerManager::power_down_work_handler(struct k_work * work) {
	power_manager.power_down();
}

void PowerManager::charge_ctrl_work_handler(struct k_work * work) {
	power_manager.charge_task();
}

void PowerManager::fuel_gauge_work_handler(struct k_work * work) {
	
}

int PowerManager::begin() {
    int ret;

    fuel_gauge.begin();
    battery_controller.begin();
    power_switch.begin();

    k_timer_init(&charge_timer, charge_timer_handler, NULL);

    // check charging state
    bool charging = battery_controller.power_connected();

    float temp_min = 0;
    float temp_fast_min = 15;
    float temp_fast_max = 45;
    float temp_max = 50;

    if (charging) {
        float temp = fuel_gauge.temperature();
        
        if (temp < temp_min || temp > temp_max) {
            // set params
            battery_controller.disable_charge();
        } else if (temp < temp_fast_min || temp > temp_fast_max) {
            // set params
            battery_controller.enable_charge();
        } else {
            // normal params
            battery_controller.enable_charge();
        }
    }

    if (power_switch.is_on()) {
        battery_controller.set_power_connect_callback(power_good_callback);
        fuel_gauge.set_int_callback(fuel_gauge_callback);
        power_switch.set_power_off_callback(power_switch_callback);

        battery_controller.enter_high_impedance();
    } else {
        return power_down();
    }

    return 0;
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

int PowerManager::power_down(bool fault) {
    // if charging do not power off
    uint16_t last_charging_state = 0;
    if (!fault && battery_controller.power_connected()) {
        return -1;
    }

    // power disonnected
    // prepare interrupts

    int ret;

    ret = battery_controller.set_wakeup_int();
    if (ret != 0) return ret;
    ret = fuel_gauge.set_wakeup_int();
    if (ret != 0) return ret;
    
    // check battery good
    if (!fault) ret = power_switch.set_wakeup_int();
    if (ret != 0) return ret;

    //led_controller.power_off();

    battery_controller.enter_high_impedance();

    LOG_INF("Power off");
    LOG_PANIC();

    //ret = bt_mgmt_conn_disconnect(msg.conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    uint8_t data = BT_HCI_ERR_REMOTE_USER_TERM_CONN;
    bt_conn_foreach(BT_CONN_TYPE_ALL, bt_disconnect_handler, &data);
    if (ret) {
        LOG_ERR("Failed to disconnect: %d", ret);
    }

    bt_disable();

    ret = led_off(0);
	ret = led_off(1);
	ret = led_off(2);
	ret = led_off(3);

    int wdt_ch_id = task_wdt_delete(wdt_ch_id);

    //k_msleep(100);

    const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    ret = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
    ERR_CHK(ret);

    const struct device *const i2s = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    ret = pm_device_action_run(i2s, PM_DEVICE_ACTION_SUSPEND);
    ERR_CHK(ret);

    /*const struct device *const watch_dog = DEVICE_DT_GET(DT_CHOSEN(zephyr_bt_hci_rpmsg_ipc));
    ret = pm_device_action_run(watch_dog, PM_DEVICE_ACTION_SUSPEND);
    ERR_CHK(ret);*/

    /*const struct device *const watch_dog = DEVICE_DT_GET(DT_ALIAS(watchdog0));
    ret = pm_device_action_run(watch_dog, PM_DEVICE_ACTION_SUSPEND);
    ERR_CHK(ret);*/

    //struct pm_state_info info = {PM_STATE_SOFT_OFF, 0, 0};

    //pm_state_force(0u, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});
    //k_sleep(K_SECONDS(SLEEP_S));

    //pm_system_suspend();

    //pm_suspend_devices();

    //pm_device_init_off

    //pm_device_state_get

    sys_poweroff();
}


void PowerManager::charge_task() {
    uint16_t charging_state = battery_controller.read_charging_state() >> 6;

    if (last_charging_state != charging_state) {
            switch (charging_state) {
            case 0:
            LOG_INF("charging state: ready\n");
            break;
            case 1:
            LOG_INF("charging state: charing\n");
            break;
            case 2:
            LOG_INF("charging state: done\n");
            break;
            case 3:
            LOG_WRN("charging state: fault\n");

            uint16_t ts_fault = battery_controller.read_ts_fault();
            printk("TS_ENABLED: %i, TS: %i\n", ts_fault >> 7, (ts_fault >> 5) & 0x3);ts_fault &= ~(1 << 7);

            battery_controller.disable_ts();
            break;
            }
    }

    last_charging_state = charging_state;
}

PowerManager power_manager;