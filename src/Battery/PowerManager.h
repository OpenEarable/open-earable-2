#ifndef _POWER_MANAGER_H
#define _POWER_MANAGER_H

#include <zephyr/kernel.h>

#include "BQ27220.h"
#include "BQ25120a.h"
#include "PowerSwitch.h"
#include "LoadSwitch.h"

#include "nrf5340_audio_common.h"

#define CHARGE_CONTROLLER_INTERVAL_SECONDS 10

class PowerManager {
public:
    int begin();

    int power_down(bool fault = false);
    //bool check_boot_condition();

    static LoadSwitch v1_8_switch;

    void set_1_8(bool on);
    void set_3_3(bool on);

    /*BQ27220 fuel_gauge;
    BQ25120a battery_controller;
    PowerSwitch power_switch;*/
private:
    uint16_t last_charging_state = 0;
    void charge_task();

    void power_connected();

    k_timeout_t chrg_interval = K_SECONDS(CHARGE_CONTROLLER_INTERVAL_SECONDS);

    static k_timer charge_timer;

    static k_work charge_ctrl_work;
    static k_work power_down_work;
    static k_work fuel_gauge_work;

    static void charge_ctrl_work_handler(struct k_work * work);
    static void power_down_work_handler(struct k_work * work);
    static void fuel_gauge_work_handler(struct k_work * work);

    static void power_switch_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
    static void power_good_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
    static void fuel_gauge_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

    static void charge_timer_handler(struct k_timer * timer);

    static battery_data msg;
};

extern PowerManager power_manager;

#endif