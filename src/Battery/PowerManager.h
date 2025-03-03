#ifndef _POWER_MANAGER_H
#define _POWER_MANAGER_H

#include <zephyr/kernel.h>

#include "BQ27220.h"
#include "BQ25120a.h"

#include "../bluetooth/gatt_services/battery_service.h"

#include "openearable_common.h"
#include "BootState.h"

#define CHARGE_CONTROLLER_INTERVAL K_SECONDS(10)

#define DEBOUNCE_POWER_MS K_MSEC(1000)

class PowerManager {
public:
    int begin();

    int power_down(bool fault = false);
    //bool check_boot_condition();

    //static LoadSwitch v1_8_switch;

    void reboot();

    void get_battery_status(battery_level_status &status);
    void get_energy_status(battery_energy_status &status);
    void get_health_status(battery_health_status &status);

    // bool is_power_on();

    //void set_1_8(bool on);
    //void set_3_3(bool on);

    /*BQ27220 fuel_gauge;
    BQ25120a battery_controller;*/

    static k_work_delayable power_down_work;
private:
    bool power_on = false;
    uint16_t last_charging_state = 0;

    void charge_task();

    void power_connected();

    bool check_battery();

    k_timeout_t chrg_interval = CHARGE_CONTROLLER_INTERVAL;

    static k_timer charge_timer;

    static k_work charge_ctrl_work;
    //static k_work power_down_work;
    static k_work fuel_gauge_work;
    static k_work battery_controller_work;

    static void charge_ctrl_work_handler(struct k_work * work);
    static void power_down_work_handler(struct k_work * work);
    static void fuel_gauge_work_handler(struct k_work * work);
    static void battery_controller_work_handler(struct k_work * work);

    static void power_good_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
    static void fuel_gauge_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
    static void battery_controller_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

    static void charge_timer_handler(struct k_timer * timer);

    static battery_data msg;

    const battery_settings _battery_settings = {
        3.7, 4.3, 3.0, 2.5,  // Spannungen
        2.5, 110, 200,       // Ströme
        108,                 // Kapazität
        0, 15, 45, 50        // Temperaturen
    };

    const struct gpio_dt_spec error_led = GPIO_DT_SPEC_GET(DT_NODELABEL(led_error), gpios);
};

extern PowerManager power_manager;

#endif