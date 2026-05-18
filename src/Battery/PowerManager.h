#ifndef _POWER_MANAGER_H
#define _POWER_MANAGER_H

#include <zephyr/kernel.h>

#include "BQ27220.h"
#include "BQ25120a.h"

#include "../bluetooth/gatt_services/battery_service.h"

#include "openearable_common.h"
#include "BootState.h"

#define DEBOUNCE_POWER_MS K_MSEC(1000)

class PowerManager {
public:
    int begin();

    int power_down(bool fault = false);

    void reboot();

    void get_battery_status(battery_level_status &status);
    void get_energy_status(battery_energy_status &status);
    void get_health_status(battery_health_status &status);

    void set_error_led(int val = 1);
    void setup_pmic();

    static k_work_delayable power_down_work;
private:
    bool power_on = false;
    bool charging_disabled = false;
    // ISR→work handoff: the USB plug-in ISR sets this; the work handler
    // clears it after re-running setup_pmic() (I2C can't run in ISR context).
    bool charger_init_pending = false;
    // Set the first time the button GPIO reads RELEASED after boot. BQ25120A
    // WAKE events are ignored until this flag is set, so the power-on press
    // doesn't immediately toggle power back off.
    bool first_release_seen = false;

    void charge_task();

    void power_connected();

    // Teardown sequence shared between reboot() and power_down(): disconnect
    // BT peers, stop ext adv, stop sensors, stop BT watchdog, stop DAC. Each
    // step logs a breadcrumb and failures are warned but not fatal so the
    // caller always reaches its final sys_reboot / sys_poweroff.
    void shutdown_subsystems();

    bool check_battery();

    k_timeout_t chrg_interval = K_SECONDS(CONFIG_BATTERY_CHARGE_CONTROLLER_NORMAL_INTERVAL_SECONDS);

    static k_work_delayable charge_ctrl_delayable;
    static k_work_delayable power_button_watch_work;

    static k_work fuel_gauge_work;
    static k_work battery_controller_work;
    static k_work usb_plug_reboot_work;

    static void charge_ctrl_work_handler(struct k_work * work);
    static void power_down_work_handler(struct k_work * work);
    static void fuel_gauge_work_handler(struct k_work * work);
    static void battery_controller_work_handler(struct k_work * work);
    static void power_button_watch_handler(struct k_work * work);
    static void usb_plug_reboot_handler(struct k_work * work);

    static void power_good_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
    static void fuel_gauge_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
    static void battery_controller_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

    const battery_settings _battery_settings = {
        3.7, 4.3, 3.0, 2.5,  // Spannungen
        10, 110, 200,        // Ströme
        108,                 // Kapazität
        0, 15, 45, 50        // Temperaturen
    };

    const struct gpio_dt_spec error_led = GPIO_DT_SPEC_GET(DT_NODELABEL(led_error), gpios);

    friend int cmd_setup_fuel_gauge(const struct shell *shell, size_t argc, const char **argv);
};

extern PowerManager power_manager;

#endif