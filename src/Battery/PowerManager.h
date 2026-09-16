#ifndef _POWER_MANAGER_H
#define _POWER_MANAGER_H

#include <zephyr/kernel.h>
// Boot/button handling and the battery worker share these state flags.
#include <atomic>

#include "BQ27220.h"
#include "BQ25120a.h"

#include "../bluetooth/gatt_services/battery_service.h"

#include "openearable_common.h"
#include "BootState.h"

class PowerManager {
public:
    int begin();

    /** Queue an irreversible shutdown; USB reboots into charge-only mode.
     * Returns 0 after accepting the request, including repeated requests.
     */
    int power_down(bool fault = false);
    void reboot();

    void get_battery_status(battery_level_status &status);
    void get_energy_status(battery_energy_status &status);
    void get_health_status(battery_health_status &status);

    void set_error_led(int val = 1);

    static k_work_delayable power_down_work;
private:
    // Serialize shutdown with battery servicing and retain a bounded timer retry.
    std::atomic<bool> power_on{false};
    std::atomic<bool> stopping{false};
    bool shutdown_fault = false;
    bool usb_connected = false;
    std::atomic<bool> charger_configured{false};
    std::atomic<bool> indicator_ready{false};
    bool timer_recovery_used = false;
    bool charger_fault_latched = false;
    float requested_current = 0;
    void finish_power_down();
    void set_charger_session(bool recovery_used, bool fault_latched);

    void charge_task();

    bool check_battery();

    k_timeout_t chrg_interval = K_SECONDS(CONFIG_BATTERY_CHARGE_CONTROLLER_NORMAL_INTERVAL_SECONDS);

    static k_work_delayable charge_ctrl_delayable;

    static k_work fuel_gauge_work;
    static k_work battery_controller_work;

    static void charge_ctrl_work_handler(struct k_work * work);
    static void power_down_work_handler(struct k_work * work);
    static void fuel_gauge_work_handler(struct k_work * work);
    static void battery_controller_work_handler(struct k_work * work);

    static void power_good_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
    static void fuel_gauge_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
    static void battery_controller_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

    // Match the CP1454 A4X charge limits and cut system loads off above deep discharge.
    const battery_settings _battery_settings = {
        3.7, 4.3, 3.0, 2.5,  // Nominal, regulation, UVLO, charge-prevent voltage (V)
        10, 100, 200,        // Shared precharge/termination, fast charge, input (mA)
        108,                 // Design capacity (mAh), VARTA CP1454 A4X
        0, 15, 45, 45        // Min, fast min/max, max charge temperature (C)
    };

    const struct gpio_dt_spec error_led = GPIO_DT_SPEC_GET(DT_NODELABEL(led_error), gpios);

    friend int cmd_setup_fuel_gauge(const struct shell *shell, size_t argc, const char **argv);
};

extern PowerManager power_manager;

#endif
