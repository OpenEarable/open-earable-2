#ifndef _BQ25120a_H
#define _BQ25120a_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <math.h>
//#include <Wire.h>
#include <TWIM.h>

#include "openearable_common.h"

#define BQ25120a_I2C_TIMEOUT_US 66
#define BQ25120a_HIGH_Z_TIMEOUT_US 1000

#define EPS 1e-3

struct chrg_state {
        float mAh = 0;
        bool enabled = false;
        bool high_impedance = false;
};

struct button_state {
        bool wake_1 = false;
        bool wake_2 = false;
};

struct ilim_uvlo {
        float lim_mA = 0;
        float uvlo_v = 0;
};

class BQ25120a {
public:
    enum registers : uint8_t {
        CTRL = 0x00,
        FAULT = 0x01,
        TS_FAULT = 0x02,
        CHARGE_CTRL = 0x03,
        TERM_CTRL = 0x04,
        BAT_VOL_CTRL = 0x05,
        LS_LDO_CTRL = 0x07,
        BTN_CTRL = 0x08,
        ILIM_UVLO = 0x09,
        TIMERS = 0x0B
    };

    BQ25120a(TWIM * i2c);

    int begin();
    int set_wakeup_int();

    int reset();

    int setup_ts_control();

    // Configuration is verified with charging inhibited. The power manager must
    // approve battery conditions and call enable_charge() afterwards.
    int setup(const battery_settings &_battery_settings);
    // Boot only: holds the peripheral 3.3 V rail off while repairing retained
    // LS/LDO state. Call before starting any consumers of that rail.
    int setup_boot(const battery_settings &_battery_settings);
    int recover_charging(const battery_settings &_battery_settings);
    int enter_ship_mode();
    bool read_status(uint8_t &control, uint8_t &fault, uint8_t &ts_fault);
    // Detect watchdog/default-register drift without resetting the live rails.
    // A mismatch or failed read inhibits charging until setup succeeds again.
    bool configuration_valid(const battery_settings &settings, float current);

    bool power_connected();
    int enter_high_impedance();
    int exit_high_impedance();
    int disable_charge();
    int enable_charge();
    
    uint8_t read_charging_state();
    uint8_t read_fault();
    uint8_t read_ts_fault();
    chrg_state read_charging_control();
    int write_charging_control(float mA);
    float read_battery_voltage_control();
    int write_battery_voltage_control(float volt);
    struct chrg_state read_termination_control();
    int write_termination_control(float mA, bool enable_termination = true);
    ilim_uvlo read_uvlo_ilim();
    int write_uvlo_ilim(ilim_uvlo param);
    void disable_ts();
    int write_LDO_voltage_control(float volt);
    float read_ldo_voltage();
    int write_LS_control(bool enable);

    button_state read_button_state();

    int set_power_connect_callback(gpio_callback_handler_t handler);
    int set_int_callback(gpio_callback_handler_t handler);
private:
    bool readReg(uint8_t reg, uint8_t * buffer, uint16_t len);
    bool writeReg(uint8_t reg, const uint8_t * buffer, uint16_t len);
    int write_verified(uint8_t reg, uint8_t value, uint8_t mask = 0xFF);
    int set_cd(bool high);
    void wait_for_i2c();

    const int address = DT_REG_ADDR(DT_NODELABEL(bq25120a));

    uint64_t last_i2c = 0;
    uint64_t last_high_z = 0;
    bool charge_disabled = true;
    bool configured = false;

    TWIM *_i2c;

    gpio_callback power_connect_cb_data;
    gpio_callback int_cb_data;

    const struct gpio_dt_spec pg_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(bq25120a), pg_gpios);
    const struct gpio_dt_spec cd_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(bq25120a), cd_gpios);
    const struct gpio_dt_spec int_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(bq25120a), int_gpios);
    const struct gpio_dt_spec lsctrl_pin =
        GPIO_DT_SPEC_GET(DT_CHILD(DT_NODELABEL(bq25120a), load_switch), enable_gpios);

    /*const struct gpio_dt_spec pg_pin = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(bq25120a), pg_gpios, {0});
    const struct gpio_dt_spec cd_pin = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(bq25120a), cd_gpios, {0});
    const struct gpio_dt_spec int_pin = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(bq25120a), int_gpios, {0});*/
};

extern BQ25120a battery_controller;

#endif
