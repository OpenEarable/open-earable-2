#ifndef _BQ25120a_H
#define _BQ25120a_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <math.h>
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
        ILIM_UVLO = 0x09
    };

    enum class ChargePhase : uint8_t {
        Discharge = 0,
        Charging = 1,
        Done = 2,
        Fault = 3,
    };

    // Register 0x01 (Faults) bit layout per BQ25120A datasheet Table 13.
    struct FaultBit { uint8_t mask; const char *name; };
    static constexpr FaultBit fault_bits[] = {
        { 1 << 4, "BAT_OCP: battery over-current (cleared on read)" },
        { 1 << 5, "BAT_UVLO: battery under-voltage lockout (persistent)" },
        { 1 << 6, "VIN_UV: input under-voltage (cleared on read)" },
        { 1 << 7, "VIN_OV: input over-voltage (persistent)" },
    };

    BQ25120a(TWIM * i2c);

    int begin();
    int set_wakeup_int();

    int reset();

    void setup(const battery_settings &_battery_settings);

    bool power_connected();
    // Fire-and-die Ship Mode entry (datasheet §9.3.1.1, Figure 15). Caller
    // MUST ensure VIN is absent and MR is high; this drives CD high and
    // writes EN_SHIPMODE=1. SYS collapses within tQUIET and the SoC loses
    // power — this call does not return to a usable state.
    void enter_ship_mode();
    void disable_charge();
    void enable_charge();

    uint8_t read_charging_state();
    ChargePhase read_charge_phase();
    uint8_t read_fault();
    uint8_t read_ts_fault();
    chrg_state read_charging_control();
    uint8_t write_charging_control(float mA);
    float read_battery_voltage_control();
    struct chrg_state read_termination_control();
    ilim_uvlo read_uvlo_ilim();
    void disable_ts();
    float read_ldo_voltage();
    uint8_t read_ls_ldo_ctrl_raw();
    uint8_t write_LS_control(bool enable);

    button_state read_button_state();

    int set_power_connect_callback(gpio_callback_handler_t handler);
    int set_int_callback(gpio_callback_handler_t handler);
private:
    // RAII guard: brackets any I2C access with exit/enter high-impedance.
    // Refcounted so nested scopes compose; mutex serialises concurrent
    // thread/work-handler entries so the 0↔1 transition is atomic. Every
    // public I2C method self-wraps in one of these, so external callers
    // never need to touch it directly.
    class ActiveScope {
    public:
        explicit ActiveScope(BQ25120a &c);
        ~ActiveScope();
        ActiveScope(const ActiveScope&) = delete;
        ActiveScope& operator=(const ActiveScope&) = delete;
    private:
        BQ25120a &c_;
    };

    // Hi-Z control. Driven exclusively by ActiveScope ctor/dtor. Only
    // enter_ship_mode() pokes Hi-Z directly (see its comment for why).
    void enter_high_impedance();
    void exit_high_impedance();

    // Setup-only helpers. setup()'s ActiveScope covers all of them.
    void setup_ts_control();
    uint8_t write_battery_voltage_control(float volt);
    uint8_t write_termination_control(float mA, bool enable_termination = true);
    uint8_t write_uvlo_ilim(ilim_uvlo param);
    uint8_t write_LDO_voltage_control(float volt);

    bool readReg(uint8_t reg, uint8_t * buffer, uint16_t len);
    void writeReg(uint8_t reg, uint8_t * buffer, uint16_t len);

    const int address = DT_REG_ADDR(DT_NODELABEL(bq25120a));

    uint64_t last_i2c;
    uint64_t last_high_z;

    struct k_mutex active_mutex;
    uint32_t active_count = 0;

    TWIM *_i2c;

    gpio_callback power_connect_cb_data;
    gpio_callback int_cb_data;

    const struct gpio_dt_spec pg_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(bq25120a), pg_gpios);
    const struct gpio_dt_spec cd_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(bq25120a), cd_gpios);
    const struct gpio_dt_spec int_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(bq25120a), int_gpios);
};

extern BQ25120a battery_controller;

#endif