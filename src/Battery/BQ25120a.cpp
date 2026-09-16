#include "BQ25120a.h"

#include <errno.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bq25120a, LOG_LEVEL_DBG);

namespace {
K_MUTEX_DEFINE(charger_mutex);

// Keep register read/modify/write sequences and CD transitions together. Zephyr
// mutexes are recursive, so setup can use the same checked public accessors.
class ChargerLock {
public:
    ChargerLock() { k_mutex_lock(&charger_mutex, K_FOREVER); }
    ~ChargerLock() { k_mutex_unlock(&charger_mutex); }
};

constexpr uint8_t CHARGE_DISABLED = BIT(1);
constexpr uint8_t HIGH_IMPEDANCE = BIT(0);
constexpr uint8_t SAFETY_TIMER_FAULT = BIT(3);
constexpr uint8_t SHIP_MODE = BIT(5);
constexpr uint8_t TS_ENABLED = BIT(7);
// v2.7 has a fixed 57.7% VIN divider, not a thermistor. This lies inside
// TSOFF's 55%-60% tolerance band and can otherwise be reported as cold.
// Battery temperature protection is enforced from checked fuel-gauge data.
constexpr uint8_t TS_CONFIG = 0;

uint8_t encode_charge_current(float mA) {
    if (mA >= 40) {
        return (static_cast<uint8_t>((mA - 40) / 10 + EPS) << 2) | BIT(7);
    }
    return static_cast<uint8_t>(MIN(mA, 35.0f) - 5 + EPS) << 2;
}

uint8_t encode_termination_current(float mA) {
    if (mA >= 6) return (static_cast<uint8_t>(mA - 6 + EPS) << 2) | BIT(7);
    return static_cast<uint8_t>(2 * (MIN(mA, 5.0f) - 0.5f) + EPS) << 2;
}

uint8_t encode_voltage(float volt) {
    return static_cast<uint8_t>((volt - 3.6f) * 100 + EPS) << 1;
}

uint8_t encode_input_uvlo(float mA, float volt) {
    return (static_cast<uint8_t>(mA / 50 - 1) << 3) |
        (static_cast<uint8_t>((3.0f - volt) * 5 + EPS) + 2);
}
}

BQ25120a battery_controller(&I2C1);

BQ25120a::BQ25120a(TWIM *i2c) : _i2c(i2c) {}

int BQ25120a::begin() {
    if (!device_is_ready(pg_pin.port) || !device_is_ready(int_pin.port) ||
        !device_is_ready(cd_pin.port) || !device_is_ready(_i2c->master)) {
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&pg_pin, GPIO_INPUT);
    if (ret) return ret;
    ret = gpio_pin_configure_dt(&int_pin, GPIO_INPUT);
    if (ret) return ret;

    // CD low is the safe sleep state on battery; with USB present inhibit
    // charging until the fuel gauge's voltage and temperature have been read.
    charge_disabled = true;
    configured = false;
    ret = gpio_pin_configure_dt(&cd_pin,
        power_connected() ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    ret = gpio_pin_interrupt_configure_dt(&pg_pin, GPIO_INT_EDGE_BOTH);
    if (ret) return ret;
    ret = gpio_pin_interrupt_configure_dt(&int_pin, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) return ret;

    _i2c->begin();
    last_i2c = last_high_z = micros();
    return 0;
}

int BQ25120a::reset() {
    ChargerLock lock;
    configured = false;
    int ret = disable_charge();
    if (ret) return ret;
    ret = exit_high_impedance();
    if (ret) return ret;
    const uint8_t value = BIT(7);
    bool ok = writeReg(registers::ILIM_UVLO, &value, sizeof(value));
    k_usleep(BQ25120a_HIGH_Z_TIMEOUT_US);
    int restore = enter_high_impedance();
    return ok ? restore : -EIO;
}

int BQ25120a::set_wakeup_int() {
    int ret = gpio_pin_interrupt_configure_dt(&pg_pin, GPIO_INT_LEVEL_ACTIVE);
    if (ret) return ret;
    return gpio_pin_interrupt_configure_dt(&int_pin, GPIO_INT_LEVEL_ACTIVE);
}

void BQ25120a::wait_for_i2c() {
    const uint64_t now = micros();
    // Compare unsigned elapsed times before narrowing. Casting a long idle time
    // to int could wrap and skip the mandatory inter-transaction quiet period.
    const uint64_t bus_elapsed = now - last_i2c;
    const uint64_t wake_elapsed = now - last_high_z;
    uint32_t delay = bus_elapsed < BQ25120a_I2C_TIMEOUT_US ?
        BQ25120a_I2C_TIMEOUT_US - bus_elapsed : 0;
    if (wake_elapsed < BQ25120a_HIGH_Z_TIMEOUT_US) {
        delay = MAX(delay, BQ25120a_HIGH_Z_TIMEOUT_US - wake_elapsed);
    }
    if (delay) k_usleep(delay);
}

bool BQ25120a::readReg(uint8_t reg, uint8_t *buffer, uint16_t len) {
    ChargerLock lock;
    _i2c->aquire();
    wait_for_i2c();
    int ret = i2c_burst_read(_i2c->master, address, reg, buffer, len);
    last_i2c = micros();
    _i2c->release();
    if (ret) LOG_WRN("Charger register 0x%02x read failed: %d", reg, ret);
    return ret == 0;
}

bool BQ25120a::writeReg(uint8_t reg, const uint8_t *buffer, uint16_t len) {
    ChargerLock lock;
    _i2c->aquire();
    wait_for_i2c();
    int ret = i2c_burst_write(_i2c->master, address, reg, buffer, len);
    last_i2c = micros();
    _i2c->release();
    if (ret) LOG_WRN("Charger register 0x%02x write failed: %d", reg, ret);
    return ret == 0;
}

int BQ25120a::write_verified(uint8_t reg, uint8_t value, uint8_t mask) {
    ChargerLock lock;
    uint8_t actual;
    if (!writeReg(reg, &value, 1) || !readReg(reg, &actual, 1)) return -EIO;
    if ((actual & mask) != (value & mask)) {
        LOG_ERR("Charger register 0x%02x expected 0x%02x, read 0x%02x (mask 0x%02x)",
                reg, value, actual, mask);
        return -EIO;
    }
    return value;
}

int BQ25120a::setup_boot(const battery_settings &settings) {
    ChargerLock lock;
    configured = false;
    int ret = disable_charge();
    if (ret) return ret;
    if (!device_is_ready(lsctrl_pin.port)) return -ENODEV;
    // The PMIC ignores LDO voltage writes while either enable source is high.
    // A CPU reset does not reset its registers, and the bootloader may have
    // raised LSCTRL. Establish both off states before programming 3.3 V.
    ret = gpio_pin_configure_dt(&lsctrl_pin, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;
    k_usleep(BQ25120a_HIGH_Z_TIMEOUT_US);
    ret = exit_high_impedance();
    if (ret) return ret;
    ret = write_LS_control(false);
    int restore = enter_high_impedance();
    if (ret < 0) return ret;
    if (restore) return restore;
    return setup(settings);
}

int BQ25120a::setup(const battery_settings &settings) {
    ChargerLock lock;
    configured = false;
    int ret = disable_charge();
    if (ret) return ret;
    ret = exit_high_impedance();
    if (ret) return ret;

    // Do not let invalid settings wrap into a higher charge current or voltage.
    if (!isfinite(settings.i_charge) || settings.i_charge < 5 || settings.i_charge > 300 ||
        !isfinite(settings.i_term) || settings.i_term < 0.5f || settings.i_term > 37 ||
        !isfinite(settings.i_max) || settings.i_max < 50 || settings.i_max > 400 ||
        !isfinite(settings.u_term) || settings.u_term < 3.6f || settings.u_term > 4.65f ||
        !isfinite(settings.u_vlo) || settings.u_vlo < 2.2f || settings.u_vlo > 3.0f) {
        enter_high_impedance();
        return -EINVAL;
    }

    ilim_uvlo params;
    params.lim_mA = settings.i_max;
    params.uvlo_v = settings.u_vlo;
    // USB can arrive between the ship-mode PG check and the command write.
    // Clear any pending ship request before later USB removal can act on it.
    const uint8_t normal_mode = 0;
    ret = writeReg(registers::CTRL, &normal_mode, 1) ? 0 : -EIO;
    if (ret >= 0) ret = setup_ts_control();
    if (ret >= 0) ret = write_battery_voltage_control(settings.u_term);
    if (ret >= 0) ret = write_termination_control(settings.i_term);
    if (ret >= 0) ret = write_uvlo_ilim(params);
    // Keep the three-hour safety timer and enable its documented 2x slowdown
    // during input/system-load/temperature current limiting (SLUSD08A 9.3.15).
    if (ret >= 0) ret = write_verified(registers::TIMERS, 0x4A, 0xFE);
    // Explicitly clear stale CE/HZ bits. Rewriting HZ=1 before clearing it could
    // disable I2C and prevent the recovery write on battery power.
    if (ret >= 0) ret = write_verified(registers::CHARGE_CTRL,
        encode_charge_current(settings.i_charge));
    if (ret >= 0) ret = write_LDO_voltage_control(3.3f);

    int restore = enter_high_impedance();
    if (ret < 0) return ret;
    if (restore) return restore;
    configured = true;
    return 0;
}

int BQ25120a::recover_charging(const battery_settings &settings) {
    ChargerLock lock;
    if (!power_connected()) return -ENODEV;
    // A register reset does NOT clear TIMER. The documented recovery is a CD
    // pulse; the power manager must bound attempts and check physical faults.
    int ret = disable_charge();
    if (ret) return ret;
    k_usleep(BQ25120a_HIGH_Z_TIMEOUT_US);
    ret = setup(settings);
    if (ret) return ret;
    if (!power_connected()) return -ENODEV;
    ret = set_cd(false);
    if (ret) return ret;
    k_usleep(BQ25120a_HIGH_Z_TIMEOUT_US);
    uint8_t status;
    bool ok = readReg(registers::CTRL, &status, 1);
    int restore = disable_charge();
    if (!ok || (status & SAFETY_TIMER_FAULT)) return -EIO;
    return restore;
}

int BQ25120a::enter_ship_mode() {
    ChargerLock lock;
    if (power_connected()) return -EBUSY;
    int ret = set_cd(true);
    if (ret) return ret;
    const uint8_t command = SHIP_MODE;
    if (!writeReg(registers::CTRL, &command, 1)) {
        enter_high_impedance();
        return -EIO;
    }
    // Leave CD high. MR release completes ship entry after tQUIET; the host
    // supply is then removed. Hi-Z would leave PMID/SYS and their loads powered.
    k_usleep(BQ25120a_HIGH_Z_TIMEOUT_US);
    return 0;
}

bool BQ25120a::read_status(uint8_t &control, uint8_t &fault, uint8_t &ts_fault) {
    ChargerLock lock;
    uint8_t values[3];
    if (exit_high_impedance()) return false;
    bool ok = readReg(registers::CTRL, &values[0], 1) &&
        readReg(registers::FAULT, &values[1], 1) &&
        readReg(registers::TS_FAULT, &values[2], 1);
    if (enter_high_impedance()) {
        configured = false;
        disable_charge();
        ok = false;
    }
    if (!ok) return false;
    control = values[0];
    fault = values[1];
    ts_fault = values[2];
    return true;
}

bool BQ25120a::configuration_valid(const battery_settings &settings, float current) {
    ChargerLock lock;
    if (!configured) return false;
    if (!isfinite(current) || current < 5 || current > 300) {
        configured = false;
        disable_charge();
        return false;
    }
    const struct {
        uint8_t reg;
        uint8_t value;
        uint8_t mask;
    } expected[] = {
        {registers::TS_FAULT, TS_CONFIG, 0x8F},
        {registers::CHARGE_CTRL, encode_charge_current(current), 0xFF},
        {registers::TERM_CTRL, static_cast<uint8_t>(encode_termination_current(settings.i_term) | BIT(1)), 0xFF},
        {registers::BAT_VOL_CTRL, encode_voltage(settings.u_term), 0xFF},
        {registers::LS_LDO_CTRL, 0x64, 0x7C},
        {registers::ILIM_UVLO, encode_input_uvlo(settings.i_max, settings.u_vlo), 0x3F},
        {registers::TIMERS, 0x4A, 0xFE},
    };
    bool ok = exit_high_impedance() == 0;
    for (const auto &item : expected) {
        uint8_t actual;
        if (!ok || !readReg(item.reg, &actual, 1) ||
            (actual & item.mask) != (item.value & item.mask)) {
            ok = false;
            break;
        }
    }
    if (!ok) {
        configured = false;
        disable_charge();
    }
    if (enter_high_impedance()) {
        configured = false;
        disable_charge();
        ok = false;
    }
    return ok;
}

uint8_t BQ25120a::read_charging_state() {
    uint8_t status;
    return readReg(registers::CTRL, &status, 1) ? status : 0xC0;
}

uint8_t BQ25120a::read_fault() {
    uint8_t status;
    return readReg(registers::FAULT, &status, 1) ? status : 0xF0;
}

uint8_t BQ25120a::read_ts_fault() {
    uint8_t status;
    return readReg(registers::TS_FAULT, &status, 1) ? status : 0xA0;
}

chrg_state BQ25120a::read_charging_control() {
    uint8_t status;
    chrg_state charge;
    if (!readReg(registers::CHARGE_CTRL, &status, 1)) return charge;
    charge.enabled = !(status & CHARGE_DISABLED);
    charge.high_impedance = status & HIGH_IMPEDANCE;
    unsigned code = (status >> 2) & 0x1F;
    charge.mAh = status & BIT(7) ? MIN(40 + code * 10, 300) : MIN(5 + code, 35);
    return charge;
}

int BQ25120a::write_charging_control(float mA) {
    ChargerLock lock;
    if (!isfinite(mA) || mA < 5 || mA > 300) return -EINVAL;
    uint8_t status;
    if (!readReg(registers::CHARGE_CTRL, &status, 1)) return -EIO;
    status &= CHARGE_DISABLED | HIGH_IMPEDANCE;
    status |= encode_charge_current(mA);
    return write_verified(registers::CHARGE_CTRL, status);
}

int BQ25120a::write_LS_control(bool enable) {
    ChargerLock lock;
    uint8_t status;
    if (!readReg(registers::LS_LDO_CTRL, &status, 1)) return -EIO;
    status = (status & ~BIT(7)) | (enable ? BIT(7) : 0);
    return write_verified(registers::LS_LDO_CTRL, status, 0xFD);
}

int BQ25120a::write_LDO_voltage_control(float volt) {
    ChargerLock lock;
    if (volt > 10) volt /= 1000;
    if (!isfinite(volt) || volt < 0.8f || volt > 3.3f) return -EINVAL;
    uint8_t status;
    if (!readReg(registers::LS_LDO_CTRL, &status, 1)) return -EIO;
    uint8_t desired = (status & 0x81) |
        (static_cast<uint8_t>((volt - 0.8f) * 10 + EPS) << 2);
    if ((status & 0x7C) == (desired & 0x7C)) return desired;
    // Changing voltage while this output is enabled is ignored by the IC. Do
    // not interrupt a live peripheral supply here; report that setup failed.
    if (status & BIT(7)) return -EBUSY;
    return write_verified(registers::LS_LDO_CTRL, desired, 0xFD);
}

float BQ25120a::read_ldo_voltage() {
    uint8_t status;
    if (!readReg(registers::LS_LDO_CTRL, &status, 1)) return NAN;
    unsigned code = (status >> 2) & 0x1F;
    return code <= 25 ? 0.8f + code * 0.1f : NAN; // >25 means pass-through
}

float BQ25120a::read_battery_voltage_control() {
    uint8_t status;
    if (!readReg(registers::BAT_VOL_CTRL, &status, 1)) return NAN;
    return MIN(3.6f + (status >> 1) * 0.01f, 4.65f);
}

int BQ25120a::write_battery_voltage_control(float volt) {
    if (volt > 10) volt /= 1000;
    if (!isfinite(volt) || volt < 3.6f || volt > 4.65f) return -EINVAL;
    return write_verified(registers::BAT_VOL_CTRL, encode_voltage(volt));
}

chrg_state BQ25120a::read_termination_control() {
    uint8_t status;
    chrg_state charge;
    if (!readReg(registers::TERM_CTRL, &status, 1)) return charge;
    charge.enabled = status & BIT(1);
    unsigned code = (status >> 2) & 0x1F;
    charge.mAh = status & BIT(7) ? 6 + code : 0.5f + code * 0.5f;
    return charge;
}

int BQ25120a::write_termination_control(float mA, bool enable_termination) {
    if (!isfinite(mA) || mA < 0.5f || mA > 37) return -EINVAL;
    uint8_t status = encode_termination_current(mA);
    if (enable_termination) status |= BIT(1);
    return write_verified(registers::TERM_CTRL, status);
}

ilim_uvlo BQ25120a::read_uvlo_ilim() {
    ilim_uvlo param;
    uint8_t status;
    if (!readReg(registers::ILIM_UVLO, &status, 1)) return param;
    param.uvlo_v = CLAMP(3.0f - 0.2f * ((status & 0x7) - 2), 2.2f, 3.0f);
    param.lim_mA = 50.0f + 50.0f * ((status >> 3) & 0x7);
    return param;
}

int BQ25120a::write_uvlo_ilim(ilim_uvlo param) {
    if (!isfinite(param.lim_mA) || param.lim_mA < 50 || param.lim_mA > 400 ||
        !isfinite(param.uvlo_v) || param.uvlo_v < 2.2f || param.uvlo_v > 3.0f) return -EINVAL;
    // Round the cutoff upwards to the supported 200 mV step, never downwards.
    return write_verified(registers::ILIM_UVLO,
        encode_input_uvlo(param.lim_mA, param.uvlo_v), 0x3F);
}

int BQ25120a::setup_ts_control() {
    // Disable the fixed-divider input on v2.7. EN_INT=0 makes INT a pulsed
    // fault/button interrupt instead of a level held low while charging.
    return write_verified(registers::TS_FAULT, TS_CONFIG, 0x8F);
}

void BQ25120a::disable_ts() {
    // Retained for source compatibility; normal setup/recovery never calls it.
    ChargerLock lock;
    uint8_t status;
    if (!readReg(registers::TS_FAULT, &status, 1)) return;
    write_verified(registers::TS_FAULT, status & ~TS_ENABLED, 0x8F);
}

bool BQ25120a::power_connected() {
    return gpio_pin_get_dt(&pg_pin) > 0;
}

int BQ25120a::set_cd(bool high) {
    int ret = gpio_pin_set_dt(&cd_pin, high);
    if (!ret && high) last_high_z = micros();
    return ret;
}

int BQ25120a::enter_high_impedance() {
    ChargerLock lock;
    // Always restore CD even if USB arrived while an I2C access had it high.
    // Preserve an intentional charge inhibit whenever USB is present.
    return set_cd(power_connected() && charge_disabled);
}

int BQ25120a::exit_high_impedance() {
    ChargerLock lock;
    return set_cd(!power_connected() || charge_disabled);
}

int BQ25120a::disable_charge() {
    ChargerLock lock;
    charge_disabled = true;
    return enter_high_impedance();
}

int BQ25120a::enable_charge() {
    ChargerLock lock;
    if (!configured) return -EACCES;
    int ret = set_cd(false);
    if (ret) {
        // A failed GPIO write must not authorize a later status read to drop
        // CD. The manager still considers charging inhibited after this error,
        // and battery conditions may become unsafe before the next poll.
        charge_disabled = true;
        (void)enter_high_impedance();
        return ret;
    }
    charge_disabled = false;
    return 0;
}

button_state BQ25120a::read_button_state() {
    button_state btn;
    uint8_t status;
    if (!readReg(registers::BTN_CTRL, &status, 1)) return btn;
    btn.wake_1 = status & BIT(1);
    btn.wake_2 = status & BIT(0);
    return btn;
}

int BQ25120a::set_power_connect_callback(gpio_callback_handler_t handler) {
    gpio_init_callback(&power_connect_cb_data, handler, BIT(pg_pin.pin));
    return gpio_add_callback(pg_pin.port, &power_connect_cb_data);
}

int BQ25120a::set_int_callback(gpio_callback_handler_t handler) {
    gpio_init_callback(&int_cb_data, handler, BIT(int_pin.pin));
    return gpio_add_callback(int_pin.port, &int_cb_data);
}
