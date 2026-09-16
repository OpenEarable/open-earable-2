#include "BQ25120a.h"

#include "openearable_common.h"

#include <errno.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bq25120a, LOG_LEVEL_DBG);

namespace {
K_MUTEX_DEFINE(charger_mutex);

// CD changes and register read/modify/write must not interleave. Zephyr mutexes
// are recursive so setup can call the same checked accessors as normal service.
class ChargerLock {
public:
        ChargerLock() { k_mutex_lock(&charger_mutex, K_FOREVER); }
        ~ChargerLock() { k_mutex_unlock(&charger_mutex); }
};

// Setup clears stale CE/HZ bits; normal current changes preserve them.
uint8_t encode_charge_current(float mA) {
        if (mA >= 40) {
                return (static_cast<uint8_t>((mA - 40) / 10 + EPS) << 2) | BIT(7);
        }
        return static_cast<uint8_t>(MIN(mA, 35.0f) - 5 + EPS) << 2;
}
}

BQ25120a battery_controller(&I2C1);

BQ25120a::BQ25120a(TWIM * i2c) : _i2c(i2c) { //, load_switch(LoadSwitch(GPIO_DT_SPEC_GET(DT_NODELABEL(bq25120a), lsctrl_gpios))) {

}

int BQ25120a::begin() {
        int ret;

        ret = device_is_ready(pg_pin.port) && device_is_ready(int_pin.port) &&
                device_is_ready(cd_pin.port) && device_is_ready(_i2c->master); //bool
        if (!ret) {
                LOG_ERR("BQ25120a pins not ready.\n");
                return -1;
        }

        ret = gpio_pin_configure_dt(&pg_pin, GPIO_INPUT);
	if (ret != 0) {
                LOG_ERR("Failed to set PG as input.\n");
                return ret;
        }

        ret = gpio_pin_configure_dt(&int_pin, GPIO_INPUT);
	if (ret != 0) {
                LOG_ERR("Failed to set INT as input.\n");
                return ret;
        }

        // USB must not charge until the gauge and charger settings are checked.
        charge_disabled = true;
        configured = false;
        ret = gpio_pin_configure_dt(&cd_pin,
                power_connected() ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
                LOG_ERR("Failed to set GPOUT as input.\n");
                return ret;
        }

        ret = gpio_pin_interrupt_configure_dt(&pg_pin, GPIO_INT_EDGE_BOTH);
        if (ret != 0) {
                LOG_ERR("Failed to setup interrupt on PG.\n");
                return ret;
        }

        ret = gpio_pin_interrupt_configure_dt(&int_pin, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret != 0) {
                LOG_ERR("Failed to setup interrupt on INT: ERROR -%i.\n", ret);
                return ret;
        }

        _i2c->begin();

        uint64_t now = micros();
        last_i2c = last_high_z = now;

        return 0;
}

int BQ25120a::reset() {
        // Register reset loses the safety profile; keep charging inhibited.
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
        int ret;

        ret = device_is_ready(pg_pin.port); //bool
        if (!ret) {
                LOG_ERR("BQ25120a pins not ready.\n");
                return -1;
        }

        ret = gpio_pin_interrupt_configure_dt(&pg_pin, GPIO_INT_LEVEL_ACTIVE);
        if (ret != 0) {
                LOG_ERR("Failed to setup interrupt on PG.\n");
                return ret;
        }

        ret = gpio_pin_interrupt_configure_dt(&int_pin, GPIO_INT_LEVEL_ACTIVE);
        if (ret != 0) {
                LOG_ERR("Failed to setup interrupt on INT.\n");
                return ret;
        }

        return 0;
}

bool BQ25120a::readReg(uint8_t reg, uint8_t * buffer, uint16_t len) {
        int ret;
        // Hold CD and the quiet interval stable throughout the transfer.
        ChargerLock lock;
        _i2c->aquire();
        wait_for_i2c();

        ret = i2c_burst_read(_i2c->master, address, reg, buffer, len);
        if (ret) LOG_WRN("I2C read failed: %d\n", ret);

        last_i2c = micros();
        _i2c->release();

        return ret == 0;
}

bool BQ25120a::writeReg(uint8_t reg, const uint8_t *buffer, uint16_t len) {
        int ret;
        // Serialize the quiet period with the transfer; a failed write is not setup.
        ChargerLock lock;
        _i2c->aquire();
        wait_for_i2c();

        ret = i2c_burst_write(_i2c->master, address, reg, buffer, len);
        if (ret) LOG_WRN("I2C write failed: %d", ret);

        last_i2c = micros();
        _i2c->release();

        return ret == 0;
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

uint8_t BQ25120a::read_charging_state() {
        uint8_t status = 0;
        bool ret = readReg(registers::CTRL, (uint8_t *) &status, sizeof(status));

        // Failed reads must report a fault, never a healthy charger.
        return ret ? status : 0xC0;
}

uint8_t BQ25120a::read_fault() {
        uint8_t status = 0;
        bool ret = readReg(registers::FAULT, (uint8_t *) &status, sizeof(status));

        // Failed reads must report a fault, never a healthy charger.
        return ret ? status : 0xF0;
}

uint8_t BQ25120a::read_ts_fault() {
        uint8_t status = 0;
        bool ret = readReg(registers::TS_FAULT, (uint8_t *) &status, sizeof(status));

        // Failed reads must report a fault, never a healthy charger.
        return ret ? status : 0xA0;
}

chrg_state BQ25120a::read_charging_control() {
        uint8_t status = 0;
        bool ret = readReg(registers::CHARGE_CTRL, (uint8_t *) &status, sizeof(status));

        chrg_state chrg;
        // Preserve disabled defaults when the register cannot be read.
        if (!ret) return chrg;

        chrg.enabled = !(status & 0x2);
        chrg.high_impedance = status & 0x1;

        // charger disabled
        if (!chrg.enabled) return chrg;

        float mAh = (status & 0x7F) >> 2;

        if (status & (1 << 7)) {
                mAh = MIN(40 + mAh * 10, 300);
        } else {
                mAh = MIN(mAh + 5, 35);
        }

        chrg.mAh = mAh;

        return chrg;
}

int BQ25120a::write_charging_control(float mA) {
        // Reject invalid currents and failed reads before changing charge control.
        ChargerLock lock;
        if (!isfinite(mA) || mA < 5 || mA > 300) return -EINVAL;
        uint8_t status = 0;
        bool ret = readReg(registers::CHARGE_CTRL, &status, sizeof(status));

        if (!ret) return -EIO;
        status &= 0x3;

        if (mA >= 40) {
                if (mA > 300) mA = 300;
                status |= (((uint16_t)((mA - 40) / 10 + EPS)) & 0x1F) << 2;
                status |= 1 << 7;
        } else {
                if (mA > 35) mA = 35;
                status |= (((uint16_t)(mA - 5 + EPS)) & 0x1F) << 2;
        }

        return write_verified(registers::CHARGE_CTRL, status);
}

int BQ25120a::write_LS_control(bool enable) {
        // Boot needs confirmed rail disable before changing its voltage.
        ChargerLock lock;
        uint8_t status = 0;

        if (!readReg(registers::LS_LDO_CTRL, &status, sizeof(status))) return -EIO;

        uint8_t ls_bit = enable ? 1 : 0;

        status &= ~(1 << 7);
        status |= ls_bit << 7;

        return write_verified(registers::LS_LDO_CTRL, status, 0xFD);
}

int BQ25120a::write_LDO_voltage_control(float volt) {
        ChargerLock lock;
        uint8_t status = 0;

        if (volt > 10) volt /= 1000;

        // Invalid voltages must not silently become pass-through mode.
        if (!isfinite(volt) || volt < 0.8f || volt > 3.3f) return -EINVAL;

        if (!readReg(registers::LS_LDO_CTRL, &status, sizeof(status))) return -EIO;
        uint8_t previous = status;

        status &= 0x81; // Preserve enable and MR reset behavior.
        status |= ((uint8_t)((volt - 0.8f) * 10 + EPS)) << 2;
        if ((previous & 0x7C) == (status & 0x7C)) return status;
        // The IC ignores voltage changes while enabled. Report failure instead
        // of interrupting a live rail; setup_boot disables both enable sources.
        if (previous & (1 << 7)) return -EBUSY;
        return write_verified(registers::LS_LDO_CTRL, status, 0xFD);
}

float BQ25120a::read_ldo_voltage() {
        uint8_t status = 0;
        bool ret = readReg(registers::LS_LDO_CTRL, (uint8_t *) &status, sizeof(status));

        // Pass-through is not a regulated voltage; failed reads are unknown.
        if (!ret || ((status >> 2) & 0x1F) > 25) return NAN;

        float voltage = 0.8f + ((status >> 2 & 0x1F)) * 0.1f;

        return voltage;
}

float BQ25120a::read_battery_voltage_control() {
        uint8_t status = 0;
        bool ret = readReg(registers::BAT_VOL_CTRL, (uint8_t *) &status, sizeof(status));

        // A failed read is not a valid charge-voltage setting.
        if (!ret) return NAN;

        float voltage = 3.6f + (status >> 1) * 0.01f;

        return MIN(voltage, 4.65f);
}

int BQ25120a::write_battery_voltage_control(float volt) {
        uint8_t status = 0;

        if (volt > 10) volt /= 1000;

        // Refuse invalid settings rather than silently raising the cell limit.
        if (!isfinite(volt) || volt < 3.6f || volt > 4.65f) return -EINVAL;

        status |= (((uint16_t)((volt - 3.6f) * 100 + EPS)) & 0x7F) << 1;

        return write_verified(registers::BAT_VOL_CTRL, status);
}

chrg_state BQ25120a::read_termination_control() {
        uint8_t status = 0;
        bool ret = readReg(registers::TERM_CTRL, (uint8_t *) &status, sizeof(status));

        struct chrg_state chrg;

        // Preserve disabled defaults on a failed read.
        if (!ret) return chrg;

        chrg.enabled = status & 0x2;
        //chrg.high_impedance = status & 0x1;

        // charger disabled
        if (!chrg.enabled) return chrg;

        float mAh = (status & 0x7F) >> 2;

        if (status & (1 << 7)) {
                mAh = 6 + mAh * 1;
        } else {
                mAh = 0.5 + mAh * 0.5;
        }

        chrg.mAh = mAh;

        return chrg;
}

int BQ25120a::write_termination_control(float mA, bool enable_termination) {
        // Underflow would encode an unsafe precharge/termination current.
        if (!isfinite(mA) || mA < 0.5f || mA > 37) return -EINVAL;
        uint8_t status = 0;

        //bool ret = readReg(registers::TERM_CTRL, &status, sizeof(status));
        //status &= 0x3;

        if (mA >= 6) {
                if (mA > 37) mA = 37;
                status |= (((uint16_t)(mA - 6 + EPS)) & 0x1F) << 2;
                status |= 1 << 7;
        } else {
                if (mA > 5) mA = 5;
                status |= (((uint16_t)(2 * (mA - 0.5) + EPS)) & 0x1F) << 2;
        }

        if (enable_termination) {
                status |= 0x2; // enable termination
        }

        return write_verified(registers::TERM_CTRL, status);
}

ilim_uvlo BQ25120a::read_uvlo_ilim() {
        struct ilim_uvlo param;
        uint8_t status = 0;

        bool ret = readReg(registers::ILIM_UVLO, (uint8_t *) &status, sizeof(status));

        // Do not report a successful cutoff setting after an I2C error.
        if (!ret) return param;

        param.uvlo_v = CLAMP(3.0f- 0.2f * ((status & 0x7) - 2), 2.2, 3.0);
        param.lim_mA = 50.f + 50.f * ((status >> 3) & 0x7);

        return param;
}

int BQ25120a::write_uvlo_ilim(ilim_uvlo param) {
        // Reject invalid cutoff/current limits instead of wrapping their codes.
        if (!isfinite(param.lim_mA) || param.lim_mA < 50 || param.lim_mA > 400 ||
                !isfinite(param.uvlo_v) || param.uvlo_v < 2.2f || param.uvlo_v > 3.0f) return -EINVAL;
        float mA = param.lim_mA;
        float v = param.uvlo_v;

        uint8_t status = 0;

        status |= ((uint16_t)(mA / 50 - 1) & 0x7) << 3;
        status |= ((uint16_t)((3.0f - v) * 5 + EPS + 2) & 0x7);

        return write_verified(registers::ILIM_UVLO, status, 0x3F);
}

int BQ25120a::setup_ts_control() {
        uint8_t ts_fault = 0;

        // v2.7 has a fixed divider, not an NTC: verify TS stays disabled.
        // EN_INT=0 also keeps the fault/button interrupt pulsed, not held low.
        return write_verified(registers::TS_FAULT, ts_fault, 0x8F);
}

void BQ25120a::disable_ts() {
        // Do not write back a fabricated status after an I2C failure.
        ChargerLock lock;
        uint8_t ts_fault;
        if (!readReg(registers::TS_FAULT, &ts_fault, sizeof(ts_fault))) return;

        ts_fault &= ~(1 << 7);

        write_verified(registers::TS_FAULT, ts_fault, 0x8F);
}

bool BQ25120a::power_connected() {
        int pg = gpio_pin_get_dt(&pg_pin);
        return pg > 0; // GPIO errors must not be interpreted as USB power.
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
        struct button_state btn;

        uint8_t status = 0;
        bool ret = readReg(registers::BTN_CTRL, (uint8_t *) &status, sizeof(status));

        // An unreadable register must not synthesize a button press.
        if (!ret) return btn;

        btn.wake_1 = status & 0x2;
        btn.wake_2 = status & 0x1;

        return btn;
}

int BQ25120a::set_power_connect_callback(gpio_callback_handler_t handler) {
    gpio_init_callback(&power_connect_cb_data, handler, power_connect_cb_data.pin_mask | BIT(pg_pin.pin));
    return gpio_add_callback(pg_pin.port, &power_connect_cb_data);
}

int BQ25120a::set_int_callback(gpio_callback_handler_t handler) {
    gpio_init_callback(&int_cb_data, handler, int_cb_data.pin_mask | BIT(int_pin.pin));
    return gpio_add_callback(int_pin.port, &int_cb_data);
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

int BQ25120a::write_verified(uint8_t reg, uint8_t value, uint8_t mask) {
        // An acknowledged write can still be ignored by the IC (notably LS/LDO).
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
        if (!ok || (status & BIT(3))) return -EIO;
        return restore;
}

int BQ25120a::enter_ship_mode() {
        ChargerLock lock;
        if (power_connected()) return -EBUSY;
        int ret = set_cd(true);
        if (ret) return ret;
        const uint8_t command = BIT(5);
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
        // Publish only a complete read, and preserve intentional CD inhibition.
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
        // The PMIC watchdog can reset these registers while the CPU keeps running.
        ChargerLock lock;
        if (!configured) return false;
        if (!isfinite(current) || current < 5 || current > 300) {
                configured = false;
                disable_charge();
                return false;
        }
        uint8_t term = settings.i_term >= 6 ?
                ((uint8_t)(settings.i_term - 6 + EPS) << 2) | 0x80 :
                (uint8_t)(2 * (MIN(settings.i_term, 5.0f) - 0.5f) + EPS) << 2;
        term |= 0x02;
        uint8_t voltage = (uint8_t)((settings.u_term - 3.6f) * 100 + EPS) << 1;
        uint8_t limits = ((uint8_t)(settings.i_max / 50 - 1) << 3) |
                ((uint8_t)((3.0f - settings.u_vlo) * 5 + EPS) + 2);
        const struct {
                uint8_t reg;
                uint8_t value;
                uint8_t mask;
        } expected[] = {
                {registers::TS_FAULT, 0, 0x8F},
                {registers::CHARGE_CTRL, encode_charge_current(current), 0xFF},
                {registers::TERM_CTRL, term, 0xFF},
                {registers::BAT_VOL_CTRL, voltage, 0xFF},
                {registers::LS_LDO_CTRL, 0x64, 0x7C},
                {registers::ILIM_UVLO, limits, 0x3F},
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

int BQ25120a::set_cd(bool high) {
        // I2C needs 1 ms after CD wakes the battery-only device.
        int ret = gpio_pin_set_dt(&cd_pin, high);
        if (!ret && high) last_high_z = micros();
        return ret;
}
