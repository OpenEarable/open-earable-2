#include "BQ25120a.h"

#include "openearable_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bq25120a, LOG_LEVEL_DBG);

BQ25120a battery_controller(&I2C1);

BQ25120a::BQ25120a(TWIM * i2c) : _i2c(i2c) { //, load_switch(LoadSwitch(GPIO_DT_SPEC_GET(DT_NODELABEL(bq25120a), lsctrl_gpios))) {

}

int BQ25120a::begin() {
        int ret;

        ret = device_is_ready(pg_pin.port); //bool
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

        ret = gpio_pin_configure_dt(&cd_pin, GPIO_OUTPUT_INACTIVE);
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
        uint8_t val = 0x80;
        writeReg(registers::ILIM_UVLO, &val, sizeof(val));

        k_usleep(1000); //TODO: check value

        return 0;
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
        uint64_t now = micros();
        int delay = MIN(BQ25120a_I2C_TIMEOUT_US - (int)(now - last_i2c), BQ25120a_I2C_TIMEOUT_US);
        int delay_hz = MIN(BQ25120a_HIGH_Z_TIMEOUT_US - (int)(now - last_high_z), BQ25120a_HIGH_Z_TIMEOUT_US);

        delay = MAX(delay, delay_hz);

        if (delay > 0) k_usleep(delay);

        _i2c->aquire();

        ret = i2c_burst_read(_i2c->master, address, reg, buffer, len);
        if (ret) LOG_WRN("I2C read failed: %d\n", ret);

        _i2c->release();

        last_i2c = micros();

        return ret == 0;
}

void BQ25120a::writeReg(uint8_t reg, uint8_t *buffer, uint16_t len) {
        int ret;
        uint64_t now = micros();
        int delay = MIN(BQ25120a_I2C_TIMEOUT_US - (int)(now - last_i2c), BQ25120a_I2C_TIMEOUT_US);
        int delay_hz = MIN(BQ25120a_HIGH_Z_TIMEOUT_US - (int)(now - last_high_z), BQ25120a_HIGH_Z_TIMEOUT_US);

        delay = MAX(delay, delay_hz);

        if (delay > 0) k_usleep(delay);  //TODO: assert no message ?

        _i2c->aquire();

        ret = i2c_burst_write(_i2c->master, address, reg, buffer, len);
        if (ret) LOG_WRN("I2C write failed: %d", ret);

        _i2c->release();

        last_i2c = micros();
}

void BQ25120a::setup(const battery_settings &_battery_settings) {
        ilim_uvlo params;
        params.lim_mA = _battery_settings.i_max;
        params.uvlo_v = _battery_settings.u_vlo;

        exit_high_impedance();

        //reset();
        setup_ts_control();
        write_battery_voltage_control(_battery_settings.u_term);
        write_charging_control(_battery_settings.i_charge);
        write_termination_control(_battery_settings.i_term);
        write_LDO_voltage_control(3.3);
        write_uvlo_ilim(params);

        enter_high_impedance();
}

uint8_t BQ25120a::read_charging_state() {
        uint8_t status = 0;
        bool ret = readReg(registers::CTRL, (uint8_t *) &status, sizeof(status));

        return status;
}

uint8_t BQ25120a::read_fault() {
        uint8_t status = 0;
        bool ret = readReg(registers::FAULT, (uint8_t *) &status, sizeof(status));

        return status;
}

uint8_t BQ25120a::read_ts_fault() {
        uint8_t status = 0;
        bool ret = readReg(registers::TS_FAULT, (uint8_t *) &status, sizeof(status));

        return status;
}

chrg_state BQ25120a::read_charging_control() {
        uint8_t status = 0;
        bool ret = readReg(registers::CHARGE_CTRL, (uint8_t *) &status, sizeof(status));

        chrg_state chrg;

        chrg.enabled = !(status & 0x2);
        chrg.high_impedance = status & 0x1;

        // charger disabled
        if (!chrg.enabled) return chrg;

        float mAh = (status & 0x7F) >> 2;

        if (status & (1 << 7)) {
                mAh = 40 + mAh * 10;
        } else {
                mAh += 5;
        }

        chrg.mAh = mAh;

        return chrg;
}


uint8_t BQ25120a::write_charging_control(float mA) {
        uint8_t status = 0;
        bool ret = readReg(registers::CHARGE_CTRL, &status, sizeof(status));

        status &= 0x3;

        if (mA >= 40) {
                if (mA > 300) mA = 300;
                status |= (((uint16_t)((mA - 40) / 10 + EPS)) & 0x1F) << 2;
                status |= 1 << 7;
        } else {
                if (mA > 35) mA = 35;
                status |= (((uint16_t)(mA - 5)) & 0x1F) << 2;
        }

        writeReg(registers::CHARGE_CTRL, &status, sizeof(status));

        return status;
}


uint8_t BQ25120a::write_LS_control(bool enable) {
        uint8_t status = 0;

        readReg(registers::LS_LDO_CTRL, &status, sizeof(status));

        uint8_t ls_bit = enable ? 1 : 0;

        status &= ~(1 << 7);
        status |= ls_bit << 7;

        writeReg(registers::LS_LDO_CTRL, &status, sizeof(status));

        return status;
}

uint8_t BQ25120a::write_LDO_voltage_control(float volt) {
        uint8_t status = 0;

        if (volt > 10) volt /= 1000;

        volt = CLAMP(volt, 0.8f, 3.3f);

        readReg(registers::LS_LDO_CTRL, &status, sizeof(status));

        //status |= (((uint16_t)((volt - 0.8) * 10)) & 0x1F) << 2;
        status &= 1 << 7;
        status |= ((uint8_t)((volt - 0.8f) * 10 + EPS)) << 2;
        //status |= 1 << 7;

        writeReg(registers::LS_LDO_CTRL, &status, sizeof(status));

        return status;
}

float BQ25120a::read_ldo_voltage() {
        uint8_t status = 0;
        bool ret = readReg(registers::LS_LDO_CTRL, (uint8_t *) &status, sizeof(status));

        float voltage = 0.8f + ((status >> 2 & 0x1F)) * 0.1f;

        return voltage;
}

float BQ25120a::read_battery_voltage_control() {
        uint8_t status = 0;
        bool ret = readReg(registers::BAT_VOL_CTRL, (uint8_t *) &status, sizeof(status));

        float voltage = 3.6f + (status >> 1) * 0.01f;

        return voltage;
}


uint8_t BQ25120a::write_battery_voltage_control(float volt) {
        uint8_t status = 0;

        if (volt > 10) volt /= 1000;

        volt = CLAMP(volt, 3.6f, 4.65f);

        status |= (((uint16_t)((volt - 3.6f) * 100 + EPS)) & 0x7F) << 1;

        writeReg(registers::BAT_VOL_CTRL, &status, sizeof(status));

        return status;
}

chrg_state BQ25120a::read_termination_control() {
        uint8_t status = 0;
        bool ret = readReg(registers::TERM_CTRL, (uint8_t *) &status, sizeof(status));

        struct chrg_state chrg;

        // if (!ret) printk("failed to read\n");

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

uint8_t BQ25120a::write_termination_control(float mA, bool enable_termination) {
        uint8_t status = 0;

        //bool ret = readReg(registers::TERM_CTRL, &status, sizeof(status));
        //status &= 0x3;

        if (mA >= 6) {
                if (mA > 37) mA = 37;
                status |= (((uint16_t)(mA - 6)) & 0x1F) << 2;
                status |= 1 << 7;
        } else {
                if (mA > 5) mA = 5;
                status |= (((uint16_t)(2 * (mA - 0.5))) & 0x1F) << 2;
        }

        if (enable_termination) {
                status |= 0x2; // enable termination
        }

        writeReg(registers::TERM_CTRL, &status, sizeof(status));

        return status;
}

ilim_uvlo BQ25120a::read_uvlo_ilim() {
        struct ilim_uvlo param;
        uint8_t status = 0;

        bool ret = readReg(registers::ILIM_UVLO, (uint8_t *) &status, sizeof(status));

        // if (!ret) printk("failed to read\n");

        param.uvlo_v = CLAMP(3.0f- 0.2f * ((status & 0x7) - 2), 2.2, 3.0);
        param.lim_mA = 50.f + 50.f * ((status >> 3) & 0x7);

        return param;
}

uint8_t BQ25120a::write_uvlo_ilim(ilim_uvlo param) {
        float mA = CLAMP(param.lim_mA, 50, 400);
        float v = CLAMP(param.uvlo_v, 2.2, 3.0);

        uint8_t status = 0;

        status |= ((uint16_t)(mA / 50 - 1) & 0x7) << 3;
        status |= ((uint16_t)((3.0 - v) * 5 + 2) & 0x7);

        writeReg(registers::ILIM_UVLO, &status, sizeof(status));

        return status;
}

void BQ25120a::setup_ts_control() {
        uint8_t ts_fault = 0;

        writeReg(registers::TS_FAULT, (uint8_t *) &ts_fault, sizeof(ts_fault));
}

void BQ25120a::disable_ts() {
        uint8_t ts_fault = read_ts_fault();

        ts_fault &= ~(1 << 7);

        writeReg(registers::TS_FAULT, (uint8_t *) &ts_fault, sizeof(ts_fault));
}

bool BQ25120a::power_connected() {
        int pg = gpio_pin_get_dt(&pg_pin);
        return pg;
}

void BQ25120a::enter_high_impedance() {
        if (!power_connected()) gpio_pin_set_dt(&cd_pin, 0);
}

void BQ25120a::exit_high_impedance() {
        if (!power_connected()) {
                gpio_pin_set_dt(&cd_pin, 1);
                last_high_z = micros();
        }
}

void BQ25120a::disable_charge() {
        if (power_connected()) gpio_pin_set_dt(&cd_pin, 1);
}

void BQ25120a::enable_charge() {
        if (power_connected()) gpio_pin_set_dt(&cd_pin, 0);
}


button_state BQ25120a::read_button_state() {
        struct button_state btn;

        uint8_t status = 0;
        bool ret = readReg(registers::BTN_CTRL, (uint8_t *) &status, sizeof(status));

        // if (!ret) printk("failed to read\n");

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