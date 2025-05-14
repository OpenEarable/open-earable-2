#include "BQ27220.h"

#include "openearable_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bq27220, LOG_LEVEL_DBG);

BQ27220 fuel_gauge(&I2C1);

BQ27220::BQ27220(TWIM * i2c) : _i2c(i2c) {
        
}

int BQ27220::begin() {
        int ret;

        ret = device_is_ready(gpout_pin.port); //bool
        if (!ret) {
                LOG_ERR("GPOUT not ready.\n");
                return -1;
        }

        ret = gpio_pin_configure_dt(&gpout_pin, GPIO_INPUT);
	if (ret != 0) {
                LOG_ERR("Failed to set GPOUT as input: ERROR -%i.\n", ret);
                return ret;
        }

        ret = gpio_pin_interrupt_configure_dt(&gpout_pin, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret != 0) {
                LOG_ERR("Failed to setup interrupt on GPOUT: ERROR -%i.\n", ret);
                return ret;
        }

        _i2c->begin();

        last_i2c = micros();

        return 0;
}

int BQ27220::set_wakeup_int() {
        int ret;

        ret = device_is_ready(gpout_pin.port); //bool
        if (!ret) {
                LOG_ERR("GPOUT not ready.\n");
                return -1;
        }

        ret = gpio_pin_interrupt_configure_dt(&gpout_pin, GPIO_INT_LEVEL_ACTIVE);
        if (ret != 0) {
                LOG_ERR("Failed to setup interrupt on GPOUT: ERROR -%i.\n", ret);
                return ret;
        }

        return 0;
}

bool BQ27220::readReg(uint8_t reg, uint8_t * buffer, uint16_t len) {
        int ret;
        uint64_t now = micros();
        int delay = MIN(BQ27220_I2C_TIMEOUT_US - (int)(now - last_i2c), BQ27220_I2C_TIMEOUT_US);

        if (delay > 0) k_usleep(delay);

        _i2c->aquire();

        ret = i2c_burst_read(_i2c->master, address, reg, buffer, len);
        if (ret) LOG_WRN("I2C read failed: %d\n", ret);

        _i2c->release();

        last_i2c = micros();

        return (ret == 0);
}

void BQ27220::writeReg(uint8_t reg, uint8_t *buffer, uint16_t len) {
        int ret;
        uint64_t now = micros();
        int delay = MIN(BQ27220_I2C_TIMEOUT_US - (int)(now - last_i2c), BQ27220_I2C_TIMEOUT_US);

        if (delay > 0) k_usleep(delay);

        _i2c->aquire();

        ret = i2c_burst_write(_i2c->master, address, reg, buffer, len);
        if (ret) LOG_WRN("I2C write failed: %d", ret);

        _i2c->release();

        last_i2c = micros();
}

bat_status BQ27220::battery_status() {
        bat_status status;
        uint16_t val = 0;
        bool ret = readReg(registers::FLAGS, (uint8_t *) &val, sizeof(val));

        status.DSG = val & 0x1;
        status.SYSDWN = val & (1 << 1);
        status.TDA = val & (1 << 2);
        status.BATTPRES = val & (1 << 3);
        status.AUTH_GD = val & (1 << 4);
        status.OCVGD = val & (1 << 5);
        status.TCA = val & (1 << 6);
        status.CHGINH = val & (1 << 8);
        status.FC = val & (1 << 9);
        status.OTD = val & (1 << 10);
        status.OTC = val & (1 << 11);
        status.SLEEP = val & (1 << 12);
        status.OCVFAIL = val & (1 << 13);
        status.OCVCOMP = val & (1 << 14);
        status.FD = val & (1 << 15);

        return status;
}

gauge_status BQ27220::gauging_state() {
        gauge_status status;

        uint16_t state;

        uint16_t command = GAUGING_STATUS;
        writeReg(0x3E, (uint8_t *) &command, sizeof(command));
        readReg(0x40, (uint8_t *) &state, sizeof(state));

        status.edv2 = state & (1 << 6);
        status.edv1 = state & (1 << 5);
        status.edv0 = state & (1 << 13);

        return status;
}

float BQ27220::temperature() {
        uint16_t temp_K = 0;
        bool ret = readReg(registers::TEMP, (uint8_t *) &temp_K, sizeof(temp_K));

        float temp = temp_K / 10.0 - 273.15;
        return temp;
}

float BQ27220::voltage() {
        uint16_t mV = 0;
        bool ret = readReg(registers::VOLT, (uint8_t *) &mV, sizeof(mV));

        float v = mV / 1000.0;
        return v;
}

float BQ27220::capacity() {
        uint16_t mAh = 0;
        bool ret = readReg(registers::FCC, (uint8_t *) &mAh, sizeof(mAh));
        return mAh;
}

float BQ27220::time_to_full() {
        uint16_t minutes = 0;
        bool ret = readReg(registers::TTF, (uint8_t *) &minutes, sizeof(minutes));
        return minutes;
}


float BQ27220::time_to_empty() {
        uint16_t minutes = 0;
        bool ret = readReg(registers::TTE, (uint8_t *) &minutes, sizeof(minutes));
        return minutes;
}

float BQ27220::state_of_charge() {
        uint16_t soc = 0;
        bool ret = readReg(registers::SOC, (uint8_t *) &soc, sizeof(soc));
        return soc;
}

float BQ27220::state_of_health() {
        uint16_t soc = 0;
        bool ret = readReg(registers::SOH, (uint8_t *) &soc, sizeof(soc));
        return soc;
}

float BQ27220::current() {
        int16_t mA = 0;
        bool ret = readReg(registers::NAC, (uint8_t *) &mA, sizeof(mA));
        return mA;
}

float BQ27220::average_current() {
        int16_t mA = 0;
        bool ret = readReg(registers::AI, (uint8_t *) &mA, sizeof(mA));
        return mA;
}

float BQ27220::design_cap() {
        uint16_t mAh = 0;
        bool ret = readReg(registers::DCAP, (uint8_t *) &mAh, sizeof(mAh));
        return mAh;
}

float BQ27220::remaining_cap() {
        uint16_t mAh = 0;
        bool ret = readReg(registers::RM, (uint8_t *) &mAh, sizeof(mAh));
        return mAh;
}

float BQ27220::charge_current() {
        int16_t mA = 0;
        bool ret = readReg(registers::CC, (uint8_t *) &mA, sizeof(mA));
        return mA;
}

int BQ27220::cycle_count() {
        uint16_t n_cycles = 0;
        bool ret = readReg(registers::CYCT, (uint8_t *) &n_cycles, sizeof(n_cycles));
        return n_cycles;
}

float  BQ27220::standby_current() {
        int16_t mA = 0;
        bool ret = readReg(registers::SI, (uint8_t *) &mA, sizeof(mA));

        return mA;
}

op_state BQ27220::operation_state() {
        op_state state;
        uint16_t status = 0;
        bool ret = readReg(registers::OP_STAT, (uint8_t *) &status, sizeof(status));
        
        state.CALD = status & 0x01;
        state.SEC = (status >> 1) & 0x3;
        state.EDV2 = status & 0x08;
        state.VDQ = status & 0x10;
        state.INITCOMP = status & 0x20;
        state.SMTH = status & 0x40;
        state.BTPINT = status & 0x80;
        state.CFG_UPDATE = status & 0x400;

        return state;
}

void BQ27220::write_command(BQ27220::commands cmd) {
    writeReg(registers::CTRL, (uint8_t *) &cmd, sizeof(cmd));
}

void BQ27220::write_command(uint16_t cmd) {
    writeReg(registers::CTRL, (uint8_t *) &cmd, sizeof(cmd));
}

void BQ27220::full_access() {
        write_command(0xffff);
        k_msleep(100);
        write_command(0xffff);
        k_msleep(100);
}

/*void BQ27220::sleep_mode() {

}

void BQ27220::active_mode() {

}*/

void BQ27220::enter_config_update() {
    //write_command(0x14); //0x13);
    op_state state = operation_state();
    if (state.CFG_UPDATE) {
        LOG_WRN("Already in CONFIG UPDATE MODE.");
        return;
    }
    write_command(CONFIG_UPDATE_ENTER);
    k_msleep(1100);
    do {
        state = operation_state();
        k_msleep(100);
    } 
    while (!state.CFG_UPDATE);
    LOG_INF("CONFIG UPDATE MODE entered.");

    //if (state.CFG_UPDATE) printk("CONFIG UPDATE MODE entered.\n");
    //else printk("Failed to enter CONFIG UPDATE MODE.\n");
}

void BQ27220::exit_config_update(bool init) {
    op_state state = operation_state();
    if (!state.CFG_UPDATE) {
        LOG_WRN("Device is not in CONFIG UPDATE MODE.");
        return;
    }
    if (init) write_command(CONFIG_UPDATE_EXIT);
    else write_command(CONFIG_UPDATE_EXIT_NO_INIT);
    k_msleep(1100);
    do {
        state = operation_state();
        k_msleep(100);
    } while (state.CFG_UPDATE);
    LOG_INF("CONFIG UPDATE MODE exited.");
    //if (!state.CFG_UPDATE) printk("CONFIG UPDATE MODE exited.\n");
    //else printk("Failed to exit CONFIG UPDATE MODE.\n");
}

void BQ27220::read_RAM(uint16_t ram_address, uint8_t * data, int len) {
        bool ret;

        writeReg(0x3E, (uint8_t *) &ram_address, sizeof(ram_address));
        k_usleep(BQ27220_RAM_TIMEOUT_US);
        ret = readReg(0x40, data, len);
}

int BQ27220::write_RAM(uint16_t ram_address, uint8_t * data, int len, bool check) {
        uint8_t check_sum=0;
        uint8_t data_len=0;
        uint8_t buf[len];

        bool ret;

        writeReg(0x3E, (uint8_t *) &ram_address, sizeof(ram_address));

        k_usleep(BQ27220_RAM_TIMEOUT_US);

        ret = readReg(0x61, (uint8_t *) &data_len, sizeof(data_len));
        ret = readReg(0x40, buf, len);
        ret = readReg(0x60, (uint8_t *) &check_sum, sizeof(check_sum));

        uint8_t my_check = (uint8_t)0xFF-check_sum; // - data[0] - data[1];

        for (int i = 0; i < len; i++) {
                my_check -= buf[i];
                my_check += data[i];
        }

        my_check = (uint8_t)0xFF - my_check;

        writeReg(0x40, (uint8_t *) data, len);
        writeReg(0x60, (uint8_t *) &my_check, sizeof(my_check));
        writeReg(0x61, (uint8_t *) &data_len, sizeof(data_len));

        k_usleep(BQ27220_RAM_TIMEOUT_US);
        
        if (check) {
                uint8_t * read_buff = (uint8_t *) k_malloc(data_len);

                read_RAM(ram_address, read_buff, data_len);

                for (int i = 0; i < data_len; i++) {
                        if (read_buff[i] != data[i]) {
                                k_free(read_buff);
                                return -1;
                        }
                }

                k_free(read_buff);
        }

        return 0;
}

int BQ27220::write_RAM(uint16_t ram_address, uint16_t val, bool check) {
        uint8_t data[sizeof(val)];

        data[0] = val >> 8;
        data[1] = val & 0xFF;

        return write_RAM(ram_address, data, sizeof(val), check);
}

void BQ27220::setup(const battery_settings &_battery_settings, bool init) {
        int ret;

        // unseal
        write_command(0x0414);
        k_msleep(100);
        write_command(0x3672);
        k_msleep(100);
        
        // full access
        full_access();

        enter_config_update();
        //k_usleep(1000);

        //hibernate off (not supported by fuel gauge)
        //ret = write_RAM(0x9220, 0);

        // design and full charge capacity
        ret = write_RAM(0x929F, _battery_settings.capacity);
        ret = write_RAM(0x929D, _battery_settings.capacity); //130
        // near full
        ret = write_RAM(0x926B, 5);

        ret = write_RAM(0x91F5, _battery_settings.temp_min * 10);
        ret = write_RAM(0x91F7, _battery_settings.temp_max * 10);

        // charge current
        ret = write_RAM(0x91FB, _battery_settings.i_charge);

        // charge voltage
        ret = write_RAM(0x91FD, _battery_settings.u_term * 1000);

        // taper current
        ret = write_RAM(0x9201, _battery_settings.i_term);

        // experimental: min taper capacity
        ret = write_RAM(0x9203, 4); // standard: 25

        // deadband
        uint8_t val = 1;
        ret = write_RAM(0x91DE, &val, sizeof(uint8_t));
        
        // deadband CC (verursacht Probleme, rm zählt zu schnell?)
        /*val = 5;
        ret = write_RAM(0x91DF, &val, sizeof(uint8_t));
        */
        
        // sleep current
        ret = write_RAM(0x9217, 1);

        // dischage current trd
        ret = write_RAM(0x9228, 2);
        // charge current trd
        ret = write_RAM(0x922A, 2);
        // quit current
        ret = write_RAM(0x922C, 1);

        //dod  0%: 4287
        //dod 10%: 4125
        //dod 20%: 3998
        //dod 30%: 3878
        //dod 40%: 3772
        //dod 50%: 3689
        //dod 60%: 3630
        //dod 70%: 3588
        //dod 80%: 3533
        //dod 85%: 3495
        //dod 90%: 3451, 3456
        //dod 92.5%: 3435
        //dod 95%: 3411, 3416 edv1: 6%
        //dod 97.5%: 3350, edv0: 1.5%
        //dod: 100%: 3250, 3269
        //dod: 101.25%: 3188
        //dod: 102.5%: 3123
        //dod: 103.25%: 3089

        // sysDown set Voltage
        ret = write_RAM(0x9240, _battery_settings.u_vlo * 1000 + CONFIG_BATTERY_SYSDOWN_SET_OFFSET);

        // sysDown clear Voltage
        ret = write_RAM(0x9243, _battery_settings.u_vlo * 1000 + CONFIG_BATTERY_SYSDOWN_SET_OFFSET + CONFIG_BATTERY_SYSDOWN_HYSTERESIS);

        // FD set
        ret = write_RAM(0x9282, _battery_settings.u_vlo * 1000 + CONFIG_BATTERY_FD_SET_OFFSET);

        // FD clear
        ret = write_RAM(0x9284, _battery_settings.u_vlo * 1000 + CONFIG_BATTERY_FD_SET_OFFSET + CONFIG_BATTERY_FD_HYSTERESIS); 

        // FC Voltage
        ret = write_RAM(0x9288, _battery_settings.u_term * 1000 - CONFIG_BATTERY_FC_VOLTAGE_OFFSET);

        // Electonic Load in 3µA steps
        ret = write_RAM(0x9269, 6); // 18 µA

        // EMF
        //write_RAM(0x92A7, 36001);
        //C0
        ret = write_RAM(0x92A9, 480); //bat1:250
        //R0
        ret = write_RAM(0x92AB, 19941); //bat1: 19941 //22542 //new bat:  17340
        //R1
        //write_RAM(0x92AF, 3160);

        // Configuration gauging FIXED_EDV
        //ret = write_RAM(0x929B, 0x1022); //0x102A

        //ret = write_RAM(0x927F, 0x0C8C);

        // do not use, only on CT makes sense:
        // SOC Flag, enable FC voltage detection
        uint8_t flags_b = 0x8C;
        ret = write_RAM(0x9281, &flags_b, sizeof(flags_b));

        // Overload current
        ret = write_RAM(0x9264, _battery_settings.i_max);

        // CEDV Smoothing Config
        uint8_t cedv_conf = 0x0D; //Default: 0x08, Enable SMEXT, SMEN 0x0D
        ret = write_RAM(0x9271, &cedv_conf, sizeof(cedv_conf));

        ret = write_RAM(0x9272, 3700);

        exit_config_update(init);

        // put fuel gauge to sealed state
        write_command(SEAL);
}

int BQ27220::set_int_callback(gpio_callback_handler_t handler) {
    gpio_init_callback(&int_cb_data, handler, int_cb_data.pin_mask | BIT(gpout_pin.pin));
    return gpio_add_callback(gpout_pin.port, &int_cb_data);
}