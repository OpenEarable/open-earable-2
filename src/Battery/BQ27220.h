#ifndef _BQ27220_H
#define _BQ27220_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <math.h>
#include <Wire.h>

#define BQ27220_I2C_TIMEOUT_US 66
#define BQ27220_RAM_TIMEOUT_US 1000

/*enum bq27xxx_bat_status {
        DSG = 0, SYSDWN, TDA, BATTPRES, AUTH_GD, OCVGD, TCA, RSVD,
        CHGINH, FC, OTD, OTC, SLEEP, OCVFAIL, OCVCOMP, FD
};*/

struct bat_status {
        bool DSG = false;
        bool SYSDWN = false;
        bool TDA = false;
        bool BATTPRES = false;
        bool AUTH_GD = false;
        bool OCVGD = false;
        bool TCA = false;
        bool CHGINH = false;
        bool FC = false;
        bool OTD = false;
        bool OTC = false;
        bool SLEEP = false;
        bool OCVFAIL = false;
        bool OCVCOMP = false;
        bool FD = false;
};

struct op_state {
        bool CFG_UPDATE = false;
        bool BTPINT = false;
        bool SMTH = false;
        bool INITCOMP = false;
        bool VDQ = false;
        bool EDV2 = false;
        int SEC = 0;
        bool CALD = false;
};

struct battery_config {
        uint16_t capacity_mAh = 3000;
        uint16_t nominal_voltage_mV = 3700;
        uint16_t max_voltage_mV = 4200;
        uint16_t charge_current_mA = 200;
        uint16_t taper_current_mA = 100;
};

struct gauge_status {
        bool edv2 = false;
        bool edv1 = false;
        bool edv0 = false;
};

class BQ27220 {
public:
    BQ27220(TwoWire * wire);
    enum commands : uint16_t {
        CALIBRATION_SW = 0x002D,
        GAUGING_STATUS = 0x0056,
        CMD_CALIBRATION_EXIT = 0x0080,
        CALIBRATION_ENTER = 0x0081,
        CONFIG_UPDATE_ENTER = 0x0090,
        CONFIG_UPDATE_EXIT = 0x0091,
        CONFIG_UPDATE_EXIT_NO_INIT = 0x0092,
        RESET = 0x0041,
        SEALED = 0x0030,
    };

    enum registers : uint8_t {
        CTRL = 0x00,
        TEMP = 0x06,
        INT_TEMP = 0x28,
        VOLT = 0x08,
        AI = 0x14,
        FLAGS = 0x0A,
        TTE = 0x16,
        TTF = 0x18,
        TTES = 0x1c,
        TTECP= 0x26,
        NAC = 0x0C,
        FCC = 0x12,
        CYCT = 0x2A,
        AE = 0x22,
        SOC = 0x2C,
        DCAP = 0x3C,
        AP = 0x24,
        CC = 0x32,
        SI = 0x1A,
        RM = 0x10,
        DATA = 0x40,
        CHECK_SUM = 0x60,
        DATA_LEN = 0x61,
        OP_STAT = 0x3A,
    };

    int begin();

    bat_status battery_status();
    float temperature();
    float voltage();
    float capacity();
    float time_to_full();
    float time_to_empty();
    float state_of_charge();
    float current();
    float average_current();
    float design_cap();
    float remaining_cap();
    float charge_current();
    int cycle_count();
    float standby_current();
    op_state operation_state();
    gauge_status gauging_state();
    void write_command(commands command);
    void enter_config_update();
    void exit_config_update(bool init = true);

    void full_access();
    void setup(bool init = true);

    int set_wakeup_int();

    //void sleep_mode();
    //void active_mode();

    int set_int_callback(gpio_callback_handler_t handler);
private:
    bool readReg(uint8_t reg, uint8_t * buffer, uint16_t len);
    void writeReg(uint8_t reg, uint8_t * buffer, uint16_t len);

    void read_RAM(uint16_t ram_address, uint8_t * data, int len);
    void write_RAM(uint16_t ram_address, uint8_t * data, int len);
    void write_RAM(uint16_t ram_address, uint16_t val);

    void write_command(uint16_t cmd);

    int address = 0x55;

    uint64_t last_i2c;

    gpio_callback int_cb_data;

    const gpio_dt_spec gpout_pin = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
        .pin = 10, //8
        .dt_flags = GPIO_ACTIVE_LOW
    };

    TwoWire *_pWire;
};

extern BQ27220 fuel_gauge;

//extern BQ27220 battery_gauge;

#endif