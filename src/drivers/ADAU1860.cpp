#include "ADAU1860.h"
#include "LoadSwitch.h"
#include "nrf5340_audio_common.h"
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ADAU1860, CONFIG_MAIN_LOG_LEVEL);

ADAU1860 dac(&Wire1);

ADAU1860::ADAU1860(TwoWire * wire) : _pWire(wire) {

}

int ADAU1860::begin() {
        int ret;

        ret = pm_device_runtime_get(ls_1_8);

        // pull-up PD pin

        ret = gpio_pin_configure_dt(&dac_enable_pin, GPIO_OUTPUT_ACTIVE);
	if (ret != 0) {
                LOG_ERR("Failed to set DAC enable as output.\n");
                return ret;
        }

        k_msleep(35); // see datasheet

        _pWire->begin();

        last_i2c = micros();

        // Power saving
        /*uint8_t cm_startup_over = 1 << 2; // << 4 | 1 <<;
        writeReg(registers::CHIP_PWR, &cm_startup_over, sizeof(cm_startup_over));

        k_msleep(35); // see datasheet*/

        uint8_t startup_dlycnt_byp = 1;
        writeReg(registers::PMU_CTRL2, &startup_dlycnt_byp, sizeof(startup_dlycnt_byp));

        // Power saving
        uint8_t cm_startup_over = 1 << 4;
        writeReg(registers::CHIP_PWR, &cm_startup_over, sizeof(cm_startup_over));

        uint8_t test;
        readReg(registers::CHIP_PWR, &test, sizeof(test));

        LOG_INF("CHIP_PWR: %x", test);

        // DSP_PWR - reset val

        // set up CLK_CTRL and PLL
        uint8_t pll_ctrl = 0x1; // XTAL_EN = 0, PLL_EN = 1 ?
        writeReg(registers::PLL_PGA_PWR, &pll_ctrl, sizeof(pll_ctrl));

        //setup clock (PLL integer, MCLK source)
        uint8_t clk_ctrl1 = (0x3 << 6);
        // | (1 << 4);// | (1 << 3);
        writeReg(registers::CLK_CTRL1, &clk_ctrl1, sizeof(clk_ctrl1));

        //DAC PLL multiple of 24.576MHz (=2x12.288)
        //CLK_CTRL2 Prescaler = 1
        //CLK_CTRL3 CLK_CTRL4 R = 4

        // PLL update CLK_CTRL9 ?
        uint8_t clk_ctrl9 = 1;
        writeReg(registers::CLK_CTRL9, &clk_ctrl9, sizeof(clk_ctrl9));

        k_msleep(100);

        // verify power up complete
        uint8_t status2;
        readReg(registers::STATUS2, &status2, sizeof(status2));

        LOG_INF("STATUS2: %x", status2);

        // power up complete and PLL lock (0 or 1)?
        if ((status2 & (1 << 7)) && (status2 & 1)) return 0;
        else return -1;
}

int ADAU1860::end() {
        int ret;

        // pull-down PD pin
        gpio_pin_set_dt(&dac_enable_pin, 0);

        ret = pm_device_runtime_put(ls_1_8);

        return ret;
}

int ADAU1860::setup() {
        // reset all registers
        // soft_reset(true);

        //LOG_INF("DAC_CTRL: 0x%02X", status);

        // DAC_ROUTE0 - EQ 0 / serial port 0 channel 0
        uint8_t dac_route_eq = 75;
        uint8_t dac_route_i2s = 0;
        writeReg(registers::DAC_ROUTE0, &dac_route_i2s, sizeof(dac_route_i2s));

        // HP_CTRL - reset val
        // HP_LVMODE_CTRL1

        // SPT0_CTRL1 - reset val (32 BCLKs?)
        uint8_t spt0_ctrl1 = 0x10; // 1 << 4 (16 BCLKs)
        writeReg(registers::SPT0_CTRL1, &spt0_ctrl1, sizeof(spt0_ctrl1));
        // SPT0_CTRL2 - reset val

        // SPT0_ROUTE0 - i2s output route
        /*uint8_t spt0_route0 = 39; // DMIC Channel 0
        writeReg(registers::SPT0_ROUTE0, &spt0_route0, sizeof(spt0_route0));*/

        // EQ_CFG - engine running
        // EQ_ROUTE - serial port 0 channel 0

        // set Eq params

        // stop eq and clear
        /*uint8_t eq_cfg = 0x10; // eq clear
        writeReg(registers::EQ_CFG, &eq_cfg, sizeof(eq_cfg));

        // clear done check
        uint8_t eq_status = 0;
        while (eq_status != 0x1) {
                readReg(registers::EQ_STATUS, &eq_status, sizeof(eq_status));
                k_usleep(100);
        }

        // write parameters

        // activate eq
        eq_cfg = 0x01;
        writeReg(registers::EQ_CFG, &eq_cfg, sizeof(eq_cfg));*/

        // PDM_VOL0

        // ASRCI_CTRL

        //Headphone power on
        uint8_t headphone_power = 1 << 4; // 1 << 4 (16 BCLKs)
        writeReg(registers::ADC_DAC_HP_PWR, &headphone_power, sizeof(headphone_power));

        return 0;
}

int ADAU1860::mute(bool active) {
        uint8_t status = 0;
        readReg(registers::DAC_CTRL2, &status, sizeof(status));

        // clear last bit
        status &= ~(1 << 6); // 1 << 7 force mute
        if (active) status |= (1 << 6);

        writeReg(registers::DAC_CTRL2, &status, sizeof(status));

        return 0;
}

int ADAU1860::set_volume(uint8_t volume) {
        uint8_t dac_vol = 0xFF-volume;
        writeReg(registers::DAC_VOL0, &dac_vol, sizeof(dac_vol)); // unmute

        return 0;
}

uint8_t ADAU1860::get_volume() {
        uint8_t dac_vol = 0;
        readReg(registers::DAC_VOL0, &dac_vol, sizeof(dac_vol));
        return 0xFF-dac_vol;
}

int ADAU1860::soft_reset(bool full_reset) {
        int ret = 0;

        uint8_t status = 0x1;
        if (!full_reset) status <<= 4;
        writeReg(registers::RESETS, &status, sizeof(status));
        return ret;
}


bool ADAU1860::readReg(uint32_t reg, uint8_t * buffer, uint16_t len) {
        int ret;
        uint64_t now = micros();
        int delay = MIN(ADAU1860_I2C_TIMEOUT_US - (int)(now - last_i2c), ADAU1860_I2C_TIMEOUT_US);

        if (delay > 0) k_usleep(delay);

        _pWire->aquire();

        _pWire->beginTransmission(address);
        _pWire->write(reg);
        if (_pWire->endTransmission(false) != 0) {
                if (ret != 0) LOG_WRN("I2C Error block read: End transmission");
                _pWire->release();
                return false;
        }
        _pWire->requestFrom(address, len);

        for (uint16_t i = 0; i < len; i++) {
            buffer[i] = _pWire->read();
        }

        ret = _pWire->endTransmission();

        _pWire->release();

        last_i2c = micros();
        return (ret == 0);
}

void ADAU1860::writeReg(uint32_t reg, uint8_t *buffer, uint16_t len) {
        uint64_t now = micros();
        int delay = MIN(ADAU1860_I2C_TIMEOUT_US - (int)(now - last_i2c), ADAU1860_I2C_TIMEOUT_US);

        if (delay > 0) k_usleep(delay);

        _pWire->aquire();

        _pWire->beginTransmission(address);
        _pWire->write(reg);
        for(uint16_t i = 0; i < len; i ++)
            _pWire->write(buffer[i]);
        int ret = _pWire->endTransmission();

        if (ret != 0) LOG_WRN("I2C Error block write: End transmission");

        _pWire->release();

        last_i2c = micros();
}