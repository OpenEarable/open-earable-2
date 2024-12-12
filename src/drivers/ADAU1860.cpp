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

        k_msleep(35); // see datasheet

        _pWire->begin();

        // Power saving
        uint8_t cm_startup_over = 1 << 4;
        writeReg(registers::CHIP_PWR, &cm_startup_over, sizeof(cm_startup_over));

        // set up CLK_CTRL and PLL
        uint8_t pll_ctrl = 0x1; // XTAL_EN = 0, PLL_EN = 1 ?
        writeReg(registers::PLL_PGA_PWR, &pll_ctrl, sizeof(pll_ctrl));

        //setup clock
        uint8_t clk_ctrl1 = 0x3 << 6 | 1 << 4 | 1 << 3;
        writeReg(registers::CLK_CTRL1, &clk_ctrl1, sizeof(clk_ctrl1));

        // verify power up complete
        uint8_t status2;
        readReg(registers::STATUS2, &status2, sizeof(status2));

        // power up complete and PLL lock (0 or 1)?
        if ((status2 & (1 << 7)) && (status2 & 1)) return 0;
        else return -1;
}

int ADAU1860::end() {
        int ret;

        // pull-down PD pin

        ret = pm_device_runtime_put(DEVICE_DT_GET(DT_NODELABEL(load_switch)));

        return ret;
}

int ADAU1860::setup() {
        uint8_t pwr_ctrl = 0x0;
        uint8_t dac_ctrl2 = 0x10;
        uint8_t spt_ctrl1 = 0x10; // 16 BCKL per slot, stereo
        uint8_t spt_ctrl2 = 0x00; // left channel
        uint8_t dac_vol = 0x40; // left channel
        uint8_t clock_ctrl = 0x00; //0x00 for auto

        // reset all registers
        //soft_reset(true);

        // dac_ctrl1 is set DAC_FS = 48kHz

        /*writeReg(registers::PWR_CTRL, &pwr_ctrl, sizeof(pwr_ctrl));
        writeReg(registers::CLK_CTRL, &clock_ctrl, sizeof(clock_ctrl));
        writeReg(registers::SPT_CTRL1, &spt_ctrl1, sizeof(spt_ctrl1));
        writeReg(registers::DAC_CTRL2, &dac_ctrl2, sizeof(dac_ctrl2)); // unmute

        uint8_t status = 0;
        readReg(registers::DAC_CTRL2, &status, sizeof(status));*/

        //LOG_INF("DAC_CTRL: 0x%02X", status);

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
        if (full_reset) status <<= 4;
        writeReg(registers::RESET, &status, sizeof(status));
        return ret;
}


bool ADAU1860::readReg(uint32_t reg, uint8_t * buffer, uint16_t len) {
        uint64_t now = micros();
        int delay = MIN(SSM6515_I2C_TIMEOUT_US - (int)(now - last_i2c), SSM6515_I2C_TIMEOUT_US);

        if (delay > 0) k_usleep(delay);

        _pWire->aquire();

        _pWire->beginTransmission(address);
        _pWire->write(reg);
        if (_pWire->endTransmission(false) != 0) {
                _pWire->release();
                return false;
        }
        _pWire->requestFrom(address, len);

        for (uint16_t i = 0; i < len; i++) {
            buffer[i] = _pWire->read();
        }

        int ret = _pWire->endTransmission();

        _pWire->release();

        last_i2c = micros();
        return (ret == 0);
}

void ADAU1860::writeReg(uint32_t reg, uint8_t *buffer, uint16_t len) {
        uint64_t now = micros();
        int delay = MIN(SSM6515_I2C_TIMEOUT_US - (int)(now - last_i2c), SSM6515_I2C_TIMEOUT_US);

        if (delay > 0) k_usleep(delay);

        _pWire->aquire();

        _pWire->beginTransmission(address);
        _pWire->write(reg);
        for(uint16_t i = 0; i < len; i ++)
            _pWire->write(buffer[i]);
        _pWire->endTransmission();

        _pWire->release();

        last_i2c = micros();
}