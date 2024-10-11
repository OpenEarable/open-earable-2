#include "SSM6515.h"
#include "LoadSwitchPM.h"
#include "nrf5340_audio_common.h"
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(SSM6515, CONFIG_MAIN_LOG_LEVEL);

SSM6515 dac(&Wire1);

SSM6515::SSM6515(TwoWire * wire) : _pWire(wire) {

}

int SSM6515::begin() {
        int ret;

        ret = pm_device_runtime_get(ls_1_8);

        _pWire->begin();

        _pWire->aquire();

        _pWire->beginTransmission(address);
        if (_pWire->endTransmission() == 0){
                _pWire->release();
                last_i2c = k_cyc_to_us_floor64(k_cycle_get_32());
                return 0;
        }
        _pWire->release();
        return -1;
}

int SSM6515::end() {
        int ret;

        ret = pm_device_runtime_put(DEVICE_DT_GET(DT_NODELABEL(load_switch)));

        return ret;
}

int SSM6515::setup() {
        uint8_t pwr_ctrl = 0x0;
        uint8_t dac_ctrl2 = 0x10;
        uint8_t spt_ctrl1 = 0x10; // 16 BCKL per slot, stereo
        uint8_t spt_ctrl2 = 0x00; // left channel
        uint8_t dac_vol = 0x40; // left channel
        uint8_t clock_ctrl = 0x00; //0x00 for auto

        // reset all registers
        soft_reset(true);

        writeReg(registers::PWR_CTRL, &pwr_ctrl, sizeof(pwr_ctrl));
        writeReg(registers::CLK_CTRL, &clock_ctrl, sizeof(clock_ctrl));
        writeReg(registers::SPT_CTRL1, &spt_ctrl1, sizeof(spt_ctrl1));
        writeReg(registers::DAC_CTRL2, &dac_ctrl2, sizeof(dac_ctrl2)); // unmute

        uint8_t status = 0;
        readReg(registers::DAC_CTRL2, &status, sizeof(status));

        //LOG_INF("DAC_CTRL: 0x%02X", status);

        return 0;
}

int SSM6515::mute(bool active) {
        uint8_t status = 0;
        readReg(registers::DAC_CTRL2, &status, sizeof(status));

        // clear last bit
        status &= ~0x1;
        if (active) status |= 0x1;

        writeReg(registers::DAC_CTRL2, &status, sizeof(status));

        return 0;
}

int SSM6515::set_volume(uint8_t volume) {
        uint8_t dac_vol = 0xFF-volume;
        writeReg(registers::DAC_VOL, &dac_vol, sizeof(dac_vol)); // unmute

        return 0;
}

uint8_t SSM6515::get_volume() {
        uint8_t dac_vol = 0;
        readReg(registers::DAC_VOL, &dac_vol, sizeof(dac_vol));
        return 0xFF-dac_vol;
}

int SSM6515::soft_reset(bool full_reset) {
        int ret = 0;

        uint8_t status = 0x1;
        if (full_reset) status <<= 4;
        writeReg(registers::RESET, &status, sizeof(status));
        return ret;
}


bool SSM6515::readReg(uint8_t reg, uint8_t * buffer, uint16_t len) {
        uint64_t now = k_cyc_to_us_floor64(k_cycle_get_32());
        int delay = MIN(SSM6515_I2C_TIMEOUT_US - (int)(now - last_i2c), SSM6515_I2C_TIMEOUT_US);

        if (delay > 0) k_usleep(delay);

        _pWire->aquire();

        _pWire->beginTransmission(address);
        _pWire->write(reg);
        if (_pWire->endTransmission() != 0) {
                _pWire->release();
                return false;
        }
        _pWire->requestFrom(address, len);

        for (uint16_t i = 0; i < len; i++) {
            buffer[i] = _pWire->read();
        }

        int ret = _pWire->endTransmission();

        _pWire->release();

        last_i2c = k_cyc_to_us_floor64(k_cycle_get_32());
        return (ret == 0);
}

void SSM6515::writeReg(uint8_t reg, uint8_t *buffer, uint16_t len) {
        uint64_t now = k_cyc_to_us_floor64(k_cycle_get_32());
        int delay = MIN(SSM6515_I2C_TIMEOUT_US - (int)(now - last_i2c), SSM6515_I2C_TIMEOUT_US);

        if (delay > 0) k_usleep(delay);

        _pWire->aquire();

        _pWire->beginTransmission(address);
        _pWire->write(reg);
        for(uint16_t i = 0; i < len; i ++)
            _pWire->write(buffer[i]);
        _pWire->endTransmission();

        _pWire->release();

        last_i2c = k_cyc_to_us_floor64(k_cycle_get_32());
}