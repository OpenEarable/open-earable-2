#include "KTD2026.h"

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include "openearable_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LED, CONFIG_MAIN_LOG_LEVEL);

bool KTD2026::readReg(uint8_t reg, uint8_t * buffer, uint16_t len) {
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

        return (ret == 0);
}

void KTD2026::writeReg(uint8_t reg, uint8_t *buffer, uint16_t len) {
        _pWire->aquire();
        _pWire->beginTransmission(address);
        _pWire->write(reg);
        for(uint16_t i = 0; i < len; i ++)
            _pWire->write(buffer[i]);
        _pWire->endTransmission();
        _pWire->release();
}

void KTD2026::begin() {
        int ret;
        
        ret = pm_device_runtime_get(ls_1_8);
        ret = pm_device_runtime_get(ls_3_3);

        _pWire->begin();

        reset();
}

void KTD2026::reset() {
        uint8_t val = 0x7;
        writeReg(registers::CTRL, &val, sizeof(val));
        k_usleep(200);
}

void KTD2026::power_off() {
        uint8_t val = 0x8;
        writeReg(registers::CTRL, &val, sizeof(val));
        int ret = pm_device_runtime_put(ls_1_8);
        ret = pm_device_runtime_put(ls_3_3);
}

void KTD2026::setColor(RGBColor color) {
        uint8_t channel_enable = 0;
        for (int i = 0; i < 3; i++) {
                if (color[i] > 0) {
                        channel_enable |= 1 << (2 * i);
                        color[i]--;
                }
        }

        writeReg(registers::I_R, color, sizeof(RGBColor));
        writeReg(registers::EN_CH, &channel_enable, sizeof(channel_enable));
}

void KTD2026::blink(RGBColor color, const int time_on_millis, const int period_millis) {
        pulse(color, time_on_millis, 0, 0, period_millis);
}

void KTD2026::pulse(RGBColor color, const int time_on_millis, const int time_rise_millis, const int time_fall_millis, const int period_millis) {
        uint8_t channel_enable = 0;
        RGBColor _color = {0,0,0};

        uint8_t flash_period = (period_millis >> 7) & 0x7F; // =/ 128
        uint8_t time_on = 250 * time_on_millis / period_millis; // to 0.4% steps
        uint8_t t_rise_fall = (((time_fall_millis >> 7) & 0x0F) << 4) | ((time_rise_millis >> 7) & 0x0F); // =/ 128

        for (int i = 0; i < 3; i++) {
                if (color[i] > 0) {
                        channel_enable |= 2 << (2 * i);
                        _color[i] = color[i] - 1;
                }
        }

        writeReg(registers::I_R, color, sizeof(RGBColor));
        writeReg(registers::EN_CH, &channel_enable, sizeof(channel_enable));

        writeReg(registers::FP, &flash_period, sizeof(flash_period));
        writeReg(registers::RAMP, &t_rise_fall, sizeof(t_rise_fall));
        writeReg(registers::PWM1, &time_on, sizeof(time_on));
}

/* Not working */
void KTD2026::getColor(RGBColor * color) {
        uint8_t channel_enable = 0;

        readReg(registers::EN_CH, &channel_enable, sizeof(channel_enable));
        readReg(registers::I_R, *color, sizeof(RGBColor));

        for (int i = 0; i < 3; i++) {
                (*color)[i] ++;
                if ((channel_enable & BIT(2 * i)) == 0) {
                        (*color)[i] = 0;
                }
        }
}

KTD2026 led_controller;