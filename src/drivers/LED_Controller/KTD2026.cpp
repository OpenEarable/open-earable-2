#include "KTD2026.h"

#include <string.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include "openearable_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LED, CONFIG_MAIN_LOG_LEVEL);

namespace {

constexpr uint8_t KTD2026_CTRL_RESET_REGISTERS = 0x05;
constexpr uint8_t KTD2026_CTRL_RESET_COMPLETE_CHIP = 0x07;
constexpr uint8_t KTD2026_CTRL_SHUTDOWN = 0x08;
constexpr uint8_t KTD2026_REGISTER_COUNT = KTD2026::I_B + 1;

constexpr uint8_t KTD2026_RESET_DEFAULTS[] = {
        0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x4F, 0x4F, 0x4F
};
static_assert(sizeof(KTD2026_RESET_DEFAULTS) == KTD2026_REGISTER_COUNT,
              "KTD2026 reset defaults must cover every cached register");

bool isResetCommand(uint8_t reg, const uint8_t *buffer, uint16_t len) {
        if ((reg != KTD2026::CTRL) || (buffer == NULL) || (len != 1)) {
                return false;
        }

        return (buffer[0] == KTD2026_CTRL_RESET_REGISTERS) ||
               (buffer[0] == KTD2026_CTRL_RESET_COMPLETE_CHIP);
}

bool isValidRegisterRange(uint8_t reg, uint16_t len) {
        return (len > 0) && (reg < KTD2026_REGISTER_COUNT) &&
               (len <= (KTD2026_REGISTER_COUNT - reg));
}

void clearColor(RGBColor color) {
        memset(color, 0, sizeof(RGBColor));
}

}

bool KTD2026::readReg(uint8_t reg, uint8_t * buffer, uint16_t len) {
        if ((buffer == NULL) || !isValidRegisterRange(reg, len)) {
                LOG_WRN("Invalid KTD2026 register read: reg=0x%02x len=%u",
                        (unsigned int)reg, (unsigned int)len);
                return false;
        }

        memcpy(buffer, &_register_cache[reg], len);

        return true;
}

bool KTD2026::writeReg(uint8_t reg, const uint8_t *buffer, uint16_t len, bool ignore_reset_nack) {
        bool reset_command = isResetCommand(reg, buffer, len);

        if ((buffer == NULL) || !isValidRegisterRange(reg, len)) {
                LOG_WRN("Invalid KTD2026 register write: reg=0x%02x len=%u",
                        (unsigned int)reg, (unsigned int)len);
                return false;
        }

        _i2c->aquire();

        int ret = i2c_burst_write(_i2c->master, address, reg, buffer, len);
        if (ret) {
                if (ignore_reset_nack && reset_command) {
                        LOG_DBG("Ignoring expected KTD2026 reset NACK: %d", ret);
                } else {
                        LOG_WRN("I2C write failed: %d", ret);
                }
        }

        _i2c->release();

        if (ret && !(ignore_reset_nack && reset_command)) {
                return false;
        }

        if (reset_command) {
                resetRegisterCache();
        } else {
                memcpy(&_register_cache[reg], buffer, len);
        }

        return true;
}

void KTD2026::begin() {
        int ret;

        if (_active) return;

	_active = true;
        
        ret = pm_device_runtime_get(ls_1_8);
        ret = pm_device_runtime_get(ls_3_3);

        _i2c->begin();

        reset();
}

void KTD2026::reset() {
        uint8_t val = KTD2026_CTRL_RESET_COMPLETE_CHIP;
        if (writeReg(registers::CTRL, &val, sizeof(val), true)) {
                clearColor(current_color);
        }
        k_usleep(200);
}

void KTD2026::power_off() {
        uint8_t val = KTD2026_CTRL_SHUTDOWN;
        if (writeReg(registers::CTRL, &val, sizeof(val))) {
                clearColor(current_color);
        }
        int ret = pm_device_runtime_put(ls_1_8);
        ret = pm_device_runtime_put(ls_3_3);

	_active = false;
}

void KTD2026::setColor(const RGBColor& color) {
        uint8_t channel_enable = 0;
        RGBColor _color;
        memcpy(_color, color, sizeof(RGBColor));  // Korrekte Array-Kopie
        
        for (int i = 0; i < 3; i++) {
                if (_color[i] > 0) {
                        channel_enable |= 1 << (2 * i);
                        _color[i]--;
                }
        }

        bool color_written = writeReg(registers::I_R, _color, sizeof(RGBColor));
        bool channel_written = writeReg(registers::EN_CH, &channel_enable, sizeof(channel_enable));

        if (color_written && channel_written) {
                getColor(&current_color);
        }
}

void KTD2026::blink(const RGBColor& color, const int time_on_millis, const int period_millis) {
        pulse(color, time_on_millis, 0, 0, period_millis);
}

void KTD2026::pulse(const RGBColor& color, const int time_on_millis, const int time_rise_millis, const int time_fall_millis, const int period_millis) {
        uint8_t channel_enable = 0;
        RGBColor _color = {0,0,0};

        uint8_t flash_period = (period_millis >> 7) & 0x7F;
        uint8_t time_on = 250 * time_on_millis / period_millis;
        uint8_t time_on_2 = 0x1; // reset value
        uint8_t t_rise_fall = (((time_fall_millis >> 7) & 0x0F) << 4) | ((time_rise_millis >> 7) & 0x0F);

        for (int i = 0; i < 3; i++) {
                if (color[i] > 0) {
                        channel_enable |= 0x2 << (2 * i);
                        _color[i] = color[i] - 1;
                }
        }

        bool color_written = writeReg(registers::I_R, _color, sizeof(RGBColor));
        bool channel_written = writeReg(registers::EN_CH, &channel_enable, sizeof(channel_enable));

        bool flash_written = writeReg(registers::FP, &flash_period, sizeof(flash_period));
        bool ramp_written = writeReg(registers::RAMP, &t_rise_fall, sizeof(t_rise_fall));
        bool pwm1_written = writeReg(registers::PWM1, &time_on, sizeof(time_on));
        bool pwm2_written = writeReg(registers::PWM2, &time_on_2, sizeof(time_on_2));

        if (color_written && channel_written && flash_written && ramp_written && pwm1_written &&
            pwm2_written) {
                getColor(&current_color);
        }
}

void KTD2026::pulse2(const RGBColor& color, const RGBColor& color2, const int time_on_millis, const int time_rise_millis, const int time_fall_millis, const int period_millis) {
        uint8_t channel_enable = 0;

        RGBColor _color = {0,0,0};

        uint8_t flash_period = (period_millis >> 7) & 0x7F;
        uint8_t time_on = 250 * time_on_millis / period_millis;
        uint8_t t_rise_fall = (((time_fall_millis >> 7) & 0x0F) << 4) | ((time_rise_millis >> 7) & 0x0F);

        for (int i = 0; i < 3; i++) {
                if (color[i] > 0) {
                        channel_enable |= 0x2 << (2 * i);
                        _color[i] = color[i] - 1;
                }
        }

        for (int i = 0; i < 3; i++) {
                if (color2[i] > 0) {
                        channel_enable |= 0x3 << (2 * i);
                        _color[i] = color2[i] - 1;
                }
        }

        bool color_written = writeReg(registers::I_R, _color, sizeof(RGBColor));
        bool channel_written = writeReg(registers::EN_CH, &channel_enable, sizeof(channel_enable));

        uint8_t val = 0x2; // slot 3
        bool ctrl_written = writeReg(registers::CTRL, &val, sizeof(val));

        bool flash_written = writeReg(registers::FP, &flash_period, sizeof(flash_period));
        bool ramp_written = writeReg(registers::RAMP, &t_rise_fall, sizeof(t_rise_fall));

        bool pwm_written = writeReg(registers::PWM2, &time_on, sizeof(time_on));

        if (color_written && channel_written && ctrl_written && flash_written && ramp_written &&
            pwm_written) {
                getColor(&current_color);
        }
}

void KTD2026::getColor(RGBColor * color) {
        uint8_t channel_enable = 0;

        if (color == NULL) {
                LOG_WRN("KTD2026 getColor called without output buffer");
                return;
        }

        readReg(registers::EN_CH, &channel_enable, sizeof(channel_enable));
        readReg(registers::I_R, *color, sizeof(RGBColor));

        for (int i = 0; i < 3; i++) {
                (*color)[i] ++;
                if ((channel_enable & (0x3 << (2 * i))) == 0) {
                        (*color)[i] = 0;
                }
        }
}

void KTD2026::resetRegisterCache() {
        memcpy(_register_cache, KTD2026_RESET_DEFAULTS, sizeof(_register_cache));
}

KTD2026 led_controller;
