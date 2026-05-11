#include "KTD2026.h"

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include "openearable_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LED, CONFIG_MAIN_LOG_LEVEL);

bool KTD2026::readReg(uint8_t reg, uint8_t * buffer, uint16_t len) {
        _i2c->acquire();

        int ret = i2c_burst_read(_i2c->master, address, reg, buffer, len);
        if (ret) LOG_WRN("I2C read failed: %d\n", ret);

        _i2c->release();

        return (ret == 0);
}

void KTD2026::writeReg(uint8_t reg, uint8_t *buffer, uint16_t len) {
        _i2c->acquire();

        int ret = i2c_burst_write(_i2c->master, address, reg, buffer, len);
        if (ret) LOG_WRN("I2C write reg 0x%02x failed: %d", reg, ret);

        _i2c->release();
}

void KTD2026::begin() {
        if (_active) return;

	_active = true;

        /* KTD2026 VIN is on ls_3_3; i2c1 pull-ups are on the permanent supply. */
        pm_device_runtime_get(ls_3_3);

        k_usleep(200);  /* datasheet recovery time after reset command */
        _i2c->begin();
        k_usleep(200);  /* datasheet recovery time after reset command */

        reset();
}

void KTD2026::reset() {
        /* Datasheet page 13: after power-up or VIN dropping below 2.7V, write
         * Reg0[2:0]=111 ("Reset Complete Chip") then wait 200 µs.
         *
         * Empirical observation: on any MCU-only reboot (sys_reboot, incl.
         * SYS_REBOOT_COLD), the chip NACKs this reset write, but still ACKs
         * writes to other registers (Reg 4/6 etc.), so subsequent LED ops work
         * fine. The exact reason is not documented.
         *
         * We issue the command anyway so true cold power cycles (battery
         * disconnect, or sys_poweroff→wake which disables the LS/LDO) still
         * get the datasheet-recommended sequence, and log the NACK at DBG. */
        _i2c->acquire();
        uint8_t val = 0x7;
        int ret = i2c_burst_write(_i2c->master, address, registers::CTRL, &val, sizeof(val));
        _i2c->release();
        if (ret) {
                LOG_DBG("reset NACK (expected when VIN did not drop below UVLO): %d", ret);
        }
        k_usleep(200);  /* datasheet recovery time after reset command */
}

void KTD2026::power_off() {
        if (!_active) return;

        /* Reg0 = 0x08 → Reg0[4:3]=01 = "Device ON when SCL=H AND SDA toggling;
         * shutdown when SCL goes low or SDA stops toggling" (datasheet p.13).
         * Bus goes idle right after this write, so the chip drops into
         * shutdown mode (<1 µA per datasheet p.1) before we release ls_3_3. */
        uint8_t val = 0x8;
        writeReg(registers::CTRL, &val, sizeof(val));
        pm_device_runtime_put(ls_3_3);

	_active = false;
}

void KTD2026::setColor(const RGBColor& color) {
        uint8_t channel_enable = 0;
        RGBColor _color;
        memcpy(_color, color, sizeof(RGBColor));
        
        for (int i = 0; i < 3; i++) {
                if (_color[i] > 0) {
                        channel_enable |= 1 << (2 * i);
                        _color[i]--;
                }
        }

        writeReg(registers::I_R, _color, sizeof(RGBColor));
        writeReg(registers::EN_CH, &channel_enable, sizeof(channel_enable));
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

        writeReg(registers::I_R, _color, sizeof(RGBColor));
        writeReg(registers::EN_CH, &channel_enable, sizeof(channel_enable));

        writeReg(registers::FP, &flash_period, sizeof(flash_period));
        writeReg(registers::RAMP, &t_rise_fall, sizeof(t_rise_fall));
        writeReg(registers::PWM1, &time_on, sizeof(time_on));
        writeReg(registers::PWM2, &time_on_2, sizeof(time_on_2));
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

        writeReg(registers::I_R, _color, sizeof(RGBColor));
        writeReg(registers::EN_CH, &channel_enable, sizeof(channel_enable));

        uint8_t val = 0x2; // slot 3
        writeReg(registers::CTRL, &val, sizeof(val));

        writeReg(registers::FP, &flash_period, sizeof(flash_period));
        writeReg(registers::RAMP, &t_rise_fall, sizeof(t_rise_fall));

        writeReg(registers::PWM2, &time_on, sizeof(time_on));
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