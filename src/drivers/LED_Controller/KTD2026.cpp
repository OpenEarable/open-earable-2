#include "KTD2026.h"

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LED, CONFIG_MAIN_LOG_LEVEL);

bool KTD2026::readReg(uint8_t reg, uint8_t * buffer, uint16_t len) {
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
        int ret = pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(load_switch)));
        if (ret == 0) {
                //printk("Sucessful getting device.\n");
                LOG_INF("Sucessful getting device.\n");
        } else {
                //printk("Error getting device.\n");
                LOG_WRN("Error getting device.\n");
        }

        k_msleep(10);

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
        int ret = pm_device_runtime_put(DEVICE_DT_GET(DT_NODELABEL(load_switch)));
}

void KTD2026::setColor(RGBColor color) {
        uint8_t channel_enable = 0;
        for (int i = 0; i < 3; i++) {
                if (color[i] > 0) {
                        channel_enable |= 1 << (2 * i);
                        color[i]--;
                }
                //writeReg(registers::I_R + i, &color[i], sizeof(uint8_t));
        }

        writeReg(registers::I_R, color, sizeof(RGBColor));
        //k_usleep(10);
        writeReg(registers::EN_CH, &channel_enable, sizeof(channel_enable));
}

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