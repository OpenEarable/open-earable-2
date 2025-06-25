#ifndef _KTD2026_H
#define _KTD2026_H

#include <zephyr/kernel.h>
#include <math.h>
//#include <Wire.h>
#include <TWIM.h>

#include "openearable_common.h"

class KTD2026 {
public:
    enum registers : uint8_t {
        CTRL = 0x00,
        FP = 0x01,
        PWM1 = 0x02,
        PWM2 = 0x03,
        EN_CH = 0x04,
        RAMP = 0x05,
        I_R = 0x06,
        I_G = 0x07,
        I_B = 0x08,
    };

    RGBColor current_color;

    void begin();
    void reset();
    void power_off();
    void setColor(const RGBColor& color);
    void getColor(RGBColor * color);

    void blink(const RGBColor& color, const int time_on_millis, const int period_millis);
    void pulse(const RGBColor& color, const int time_on_millis, const int time_rise_millis, const int time_fall_millis, const int period_millis);
    void pulse2(const RGBColor& color, const RGBColor& color2, const int time_on_millis, const int time_rise_millis, const int time_fall_millis, const int period_millis);
    //void setPower();
private:
    bool readReg(uint8_t reg, uint8_t *buffer, uint16_t len);
    void writeReg(uint8_t reg, uint8_t *buffer, uint16_t len);

    int address = 0x30;

    //TwoWire * _pWire = &Wire;
    TWIM * _i2c = &I2C1;

    bool _active = false;
};

extern KTD2026 led_controller;

#endif