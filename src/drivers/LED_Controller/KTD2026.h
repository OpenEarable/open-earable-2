#ifndef _KTD2026_H
#define _KTD2026_H

#include <zephyr/kernel.h>
#include <math.h>
#include <Wire.h>

typedef uint8_t RGBColor[3];

class KTD2026 {
public:
    enum registers : uint8_t {
        CTRL = 0x00,
        EN_CH = 0x04,
        I_R = 0x06,
        I_G = 0x07,
        I_B = 0x08,
    };

    RGBColor current_color;

    void setup();
    void reset();
    void power_off();
    void setColor(RGBColor color);
    void getColor(RGBColor * color);
    //void setPower();
private:
    bool readReg(uint8_t reg, uint8_t *buffer, uint16_t len);
    void writeReg(uint8_t reg, uint8_t *buffer, uint16_t len);

    int address = 0x30;

    TwoWire * _pWire = &Wire;
};

extern KTD2026 led_controller;

#endif