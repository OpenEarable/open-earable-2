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

    RGBColor current_color = {0, 0, 0};

    /** @brief Enable the LED driver power rails and reset the KTD2026. */
    void begin();

    /** @brief Reset the KTD2026 register bank and clear the cached LED color. */
    void reset();

    /** @brief Put the LED driver into shutdown and release its power rails. */
    void power_off();

    /** @brief Set a steady RGB color. */
    void setColor(const RGBColor& color);

    /** @brief Return the cached RGB current values for enabled channels. */
    void getColor(RGBColor * color);

    /** @brief Blink an RGB color with a fixed on-time and period. */
    void blink(const RGBColor& color, const int time_on_millis, const int period_millis);

    /** @brief Pulse an RGB color using the KTD2026 timer and ramp registers. */
    void pulse(const RGBColor& color, const int time_on_millis, const int time_rise_millis, const int time_fall_millis, const int period_millis);

    /** @brief Pulse two RGB colors using the KTD2026 timer slots. */
    void pulse2(const RGBColor& color, const RGBColor& color2, const int time_on_millis, const int time_rise_millis, const int time_fall_millis, const int period_millis);
    //void setPower();
private:
    bool readReg(uint8_t reg, uint8_t *buffer, uint16_t len);
    bool writeReg(uint8_t reg, const uint8_t *buffer, uint16_t len, bool ignore_reset_nack = false);
    void resetRegisterCache();

    int address = 0x30;

    //TwoWire * _pWire = &Wire;
    TWIM * _i2c = &I2C1;

    static constexpr uint8_t REGISTER_COUNT = I_B + 1;

    uint8_t _register_cache[REGISTER_COUNT] = {
        0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x4F, 0x4F, 0x4F
    };

    bool _active = false;
};

extern KTD2026 led_controller;

#endif
