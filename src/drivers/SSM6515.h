#ifndef _SSM6515_H
#define _SSM6515_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <Wire.h>

#define OUT_VOLUME_DEFAULT 0x80
#define MAX_VOLUME_REG_VAL 0xA0
#define MAX_VOLUME_DB 24

//#define OUT_VOLUME_DEFAULT 0x40
//#define MAX_VOLUME_REG_VAL 0x80

#define SSM6515_I2C_TIMEOUT_US 66

class SSM6515 {
public:
    enum registers : uint8_t {
        VENDOR_ID = 0x00,
        DEVICE_ID1 = 0x01,
        DEVICE_ID2 = 0x02,
        REVISION = 0x03,
        PWR_CTRL = 0x04,
        CLK_CTRL = 0x05,
        PDM_CTRL = 0x06,
        DAC_CTRL1 = 0x07,
        DAC_CTRL2 = 0x08,
        DAC_CTRL3 = 0x09,
        DAC_VOL = 0x0A,
        DAC_CLIP = 0x0B,
        SPT_CTRL1 = 0x0C,
        SPT_CTRL2 = 0x0D,
        AMP_CTRL = 0x0E,
        LIM_CTRL1 = 0x0F,
        LIM_CTRL2 = 0x10,
        FAULT_CTRL = 0x11,
        STATUS_CLR = 0x12,
        STATUS = 0x13,
        RESET = 0x14
    };

    SSM6515(TwoWire * wire);

    int begin();
    int end();
    int setup();
    int mute(bool active);
    int set_volume(uint8_t volume);

    int soft_reset(bool full_reset = false);

    uint8_t get_volume();
private:
    bool readReg(uint8_t reg, uint8_t * buffer, uint16_t len);
    void writeReg(uint8_t reg, uint8_t * buffer, uint16_t len);

    const int address = DT_REG_ADDR(DT_NODELABEL(ssm6515));

    uint64_t last_i2c;

    TwoWire *_pWire;

    //const struct gpio_dt_spec pg_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(bq25120a), pg_gpios);
};

extern SSM6515 dac;

#endif