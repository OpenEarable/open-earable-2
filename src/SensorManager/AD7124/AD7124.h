#ifndef AD7124_H
#define AD7124_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <stdint.h>

// Operating modes
enum AD7124_OperatingModes {
    AD7124_OpMode_Continuous = 0,
    AD7124_OpMode_SingleConv,
    AD7124_OpMode_Standby,
    AD7124_OpMode_PowerDown,
    AD7124_OpMode_Idle,
    AD7124_OpMode_InternalOffsetCalibration,
    AD7124_OpMode_InternalGainCalibration,
    AD7124_OpMode_SystemOffsetCalibration,
    AD7124_OpMode_SystemGainCalibration
};

// Power modes
enum AD7124_PowerModes {
    AD7124_LowPower = 0,
    AD7124_MidPower,
    AD7124_FullPower
};

// Input selection
enum AD7124_InputSel {
    AD7124_Input_AIN0 = 0,
    AD7124_Input_AIN1,
    AD7124_Input_AIN2,
    AD7124_Input_AIN3,
    AD7124_Input_AIN4,
    AD7124_Input_AIN5,
    AD7124_Input_AIN6,
    AD7124_Input_AIN7,
    AD7124_Input_AIN8,
    AD7124_Input_AIN9,
    AD7124_Input_AIN10,
    AD7124_Input_AIN11,
    AD7124_Input_AIN12,
    AD7124_Input_AIN13,
    AD7124_Input_AIN14,
    AD7124_Input_AIN15,
    AD7124_Input_TEMP = 16,
    AD7124_Input_AVSS,
    AD7124_Input_REF,
    AD7124_Input_DGND,
    AD7124_Input_AVDD6P,
    AD7124_Input_AVDD6M,
    AD7124_Input_IOVDD6P,
    AD7124_Input_IOVDD6M,
    AD7124_Input_ALDO6P,
    AD7124_Input_ALDO6M,
    AD7124_Input_DLDO6P,
    AD7124_Input_DLDO6M,
    AD7124_Input_V20mVP,
    AD7124_Input_V20mVM
};

// Gain selection
enum AD7124_GainSel {
    AD7124_Gain_1 = 0,
    AD7124_Gain_2,
    AD7124_Gain_4,
    AD7124_Gain_8,
    AD7124_Gain_16,
    AD7124_Gain_32,
    AD7124_Gain_64,
    AD7124_Gain_128
};

// Reference sources
enum AD7124_RefSources {
    AD7124_Ref_ExtRef1 = 0x00,
    AD7124_Ref_ExtRef2 = 0x01,
    AD7124_Ref_Internal = 0x02,
    AD7124_Ref_Avdd = 0x03
};

// Filters
enum AD7124_Filters {
    AD7124_Filter_SINC4 = 0x00,
    AD7124_Filter_SINC3 = 0x02,
    AD7124_Filter_FAST4 = 0x04,
    AD7124_Filter_FAST3 = 0x05,
    AD7124_Filter_POST = 0x07
};

// Post filters
enum AD7124_PostFilters {
    AD7124_PostFilter_NoPost = 0,
    AD7124_PostFilter_dB47 = 2,
    AD7124_PostFilter_dB62 = 3,
    AD7124_PostFilter_dB86 = 5,
    AD7124_PostFilter_dB92 = 6
};

// Register addresses
enum AD7124_Registers {
    AD7124_REG_STATUS = 0x00,
    AD7124_REG_CONTROL = 0x01,
    AD7124_REG_DATA = 0x02,
    AD7124_REG_IOCON1 = 0x03,
    AD7124_REG_IOCON2 = 0x04,
    AD7124_REG_ID = 0x05,
    AD7124_REG_ERROR = 0x06,
    AD7124_REG_ERROR_EN = 0x07,
    AD7124_REG_MCLK_COUNT = 0x08,
    AD7124_REG_CHANNEL_0 = 0x09,
    AD7124_REG_CONFIG_0 = 0x19,
    AD7124_REG_FILTER_0 = 0x21,
    AD7124_REG_OFFSET_0 = 0x29,
    AD7124_REG_GAIN_0 = 0x31
};

// Register info structure
struct AD7124_Register {
    uint8_t addr;
    uint32_t value;
    uint8_t size;
};

class AD7124 {
public:
    AD7124(const struct device *spi_dev, struct spi_cs_control *cs_ctrl);
    
    int init();
    int reset();
    
    int setAdcControl(AD7124_OperatingModes mode, AD7124_PowerModes power_mode, bool ref_en = true);
    int setConfig(uint8_t setup, AD7124_RefSources ref, AD7124_GainSel gain, bool bipolar);
    int setFilter(uint8_t setup, AD7124_Filters filter, uint16_t fs, AD7124_PostFilters postfilter = AD7124_PostFilter_NoPost, bool rej60 = false);
    int setChannel(uint8_t ch, uint8_t setup, AD7124_InputSel aiPos, AD7124_InputSel aiNeg, bool enable = false);
    
    int readRaw(int32_t *value);
    float readVolts(uint8_t ch);
    
    int waitForConvReady(uint32_t timeout_ms);
    int getCurrentChannel();

private:
    const struct device *spi_dev;
    struct spi_config spi_cfg;
    
    int readRegister(uint8_t addr, uint32_t *value, uint8_t size);
    int writeRegister(uint8_t addr, uint32_t value, uint8_t size);
    
    // Setup values for voltage conversion
    float ref_voltage;
    uint8_t gain_value;
    bool bipolar_mode;
};

#endif // AD7124_H
