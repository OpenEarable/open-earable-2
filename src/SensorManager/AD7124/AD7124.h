#ifndef AD7124_H
#define AD7124_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <stdint.h>

/**
 * @class AD7124
 * @brief AD7124 ADC driver using official Analog Devices communication patterns
 * 
 * This driver implements the exact register access sequences and timing from
 * Analog Devices' official AD7124 driver to ensure reliable SPI communication.
 */
class AD7124 {
public:
    // Operating modes
    enum class OperatingMode {
        CONTINUOUS = 0,
        SINGLE = 1,
        STANDBY = 2,
        POWER_DOWN = 3,
        IDLE = 4,
        INTERNAL_OFFSET_CAL = 5,
        INTERNAL_GAIN_CAL = 6,
        SYSTEM_OFFSET_CAL = 7,
        SYSTEM_GAIN_CAL = 8
    };

    // Power modes
    enum class PowerMode {
        LOW_POWER = 0,
        MID_POWER = 1,
        FULL_POWER = 2
    };

    // PGA gain values
    enum class PGA {
        GAIN_1 = 0,
        GAIN_2 = 1,
        GAIN_4 = 2,
        GAIN_8 = 3,
        GAIN_16 = 4,
        GAIN_32 = 5,
        GAIN_64 = 6,
        GAIN_128 = 7
    };

    // Filter types
    enum class FilterType {
        SINC4 = 0,
        SINC3 = 2,
        FS = 4,           // Fast Settling
        POST = 7          // Post Filter
    };

    // Reference source
    enum class ReferenceSource {
        EXTERNAL = 0,
        INTERNAL = 2,
        AVDD_AVSS = 3
    };

    // Analog input selections
    enum class AnalogInput {
        AIN0 = 0, AIN1 = 1, AIN2 = 2, AIN3 = 3,
        AIN4 = 4, AIN5 = 5, AIN6 = 6, AIN7 = 7,
        AIN8 = 8, AIN9 = 9, AIN10 = 10, AIN11 = 11,
        AIN12 = 12, AIN13 = 13, AIN14 = 14, AIN15 = 15,
        TEMP_SENSOR = 16, AVSS = 17, INTERNAL_REF = 18,
        DGND = 19, AVDD6_P = 20, AVDD6_M = 21,
        IOVDD6_P = 22, IOVDD6_M = 23, ALDO6_P = 24,
        ALDO6_M = 25, DLDO6_P = 26, DLDO6_M = 27,
        V_20MV_P = 28, V_20MV_M = 29
    };

    // Register addresses
    enum class Register {
        STATUS = 0x00,
        ADC_CONTROL = 0x01,
        DATA = 0x02,
        IO_CONTROL_1 = 0x03,
        IO_CONTROL_2 = 0x04,
        ID = 0x05,
        ERROR = 0x06,
        ERROR_EN = 0x07,
        MCLK_COUNT = 0x08,
        CHANNEL_0 = 0x09,
        CONFIG_0 = 0x19,
        FILTER_0 = 0x21,
        OFFSET_0 = 0x29,
        GAIN_0 = 0x31
    };

    AD7124();
    
    // Configure GPIO pins for software SPI (3-wire mode, CS hardwired to GND)
    void setSoftwareSPI(const struct device *gpio_dev, uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin);
    
    int init();
    int reset();
    
    int setAdcControl(OperatingMode mode, PowerMode power_mode, bool ref_en = true);
    int setConfig(uint8_t setup, ReferenceSource ref, PGA gain, bool bipolar);
    int setFilter(uint8_t setup, FilterType filter_type, uint16_t fs, bool rej60 = false);
    int setChannel(uint8_t ch, uint8_t setup, AnalogInput ainp, AnalogInput ainm, bool enable = false);
    
    int readRaw(int32_t *value);
    float readVolts(uint8_t ch);
    
    int waitForConvReady(uint32_t timeout_ms);
    int getCurrentChannel();
    
    int readRegister(uint8_t addr, uint32_t *value, uint8_t size);

private:
    // Software SPI GPIO pins (3-wire mode, CS hardwired to GND)
    const struct device *gpio_dev;
    uint8_t sck_pin;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    
    // Register info structure
    struct RegisterInfo {
        uint8_t addr;
        uint32_t value;
        uint8_t size;
        uint8_t rw;  // 1=write, 2=read, 3=read/write
    };
    
    // Register map
    RegisterInfo regs[57];
    
    int writeRegister(uint8_t addr, uint32_t value, uint8_t size);
    int noCheckReadRegister(uint8_t addr, uint32_t *value, uint8_t size);
    int noCheckWriteRegister(uint8_t addr, uint32_t value, uint8_t size);
    int waitForSpiReady(uint32_t timeout);
    int waitToPowerOn(uint32_t timeout);
    uint8_t computeCRC8(uint8_t *buf, uint8_t size);
    
    // Software SPI bit-banging functions (3-wire mode)
    void softSpiInit();
    uint8_t softSpiTransferByte(uint8_t data);
    
    bool use_crc;
    bool check_ready;
    uint32_t spi_rdy_poll_cnt;
    
    // Setup values for voltage conversion
    float ref_voltage;
    uint8_t gain_value;
    bool bipolar_mode;
};

#endif // AD7124_H
