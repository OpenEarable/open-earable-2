#include "AD7124.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(AD7124, 3);

// Communication command
#define AD7124_COMM_REG_WEN    (0 << 7)
#define AD7124_COMM_REG_WR     (0 << 6)
#define AD7124_COMM_REG_RD     (1 << 6)
#define AD7124_COMM_REG_RA(x)  ((x) & 0x3F)

// Status register bits
#define AD7124_STATUS_REG_RDY        (1 << 7)
#define AD7124_STATUS_REG_ERROR_FLAG (1 << 6)
#define AD7124_STATUS_REG_CH(x)      (((x) >> 0) & 0x0F)

// Control register bits
#define AD7124_CTRL_REG_DOUT_RDY_DEL  (1 << 12)
#define AD7124_CTRL_REG_CONT_READ     (1 << 11)
#define AD7124_CTRL_REG_DATA_STATUS   (1 << 10)
#define AD7124_CTRL_REG_CS_EN         (1 << 9)
#define AD7124_CTRL_REG_REF_EN        (1 << 8)
#define AD7124_CTRL_REG_POWER_MODE(x) (((x) & 0x3) << 6)
#define AD7124_CTRL_REG_MODE(x)       (((x) & 0xF) << 2)
#define AD7124_CTRL_REG_CLK_SEL(x)    (((x) & 0x3) << 0)

// Config register bits
#define AD7124_CFG_REG_BIPOLAR     (1 << 11)
#define AD7124_CFG_REG_BURNOUT(x)  (((x) & 0x3) << 9)
#define AD7124_CFG_REG_REF_BUFP    (1 << 8)
#define AD7124_CFG_REG_REF_BUFM    (1 << 7)
#define AD7124_CFG_REG_AIN_BUFP    (1 << 6)
#define AD7124_CFG_REG_AINN_BUFM   (1 << 5)
#define AD7124_CFG_REG_REF_SEL(x)  (((x) & 0x3) << 3)
#define AD7124_CFG_REG_PGA(x)      (((x) & 0x7) << 0)

// Filter register bits
#define AD7124_FILT_REG_FILTER(x)      (((x) & 0x7) << 21)
#define AD7124_FILT_REG_REJ60          (1 << 20)
#define AD7124_FILT_REG_POST_FILTER(x) (((x) & 0x7) << 17)
#define AD7124_FILT_REG_SINGLE_CYCLE   (1 << 16)
#define AD7124_FILT_REG_FS(x)          (((x) & 0x7FF) << 0)

// Channel register bits
#define AD7124_CH_MAP_REG_CH_ENABLE    (1 << 15)
#define AD7124_CH_MAP_REG_SETUP(x)     (((x) & 0x7) << 12)
#define AD7124_CH_MAP_REG_AINP(x)      (((x) & 0x1F) << 5)
#define AD7124_CH_MAP_REG_AINM(x)      (((x) & 0x1F) << 0)

AD7124::AD7124(const struct device *spi_dev, struct spi_cs_control *cs_ctrl)
    : spi_dev(spi_dev), ref_voltage(2.5f), gain_value(1), bipolar_mode(true) {
    
    // Configure for 4-wire SPI mode with CS on P0.20
    // Try SPI Mode 0: CPOL=0 (clock idle low), CPHA=0 (sample on leading edge)
    // AD7124 datasheet shows it supports both mode 0 and mode 3
    spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
    spi_cfg.frequency = 500000; 
    spi_cfg.slave = 0;
    spi_cfg.cs = *cs_ctrl;
}

int AD7124::init() {
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }
    
    return 0;
}

int AD7124::reset() {
    // Send 64 1's to reset the device
    uint8_t reset_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    
    struct spi_buf tx_buf = {
        .buf = reset_data,
        .len = sizeof(reset_data)
    };
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };
    
    int ret = spi_write(spi_dev, &spi_cfg, &tx);
    if (ret < 0) {
        LOG_ERR("SPI reset failed: %d", ret);
        return ret;
    }
    
    // AD7124 datasheet requires 500us minimum after reset
    k_msleep(2);
    return 0;
}

int AD7124::readRegister(uint8_t addr, uint32_t *value, uint8_t size) {
    uint8_t tx_data[5] = {0};
    uint8_t rx_data[5] = {0};
    
    // Communication register (read command)
    tx_data[0] = AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(addr);
    
    size_t buf_len = (size_t)(1 + size);
    struct spi_buf tx_bufs[] = {
        {.buf = tx_data, .len = buf_len}
    };
    struct spi_buf rx_bufs[] = {
        {.buf = rx_data, .len = buf_len}
    };
    struct spi_buf_set tx = {.buffers = tx_bufs, .count = 1};
    struct spi_buf_set rx = {.buffers = rx_bufs, .count = 1};
    
    LOG_INF("Reading reg 0x%02X, sending cmd: 0x%02X", addr, tx_data[0]);
    
    int ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
    if (ret < 0) {
        LOG_ERR("SPI read failed: %d", ret);
        return ret;
    }
    
    LOG_INF("RX bytes: [0x%02X 0x%02X 0x%02X 0x%02X]", rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
    
    // Convert bytes to value (big endian)
    *value = 0;
    for (int i = 0; i < size; i++) {
        *value = (*value << 8) | rx_data[1 + i];
    }
    
    return 0;
}

int AD7124::writeRegister(uint8_t addr, uint32_t value, uint8_t size) {
    uint8_t tx_data[5] = {0};
    
    // Communication register (write command)
    tx_data[0] = AD7124_COMM_REG_WR | AD7124_COMM_REG_RA(addr);
    
    // Convert value to bytes (big endian)
    for (int i = 0; i < size; i++) {
        tx_data[size - i] = (value >> (i * 8)) & 0xFF;
    }
    
    size_t buf_len = (size_t)(1 + size);
    struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = buf_len
    };
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };
    
    LOG_INF("Writing reg 0x%02X, cmd: 0x%02X, value: 0x%08X", addr, tx_data[0], value);
    
    int ret = spi_write(spi_dev, &spi_cfg, &tx);
    if (ret < 0) {
        LOG_ERR("SPI write failed: %d", ret);
        return ret;
    }
    
    // Temporarily disable verification to debug communication
    LOG_INF("Register 0x%02X written (verification disabled)", addr);
    return 0;
    
    // // Verify the write by reading back
    // uint32_t read_value;
    // ret = readRegister(addr, &read_value, size);
    // if (ret < 0) {
    //     LOG_ERR("Register readback failed for addr 0x%02X: %d", addr, ret);
    //     return ret;
    // }
    // 
    // if (read_value != value) {
    //     LOG_ERR("Register verification failed! Addr: 0x%02X, Expected: 0x%08X, Read: 0x%08X", 
    //             addr, value, read_value);
    //     return -EIO;
    // }
    // 
    // LOG_DBG("Register 0x%02X verified: 0x%08X", addr, read_value);
    // return 0;
}

int AD7124::setAdcControl(AD7124_OperatingModes mode, AD7124_PowerModes power_mode, bool ref_en) {
    uint32_t value = AD7124_CTRL_REG_MODE(mode) |
                     AD7124_CTRL_REG_POWER_MODE(power_mode) |
                     AD7124_CTRL_REG_DATA_STATUS |
                     (ref_en ? AD7124_CTRL_REG_REF_EN : 0);
    
    return writeRegister(AD7124_REG_CONTROL, value, 2);
}

int AD7124::setConfig(uint8_t setup, AD7124_RefSources ref, AD7124_GainSel gain, bool bipolar) {
    uint32_t value = AD7124_CFG_REG_REF_SEL(ref) |
                     AD7124_CFG_REG_PGA(gain) |
                     (bipolar ? AD7124_CFG_REG_BIPOLAR : 0) |
                     AD7124_CFG_REG_REF_BUFP | AD7124_CFG_REG_REF_BUFM |
                     AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_AINN_BUFM;
    
    // Store for voltage conversion
    bipolar_mode = bipolar;
    gain_value = 1 << gain;
    
    return writeRegister(AD7124_REG_CONFIG_0 + setup, value, 2);
}

int AD7124::setFilter(uint8_t setup, AD7124_Filters filter, uint16_t fs, 
                      AD7124_PostFilters postfilter, bool rej60) {
    uint32_t value = AD7124_FILT_REG_FILTER(filter) |
                     AD7124_FILT_REG_POST_FILTER(postfilter) |
                     AD7124_FILT_REG_FS(fs) |
                     (rej60 ? AD7124_FILT_REG_REJ60 : 0);
    
    return writeRegister(AD7124_REG_FILTER_0 + setup, value, 3);
}

int AD7124::setChannel(uint8_t ch, uint8_t setup, AD7124_InputSel aiPos,
                       AD7124_InputSel aiNeg, bool enable) {
    uint32_t value = AD7124_CH_MAP_REG_SETUP(setup) |
                     AD7124_CH_MAP_REG_AINP(aiPos) |
                     AD7124_CH_MAP_REG_AINM(aiNeg) |
                     (enable ? AD7124_CH_MAP_REG_CH_ENABLE : 0);
    
    return writeRegister(AD7124_REG_CHANNEL_0 + ch, value, 2);
}

int AD7124::waitForConvReady(uint32_t timeout_ms) {
    uint32_t status;
    uint64_t start_time = k_uptime_get();
    
    while (k_uptime_get() - start_time < timeout_ms) {
        int ret = readRegister(AD7124_REG_STATUS, &status, 1);
        if (ret < 0) {
            return ret;
        }
        
        if ((status & AD7124_STATUS_REG_RDY) == 0) {
            return 0; // Ready
        }
        
        k_usleep(10);
    }
    
    return -ETIMEDOUT;
}

int AD7124::readRaw(int32_t *value) {
    uint32_t data;
    int ret = readRegister(AD7124_REG_DATA, &data, 3);
    if (ret < 0) {
        return ret;
    }
    
    // Convert to signed 24-bit
    if (data & 0x800000) {
        *value = (int32_t)(data | 0xFF000000);
    } else {
        *value = (int32_t)data;
    }
    
    return 0;
}

float AD7124::readVolts(uint8_t ch) {
    int32_t raw;
    int ret = readRaw(&raw);
    if (ret < 0) {
        LOG_ERR("readRaw failed: %d", ret);
        return -1000.0f;
    }
    
    // Convert to voltage
    float voltage;
    if (bipolar_mode) {
        voltage = ((float)raw / 8388608.0f) * (ref_voltage / (float)gain_value);
    } else {
        voltage = ((float)raw / 16777216.0f) * (ref_voltage / (float)gain_value);
    }
    
    return voltage;
}

int AD7124::getCurrentChannel() {
    uint32_t status;
    int ret = readRegister(AD7124_REG_STATUS, &status, 1);
    if (ret < 0) {
        return ret;
    }
    
    return AD7124_STATUS_REG_CH(status);
}
