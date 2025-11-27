#include "AD7124.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(AD7124, 3);

// Post-reset delay (4ms for margin)
#define AD7124_POST_RESET_DELAY 4

// Communication register bits
#define AD7124_COMM_REG_WEN (0 << 7)
#define AD7124_COMM_REG_WR (0 << 6)
#define AD7124_COMM_REG_RD (1 << 6)
#define AD7124_COMM_REG_RA(x) ((x) & 0x3F)

// Status register bits
#define AD7124_STATUS_REG_RDY (1 << 7)
#define AD7124_STATUS_REG_ERROR_FLAG (1 << 6)
#define AD7124_STATUS_REG_POR_FLAG (1 << 4)
#define AD7124_STATUS_REG_CH_ACTIVE(x) ((x) & 0xF)

// ADC Control register bits
#define AD7124_ADC_CTRL_REG_DOUT_RDY_DEL (1 << 12)
#define AD7124_ADC_CTRL_REG_CONT_READ (1 << 11)
#define AD7124_ADC_CTRL_REG_DATA_STATUS (1 << 10)
#define AD7124_ADC_CTRL_REG_CS_EN (1 << 9)
#define AD7124_ADC_CTRL_REG_REF_EN (1 << 8)
#define AD7124_ADC_CTRL_REG_POWER_MODE(x) (((x) & 0x3) << 6)
#define AD7124_ADC_CTRL_REG_MODE(x) (((x) & 0xF) << 2)
#define AD7124_ADC_CTRL_REG_CLK_SEL(x) (((x) & 0x3) << 0)

// Configuration register bits
#define AD7124_CFG_REG_BIPOLAR (1 << 11)
#define AD7124_CFG_REG_BURNOUT(x) (((x) & 0x3) << 9)
#define AD7124_CFG_REG_REF_BUFP (1 << 8)
#define AD7124_CFG_REG_REF_BUFM (1 << 7)
#define AD7124_CFG_REG_AIN_BUFP (1 << 6)
#define AD7124_CFG_REG_AINN_BUFM (1 << 5)
#define AD7124_CFG_REG_REF_SEL(x) (((x) & 0x3) << 3)
#define AD7124_CFG_REG_PGA(x) (((x) & 0x7) << 0)

// Filter register bits
#define AD7124_FILT_REG_FILTER(x) (((x) & 0x7) << 21)
#define AD7124_FILT_REG_REJ60 (1 << 20)
#define AD7124_FILT_REG_POST_FILTER(x) (((x) & 0x7) << 17)
#define AD7124_FILT_REG_SINGLE_CYCLE (1 << 16)
#define AD7124_FILT_REG_FS(x) (((x) & 0x7FF) << 0)

// Channel register bits
#define AD7124_CH_MAP_REG_CH_ENABLE (1 << 15)
#define AD7124_CH_MAP_REG_SETUP(x) (((x) & 0x7) << 12)
#define AD7124_CH_MAP_REG_AINP(x) (((x) & 0x1F) << 5)
#define AD7124_CH_MAP_REG_AINM(x) (((x) & 0x1F) << 0)

// Error register bits
#define AD7124_ERR_REG_LDO_CAP_ERR (1 << 19)
#define AD7124_ERR_REG_ADC_CAL_ERR (1 << 18)
#define AD7124_ERR_REG_ADC_CONV_ERR (1 << 17)
#define AD7124_ERR_REG_ADC_SAT_ERR (1 << 16)
#define AD7124_ERR_REG_AINP_OV_ERR (1 << 15)
#define AD7124_ERR_REG_AINP_UV_ERR (1 << 14)
#define AD7124_ERR_REG_AINM_OV_ERR (1 << 13)
#define AD7124_ERR_REG_AINM_UV_ERR (1 << 12)
#define AD7124_ERR_REG_REF_DET_ERR (1 << 11)
#define AD7124_ERR_REG_DLDO_PSM_ERR (1 << 9)
#define AD7124_ERR_REG_ALDO_PSM_ERR (1 << 7)
#define AD7124_ERR_REG_SPI_IGNORE_ERR (1 << 6)
#define AD7124_ERR_REG_SPI_SLCK_CNT_ERR (1 << 5)
#define AD7124_ERR_REG_SPI_READ_ERR (1 << 4)
#define AD7124_ERR_REG_SPI_WRITE_ERR (1 << 3)
#define AD7124_ERR_REG_SPI_CRC_ERR (1 << 2)
#define AD7124_ERR_REG_MM_CRC_ERR (1 << 1)
#define AD7124_ERR_REG_ROM_CRC_ERR (1 << 0)

AD7124::AD7124()
    : use_crc(false), check_ready(true), spi_rdy_poll_cnt(10000),
      ref_voltage(2.5f), gain_value(1), bipolar_mode(true) {
    
    // Initialize register map
    // Format: {addr, value, size, rw}
    regs[0] = {0x00, 0x00, 1, 2};      // Status
    regs[1] = {0x01, 0x0000, 2, 1};    // ADC_Control
    regs[2] = {0x02, 0x0000, 3, 2};    // Data
    regs[3] = {0x03, 0x0000, 3, 1};    // IOCon1
    regs[4] = {0x04, 0x0000, 2, 1};    // IOCon2
    regs[5] = {0x05, 0x02, 1, 2};      // ID
    regs[6] = {0x06, 0x0000, 3, 2};    // Error
    regs[7] = {0x07, 0x0040, 3, 1};    // Error_En
    regs[8] = {0x08, 0x00, 1, 2};      // Mclk_Count
    
    // Channel registers 0-15
    for (int i = 0; i < 16; i++) {
        regs[9 + i] = {(uint8_t)(0x09 + i), (i == 0) ? (uint32_t)0x8001 : 0x0001, 2, 1};
    }
    
    // Config registers 0-7
    for (int i = 0; i < 8; i++) {
        regs[25 + i] = {(uint8_t)(0x19 + i), 0x0860, 2, 1};
    }
    
    // Filter registers 0-7
    for (int i = 0; i < 8; i++) {
        regs[33 + i] = {(uint8_t)(0x21 + i), 0x060180, 3, 1};
    }
    
    // Offset registers 0-7
    for (int i = 0; i < 8; i++) {
        regs[41 + i] = {(uint8_t)(0x29 + i), 0x800000, 3, 1};
    }
    
    // Gain registers 0-7
    for (int i = 0; i < 8; i++) {
        regs[49 + i] = {(uint8_t)(0x31 + i), 0x500000, 3, 1};
    }
}

/**
 * @brief Configure software SPI (bit-banging) mode - 3-wire, CS hardwired to GND
 */
void AD7124::setSoftwareSPI(const struct device *gpio_dev, uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin) {
    this->gpio_dev = gpio_dev;
    this->sck_pin = sck_pin;
    this->mosi_pin = mosi_pin;
    this->miso_pin = miso_pin;
}

/**
 * @brief Initialize software SPI GPIO pins (3-wire mode, CS hardwired to GND)
 */
void AD7124::softSpiInit() {
    // Configure SCK as output, initial HIGH (CPOL=1 for Mode 3)
    gpio_pin_configure(gpio_dev, sck_pin, GPIO_OUTPUT_HIGH);
    
    // Configure MOSI as output
    gpio_pin_configure(gpio_dev, mosi_pin, GPIO_OUTPUT_LOW);
    
    // Configure MISO as input
    gpio_pin_configure(gpio_dev, miso_pin, GPIO_INPUT);
}

/**
 * @brief Software SPI transfer one byte (Mode 3: CPOL=1, CPHA=1)
 * Mode 3: Clock idle HIGH, data changes on falling edge, sample on rising edge
 * AD7124: Outputs data on falling SCLK edge, samples input on rising SCLK edge
 */
uint8_t AD7124::softSpiTransferByte(uint8_t data) {
    uint8_t result = 0;
    
    // Transfer 8 bits, MSB first
    for (int i = 7; i >= 0; i--) {
        // Set MOSI bit (data setup before falling edge)
        gpio_pin_set(gpio_dev, mosi_pin, (data >> i) & 0x01);
        k_busy_wait(1);
        
        // Clock LOW (falling edge - AD7124 shifts data out on MISO)
        gpio_pin_set(gpio_dev, sck_pin, 0);
        
        // Wait for AD7124 to output data, then sample MISO while clock is LOW
        k_busy_wait(1);
        result |= (gpio_pin_get(gpio_dev, miso_pin) & 0x01) << i;
        
        // Clock HIGH (rising edge - AD7124 samples MOSI)
        gpio_pin_set(gpio_dev, sck_pin, 1);
        
        // Delay before next bit
        k_busy_wait(1);
    }
    
    return result;
}

int AD7124::init() {
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }
    softSpiInit();
    return 0;
}

/**
 * @brief Compute CRC8 checksum
 */
uint8_t AD7124::computeCRC8(uint8_t *buf, uint8_t size) {
    uint8_t crc = 0;
    
    while (size) {
        for (uint8_t i = 0x80; i != 0; i >>= 1) {
            bool cmp1 = (crc & 0x80) != 0;
            bool cmp2 = (*buf & i) != 0;
            if (cmp1 != cmp2) {
                crc <<= 1;
                crc ^= 0x07; // CRC polynomial
            } else {
                crc <<= 1;
            }
        }
        buf++;
        size--;
    }
    
    return crc;
}

/**
 * @brief Read register without device ready check
 */
int AD7124::noCheckReadRegister(uint8_t addr, uint32_t *value, uint8_t size) {
    if (size > 4) {
        return -EINVAL;
    }
    
    uint8_t buffer[8] = {0};
    uint8_t i = 0;
    
    // Build the command word
    buffer[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(addr);
    
    // Send command byte (response during this transfer should be ignored)
    softSpiTransferByte(buffer[0]);
    
    // Read data bytes
    for (i = 1; i <= size; i++) {
        buffer[i] = softSpiTransferByte(0x00);  // Send dummy byte, read response
    }
    
    // Ensure clock returns to idle (HIGH for Mode 3) and hold for transaction boundary
    gpio_pin_set(gpio_dev, sck_pin, 1);
    k_usleep(500);  // Longer inter-transaction delay for 3-wire mode
    
    // Build the result
    *value = 0;
    for (i = 1; i < size + 1; i++) {
        *value <<= 8;
        *value += buffer[i];
    }
    
    return 0;
}

/**
 * @brief Write register without device ready check
 */
int AD7124::noCheckWriteRegister(uint8_t addr, uint32_t value, uint8_t size) {
    if (size > 4) {
        return -EINVAL;
    }
    
    uint8_t wr_buf[8] = {0};
    uint8_t i = 0;
    int32_t reg_value = 0;
    
    // Build the command word
    wr_buf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR | AD7124_COMM_REG_RA(addr);
    
    // Fill the write buffer
    reg_value = value;
    for (i = 0; i < size; i++) {
        wr_buf[size - i] = reg_value & 0xFF;
        reg_value >>= 8;
    }
    
    // Send all bytes
    for (i = 0; i <= size; i++) {
        softSpiTransferByte(wr_buf[i]);
    }
    
    // Ensure clock returns to idle (HIGH for Mode 3) and hold for transaction boundary
    gpio_pin_set(gpio_dev, sck_pin, 1);
    k_usleep(100);
    
    return 0;
}

/**
 * @brief Wait for SPI ready
 */
int AD7124::waitForSpiReady(uint32_t timeout) {
    int32_t ret;
    bool ready = false;
    
    while (!ready && timeout--) {
        uint32_t error_val = 0;
        
        // Read the Error Register (don't use regular readRegister to avoid recursion)
        ret = noCheckReadRegister(static_cast<uint8_t>(Register::ERROR), &error_val, 3);
        if (ret) {
            return ret;
        }
        
        // Check the SPI IGNORE Error bit in the Error Register
        ready = (error_val & AD7124_ERR_REG_SPI_IGNORE_ERR) == 0;
        
        if (!ready) {
            k_usleep(10);
        }
    }
    
    if (!timeout) {
        return -ETIMEDOUT;
    }
    
    return 0;
}

/**
 * @brief Wait for device power-on
 */
int AD7124::waitToPowerOn(uint32_t timeout) {
    int32_t ret;
    bool powered_on = false;
    
    while (!powered_on && timeout--) {
        uint32_t status_val = 0;
        
        ret = noCheckReadRegister(static_cast<uint8_t>(Register::STATUS), &status_val, 1);
        if (ret) {
            return ret;
        }
        
        // Check the POR_FLAG bit in the Status Register
        powered_on = (status_val & AD7124_STATUS_REG_POR_FLAG) == 0;
        
        if (!powered_on) {
            k_usleep(100);
        }
    }
    
    if (!(timeout || powered_on)) {
        return -ETIMEDOUT;
    }
    
    return 0;
}

/**
 * @brief Reset the device
 */
int AD7124::reset() {
    int32_t ret = 0;
    
    // Send 64 consecutive 1s (8 bytes of 0xFF) to reset
    for (int i = 0; i < 8; i++) {
        softSpiTransferByte(0xFF);
    }
    
    // CRC is disabled after reset
    use_crc = false;
    
    // Read POR bit to clear
    ret = waitToPowerOn(spi_rdy_poll_cnt);
    if (ret) {
        return ret;
    }
    
    // Post-reset delay
    k_msleep(AD7124_POST_RESET_DELAY);
    
    return 0;
}

/**
 * @brief Read register with device ready check
 */
int AD7124::readRegister(uint8_t addr, uint32_t *value, uint8_t size) {
    int32_t ret;
    
    // Wait for device ready (except when reading Error register)
    if (addr != static_cast<uint8_t>(Register::ERROR) && check_ready) {
        ret = waitForSpiReady(spi_rdy_poll_cnt);
        if (ret) {
            return ret;
        }
    }
    
    return noCheckReadRegister(addr, value, size);
}

/**
 * @brief Write register with device ready check
 */
int AD7124::writeRegister(uint8_t addr, uint32_t value, uint8_t size) {
    int32_t ret;
    
    if (check_ready) {
        ret = waitForSpiReady(spi_rdy_poll_cnt);
        if (ret) {
            return ret;
        }
    }
    
    return noCheckWriteRegister(addr, value, size);
}

/**
 * @brief Set ADC control register
 */
int AD7124::setAdcControl(OperatingMode mode, PowerMode power_mode, bool ref_en) {
    uint32_t value = AD7124_ADC_CTRL_REG_MODE(static_cast<uint32_t>(mode)) |
                     AD7124_ADC_CTRL_REG_POWER_MODE(static_cast<uint32_t>(power_mode)) |
                     AD7124_ADC_CTRL_REG_DATA_STATUS |
                     (ref_en ? AD7124_ADC_CTRL_REG_REF_EN : 0);
    
    // Update local register copy
    regs[1].value = value;
    
    return writeRegister(static_cast<uint8_t>(Register::ADC_CONTROL), value, 2);
}

/**
 * @brief Configure a setup (config register)
 */
int AD7124::setConfig(uint8_t setup, ReferenceSource ref, PGA gain, bool bipolar) {
    if (setup > 7) {
        return -EINVAL;
    }
    
    uint32_t value = AD7124_CFG_REG_PGA(static_cast<uint32_t>(gain)) |
                     AD7124_CFG_REG_REF_SEL(static_cast<uint32_t>(ref)) |
                     (bipolar ? AD7124_CFG_REG_BIPOLAR : 0);
    
    // Update local register copy
    regs[25 + setup].value = value;
    
    return writeRegister(static_cast<uint8_t>(Register::CONFIG_0) + setup, value, 2);
}

/**
 * @brief Configure filter for a setup
 */
int AD7124::setFilter(uint8_t setup, FilterType filter_type, uint16_t fs, bool rej60) {
    if (setup > 7) {
        return -EINVAL;
    }
    
    uint32_t value = AD7124_FILT_REG_FILTER(static_cast<uint32_t>(filter_type)) |
                     AD7124_FILT_REG_FS(fs) |
                     (rej60 ? AD7124_FILT_REG_REJ60 : 0);
    
    // Update local register copy
    regs[33 + setup].value = value;
    
    return writeRegister(static_cast<uint8_t>(Register::FILTER_0) + setup, value, 3);
}

/**
 * @brief Configure a channel
 */
int AD7124::setChannel(uint8_t ch, uint8_t setup, AnalogInput ainp, 
                       AnalogInput ainm, bool enable) {
    if (ch > 15 || setup > 7) {
        return -EINVAL;
    }
    
    uint32_t value = AD7124_CH_MAP_REG_SETUP(setup) |
                     AD7124_CH_MAP_REG_AINP(static_cast<uint32_t>(ainp)) |
                     AD7124_CH_MAP_REG_AINM(static_cast<uint32_t>(ainm)) |
                     (enable ? AD7124_CH_MAP_REG_CH_ENABLE : 0);
    
    // Update local register copy
    regs[9 + ch].value = value;
    
    return writeRegister(static_cast<uint8_t>(Register::CHANNEL_0) + ch, value, 2);
}

/**
 * @brief Wait for conversion ready
 */
int AD7124::waitForConvReady(uint32_t timeout_ms) {
    int32_t ret;
    bool ready = false;
    uint32_t timeout = timeout_ms * 100; // Convert to 10us units
    
    while (!ready && timeout--) {
        uint32_t status = 0;
        
        // Read the Status Register
        ret = readRegister(static_cast<uint8_t>(Register::STATUS), &status, 1);
        if (ret) {
            return ret;
        }
        
        // Check the RDY bit in the Status Register (0 = ready)
        ready = (status & AD7124_STATUS_REG_RDY) == 0;
        
        if (!ready) {
            k_usleep(10);
        }
    }
    
    if (!timeout) {
        return -ETIMEDOUT;
    }
    
    return 0;
}

/**
 * @brief Read conversion result
 * NOTE: When DATA_STATUS bit is set in ADC_CONTROL, must read 4 bytes (3 data + 1 status)
 */
int AD7124::readRaw(int32_t *value) {
    int32_t ret;
    uint32_t data = 0;
    
    // Check if DATA_STATUS is enabled in ADC_CONTROL
    bool data_status_enabled = (regs[1].value & AD7124_ADC_CTRL_REG_DATA_STATUS) != 0;
    
    // Read the Data Register - 4 bytes if DATA_STATUS enabled, 3 otherwise
    uint8_t read_size = data_status_enabled ? 4 : 3;
    ret = noCheckReadRegister(static_cast<uint8_t>(Register::DATA), &data, read_size);
    if (ret) {
        return ret;
    }
    
    // If DATA_STATUS enabled, extract the 24-bit data (upper 3 bytes of 4-byte read)
    // and update the status register value from the lower byte
    if (data_status_enabled) {
        // Update STATUS register with the status byte
        regs[0].value = data & 0xFF;
        // Extract 24-bit data from upper 3 bytes
        data = data >> 8;
    }
    
    // Get the read result (keep as unsigned 24-bit for offset binary handling)
    *value = (int32_t)(data & 0xFFFFFF);
    
    return 0;
}

/**
 * @brief Read voltage from ADC
 */
float AD7124::readVolts(uint8_t ch) {
    int32_t raw;
    int ret = readRaw(&raw);
    if (ret < 0) {
        LOG_ERR("readRaw failed: %d", ret);
        return 0.0f;
    }
    
    // Convert to voltage
    float voltage;
    if (bipolar_mode) {
        // Bipolar uses offset binary: 0x800000 = 0V, 0x000000 = -Vref, 0xFFFFFF = +Vref
        // Subtract mid-scale (0x800000) to convert to signed, then scale
        int32_t signed_raw = raw - 0x800000;
        voltage = ((float)signed_raw / 8388608.0f) * (ref_voltage / (float)gain_value);
    } else {
        // Unipolar: 0 to Vref
        voltage = ((float)raw / 16777216.0f) * (ref_voltage / (float)gain_value);
    }
    
    return voltage;
}

/**
 * @brief Get current active channel
 */
int AD7124::getCurrentChannel() {
    uint32_t status;
    int ret = readRegister(static_cast<uint8_t>(Register::STATUS), &status, 1);
    if (ret < 0) {
        return ret;
    }
    
    return (int)AD7124_STATUS_REG_CH_ACTIVE(status);
}
