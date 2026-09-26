#include "BMX160.h"

#include <algorithm>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(BMX160, CONFIG_MAIN_LOG_LEVEL);

namespace {
constexpr float EARTH_GRAVITY = 9.80665f;
constexpr float ACCEL_SCALE_2G = (2.0f * EARTH_GRAVITY) / 32768.0f;
constexpr float GYRO_SCALE_2000_DPS = 2000.0f / 32768.0f;
constexpr uint8_t FIFO_FRAME_BYTES = 21; // header + 8-byte mag + 6-byte gyro + 6-byte accel
constexpr uint8_t BMX160_CHIP_ID = 0xD8;
}

BMX160 *BMX160::instance = nullptr;

BMX160::BMX160(TWIM *i2c)
    : _i2c(i2c), _addr(DT_REG_ADDR(DT_NODELABEL(bmx160)))
{
    instance = this;
}

int8_t BMX160::bmiRead(uint8_t, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    return instance == nullptr ? BMI160_E_NULL_PTR : instance->busRead(reg_addr, data, len);
}

int8_t BMX160::bmiWrite(uint8_t, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    return instance == nullptr ? BMI160_E_NULL_PTR : instance->busWrite(reg_addr, data, len);
}

void BMX160::bmiDelay(uint32_t period_ms)
{
    k_msleep(period_ms);
}

int8_t BMX160::bmmRead(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    auto *self = static_cast<BMX160 *>(intf_ptr);
    if (self == nullptr || len > UINT16_MAX) {
        return BMM150_E_NULL_PTR;
    }
    return bmi160_aux_read(reg_addr, data, static_cast<uint16_t>(len), &self->_bmi);
}

int8_t BMX160::bmmWrite(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    auto *self = static_cast<BMX160 *>(intf_ptr);
    if (self == nullptr || len > UINT16_MAX) {
        return BMM150_E_NULL_PTR;
    }
    return bmi160_aux_write(reg_addr, const_cast<uint8_t *>(data), static_cast<uint16_t>(len), &self->_bmi);
}

void BMX160::bmmDelay(uint32_t period_us, void *)
{
    k_usleep(period_us);
}

int8_t BMX160::busRead(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    _i2c->aquire();
    const int ret = i2c_burst_read(_i2c->master, _addr, reg_addr, data, len);
    _i2c->release();

    return ret == 0 ? BMI160_OK : BMI160_E_COM_FAIL;
}

int8_t BMX160::busWrite(uint8_t reg_addr, const uint8_t *data, uint16_t len)
{
    _i2c->aquire();
    const int ret = i2c_burst_write(_i2c->master, _addr, reg_addr, data, len);
    _i2c->release();

    return ret == 0 ? BMI160_OK : BMI160_E_COM_FAIL;
}

bool BMX160::init()
{
    _i2c->begin();

    _bmi = {};
    _bmi.id = _addr;
    _bmi.intf = BMI160_I2C_INTF;
    _bmi.read = bmiRead;
    _bmi.write = bmiWrite;
    _bmi.delay_ms = bmiDelay;
    _bmi.read_write_len = FIFO_CAPACITY_BYTES;

    int8_t result = bmi160_init(&_bmi);
    // The BMX160's integrated BMI160-compatible die reports 0xD8 instead of
    // the standalone BMI160's 0xD1 expected by Bosch's unmodified API.
    if (result == BMI160_E_DEV_NOT_FOUND && _bmi.chip_id == BMX160_CHIP_ID) {
        LOG_INF("BMX160 chip ID 0x%02x accepted for Bosch BMI160 API", _bmi.chip_id);
        result = bmi160_soft_reset(&_bmi);
    }
    if (result != BMI160_OK) {
        LOG_ERR("Bosch BMI160 init failed: %d (chip ID 0x%02x)", result, _bmi.chip_id);
        return false;
    }

    _bmi.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    _bmi.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    _bmi.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    _bmi.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
    _bmi.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
    _bmi.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    _bmi.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    _bmi.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    result = bmi160_set_sens_conf(&_bmi);
    if (result != BMI160_OK) {
        LOG_ERR("BMI160 sensor configuration failed: %d", result);
        return false;
    }

    if (configureMagnetometer() != 0) {
        return false;
    }

    _fifo.data = _fifo_data;
    _bmi.fifo = &_fifo;

    result = bmi160_set_fifo_config(BMI160_FIFO_HEADER | BMI160_FIFO_AUX |
                                    BMI160_FIFO_GYRO | BMI160_FIFO_ACCEL,
                                    BMI160_ENABLE, &_bmi);
    if (result != BMI160_OK) {
        LOG_ERR("BMI160 FIFO configuration failed: %d", result);
        return false;
    }

    result = bmi160_set_fifo_flush(&_bmi);
    if (result != BMI160_OK) {
        LOG_ERR("BMI160 FIFO flush failed: %d", result);
        return false;
    }

    LOG_INF("Bosch BMI160/BMM150 driver initialized with MGA FIFO");
    return true;
}

int BMX160::configureMagnetometer()
{
    _bmi.aux_cfg.aux_sensor_enable = BMI160_ENABLE;
    _bmi.aux_cfg.aux_i2c_addr = BMM150_DEFAULT_I2C_ADDRESS;
    _bmi.aux_cfg.manual_enable = BMI160_ENABLE;
    _bmi.aux_cfg.aux_rd_burst_len = BMI160_AUX_READ_LEN_3;
    _bmi.aux_cfg.aux_odr = BMI160_AUX_ODR_100HZ;

    int8_t result = bmi160_aux_init(&_bmi);
    if (result != BMI160_OK) {
        LOG_ERR("BMI160 auxiliary interface init failed: %d", result);
        return result;
    }

    _bmm = {};
    _bmm.intf = BMM150_I2C_INTF;
    _bmm.intf_ptr = this;
    _bmm.read = bmmRead;
    _bmm.write = bmmWrite;
    _bmm.delay_us = bmmDelay;

    result = bmm150_init(&_bmm);
    if (result != BMM150_OK) {
        LOG_ERR("Bosch BMM150 init through BMI160 AUX failed: %d", result);
        return result;
    }

    _bmm_settings = {};
    _bmm_settings.preset_mode = BMM150_PRESETMODE_REGULAR;
    result = bmm150_set_presetmode(&_bmm_settings, &_bmm);
    if (result != BMM150_OK) {
        LOG_ERR("BMM150 regular preset failed: %d", result);
        return result;
    }

    // This must be the final BMM150 write before auto mode. The BMI160 repeats
    // it after each AUX read, triggering the next single measurement.
    _bmm_settings.pwr_mode = BMM150_POWERMODE_FORCED;
    result = bmm150_set_op_mode(&_bmm_settings, &_bmm);
    if (result != BMM150_OK) {
        LOG_ERR("BMM150 forced mode configuration failed: %d", result);
        return result;
    }

    uint8_t data_start = BMM150_REG_DATA_X_LSB;
    result = bmi160_set_aux_auto_mode(&data_start, &_bmi);
    if (result != BMI160_OK) {
        LOG_ERR("BMI160 AUX auto mode failed: %d", result);
    }
    return result;
}

uint8_t BMX160::auxOdrFor(float sample_rate_hz)
{
    if (sample_rate_hz <= 25.0f) return BMI160_AUX_ODR_25HZ;
    if (sample_rate_hz <= 50.0f) return BMI160_AUX_ODR_50HZ;
    return BMI160_AUX_ODR_100HZ;
}

int BMX160::start(uint8_t odr, float sample_rate_hz, uint8_t buffered_samples)
{
    _bmi.accel_cfg.odr = odr;
    _bmi.gyro_cfg.odr = odr;
    _bmi.aux_cfg.aux_odr = auxOdrFor(sample_rate_hz);

    int8_t result = bmi160_set_sens_conf(&_bmi);
    if (result == BMI160_OK) {
        uint8_t data_start = BMM150_REG_DATA_X_LSB;
        result = bmi160_set_aux_auto_mode(&data_start, &_bmi);
    }
    if (result == BMI160_OK) {
        const uint16_t watermark_bytes = std::min<uint16_t>(
            FIFO_CAPACITY_BYTES - FIFO_FRAME_BYTES,
            static_cast<uint16_t>(buffered_samples) * FIFO_FRAME_BYTES);
        result = bmi160_set_fifo_wm(static_cast<uint8_t>((watermark_bytes + 3U) / 4U), &_bmi);
    }
    if (result == BMI160_OK) {
        result = bmi160_set_fifo_flush(&_bmi);
    }
    return result;
}

int BMX160::stop()
{
    int8_t result = bmi160_set_fifo_flush(&_bmi);
    _bmi.accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
    _bmi.gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;
    const int8_t power_result = bmi160_set_power_mode(&_bmi);
    return result == BMI160_OK ? power_result : result;
}

int BMX160::read(BMX160Sample *samples, uint8_t max_samples)
{
    if (samples == nullptr || max_samples == 0) {
        return BMI160_E_NULL_PTR;
    }

    _fifo.length = sizeof(_fifo_data);
    int8_t result = bmi160_get_fifo_data(&_bmi);
    if (result != BMI160_OK) {
        return result;
    }

    uint8_t accel_count = std::min<uint8_t>(max_samples, MAX_FIFO_SAMPLES);
    uint8_t gyro_count = accel_count;
    uint8_t aux_count = accel_count;

    result = bmi160_extract_accel(_accel, &accel_count, &_bmi);
    if (result == BMI160_OK) result = bmi160_extract_gyro(_gyro, &gyro_count, &_bmi);
    if (result == BMI160_OK) result = bmi160_extract_aux(_aux, &aux_count, &_bmi);
    if (result != BMI160_OK) {
        return result;
    }

    const uint8_t sample_count = std::min(accel_count, gyro_count);
    if (sample_count == 0 || aux_count == 0) {
        return 0;
    }

    for (uint8_t i = 0; i < sample_count; ++i) {
        samples[i].accel[0] = _accel[i].x * ACCEL_SCALE_2G;
        samples[i].accel[1] = _accel[i].y * ACCEL_SCALE_2G;
        samples[i].accel[2] = _accel[i].z * ACCEL_SCALE_2G;
        samples[i].gyro[0] = _gyro[i].x * GYRO_SCALE_2000_DPS;
        samples[i].gyro[1] = _gyro[i].y * GYRO_SCALE_2000_DPS;
        samples[i].gyro[2] = _gyro[i].z * GYRO_SCALE_2000_DPS;

        const uint8_t mag_index = sample_count <= aux_count
                                      ? i
                                      : std::min<uint8_t>(aux_count - 1,
                                            static_cast<uint16_t>(i) * aux_count / sample_count);
        struct bmm150_mag_data mag = {};
        result = bmm150_aux_mag_data(_aux[mag_index].data, &mag, &_bmm);
        if (result != BMM150_OK) {
            return result;
        }
        samples[i].mag[0] = mag.x;
        samples[i].mag[1] = mag.y;
        samples[i].mag[2] = mag.z;
    }

    return sample_count;
}
