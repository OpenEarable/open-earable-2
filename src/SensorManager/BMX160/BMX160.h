#ifndef BMX160_H
#define BMX160_H

#include <stdint.h>

#include <TWIM.h>

extern "C" {
#include "bosch/bmi160.h"
#include "bosch/bmm150.h"
}

struct BMX160Sample {
    float accel[3];
    float gyro[3];
    float mag[3];
};

class BMX160 {
public:
    static constexpr uint16_t FIFO_CAPACITY_BYTES = 1024;
    static constexpr uint8_t MAX_FIFO_SAMPLES = 48;

    explicit BMX160(TWIM *i2c);

    bool init();
    int start(uint8_t odr, float sample_rate_hz, uint8_t buffered_samples);
    int stop();
    int read(BMX160Sample *samples, uint8_t max_samples);

private:
    static BMX160 *instance;

    static int8_t bmiRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
    static int8_t bmiWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
    static void bmiDelay(uint32_t period_ms);

    static int8_t bmmRead(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
    static int8_t bmmWrite(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
    static void bmmDelay(uint32_t period_us, void *intf_ptr);

    int8_t busRead(uint8_t reg_addr, uint8_t *data, uint16_t len);
    int8_t busWrite(uint8_t reg_addr, const uint8_t *data, uint16_t len);
    int configureMagnetometer();
    static uint8_t auxOdrFor(float sample_rate_hz);

    TWIM *_i2c;
    uint8_t _addr;

    struct bmi160_dev _bmi = {};
    struct bmm150_dev _bmm = {};
    struct bmi160_fifo_frame _fifo = {};
    struct bmm150_settings _bmm_settings = {};

    uint8_t _fifo_data[FIFO_CAPACITY_BYTES] = {};
    struct bmi160_sensor_data _accel[MAX_FIFO_SAMPLES] = {};
    struct bmi160_sensor_data _gyro[MAX_FIFO_SAMPLES] = {};
    struct bmi160_aux_data _aux[MAX_FIFO_SAMPLES] = {};
};

#endif
