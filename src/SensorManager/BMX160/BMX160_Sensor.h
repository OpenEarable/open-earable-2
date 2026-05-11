#ifndef _BMX160_SENSOR_H
#define _BMX160_SENSOR_H

#include <stdint.h>
#include "bmi160.h"
#include <TWIM.h>

#define BMX160_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(1024)
#define BMX160_FIFO_MAX_FRAMES            UINT16_C(128)

struct BMX160Sample {
    float accel[3];
    float gyro[3];
    float mag[3];
};

class BMX160 {
public:
    int init(uint8_t accel_odr_reg, uint8_t gyro_odr_reg, uint16_t fifo_watermark_bytes);
    int start();
    int stop();
    int read(BMX160Sample *out, int max_samples);

private:
    struct bmi160_dev dev;
    struct bmi160_fifo_frame fifo_frame;
    uint8_t fifo_buf[BMX160_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

    struct bmi160_sensor_data accel_frames[BMX160_FIFO_MAX_FRAMES];
    struct bmi160_sensor_data gyro_frames[BMX160_FIFO_MAX_FRAMES];

    float _accel_lsb_to_ms2;
    float _gyro_lsb_to_dps;

    int8_t configure_magnetometer();
    int read_mag_latched(float out_mag[3]);
};

#endif
