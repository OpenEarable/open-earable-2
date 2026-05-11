#include "IMU.h"

#include "SensorManager.h"
#include "sensor_sink.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(BMX160);

static struct sensor_msg msg_imu;
static uint32_t imu_queue_full_count;

static struct {
    uint32_t timer_fires;
    uint32_t reads;
    uint32_t total_samples;
    uint32_t max_samples;
    uint32_t exec_max_us;
    uint64_t exec_total_us;
} imu_stats;

BMX160 IMU::imu;
IMU IMU::sensor;
BMX160Sample IMU::_batch[64];

/* Register values match BMI160's ACCEL_ODR_* encoding used for both the
 * accel and gyro (same layout). Legacy IMU.cpp used BMX160_GYRO_ODR_*
 * macros, which had identical numeric values. */
const SampleRateSetting<6> IMU::sample_rates = {
    { BMI160_GYRO_ODR_25HZ, BMI160_GYRO_ODR_50HZ, BMI160_GYRO_ODR_100HZ,
      BMI160_GYRO_ODR_200HZ, BMI160_GYRO_ODR_400HZ, BMI160_GYRO_ODR_800HZ },

    { 25, 50, 100, 200, 400, 800 },

    { 25.0, 50.0, 100.0, 200.0, 400.0, 800.0 }
};

/* Fixed ceiling on frames drained per tick. One FIFO full of A+G headered
 * frames (13 bytes each) is ~78 frames; we reserve some stack-safe margin. */
#define IMU_MAX_SAMPLES_PER_READ  64

void IMU::update_sensor(struct k_work *work)
{
    if (!sensor._running) return;
    uint64_t t0 = micros();
    imu_stats.timer_fires++;

    int n = imu.read(_batch, IMU_MAX_SAMPLES_PER_READ);

    if (n <= 0) {
        uint32_t elapsed = (uint32_t)(micros() - t0);
        imu_stats.exec_total_us += elapsed;
        if (elapsed > imu_stats.exec_max_us) imu_stats.exec_max_us = elapsed;
        return;
    }

    imu_stats.reads++;
    imu_stats.total_samples += n;
    if ((uint32_t)n > imu_stats.max_samples) imu_stats.max_samples = n;

    /* Preserve the legacy per-sample message format: 36 bytes of
     * (accel[3], gyro[3], mag[3]) floats, one message per IMU sample.
     * Timestamps are spaced backwards from t0 by sensor._t_sample_us so
     * downstream consumers see monotonic sample cadence even when we drain
     * multiple frames in one tick. */
    const size_t axis_sz = 3 * sizeof(float);
    for (int i = 0; i < n; i++) {
        msg_imu.sd = sensor._sd_logging;
        msg_imu.stream = sensor._ble_stream;

        msg_imu.data.id = ID_IMU;
        msg_imu.data.size = 3 * axis_sz;

        uint64_t dt_us = (uint64_t)((n - 1 - i)) * sensor.t_sample_us;
        msg_imu.data.time = t0 - dt_us;

        memcpy(msg_imu.data.data, _batch[i].accel, axis_sz);
        memcpy(msg_imu.data.data + axis_sz, _batch[i].gyro, axis_sz);
        memcpy(msg_imu.data.data + 2 * axis_sz, _batch[i].mag, axis_sz);

        int ret = sensor_sink_put(&msg_imu);
        if (ret) imu_queue_full_count++;
    }

    uint32_t elapsed = (uint32_t)(micros() - t0);
    imu_stats.exec_total_us += elapsed;
    if (elapsed > imu_stats.exec_max_us) imu_stats.exec_max_us = elapsed;
}

void IMU::sensor_timer_handler(struct k_timer *dummy)
{
    k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
}

bool IMU::init()
{
    if (!_active) {
        pm_device_runtime_get(ls_1_8);
        _active = true;
    }

    /* A placeholder ODR/watermark — real config happens in start(). We just
     * need the chip alive here so begin() succeeds and we can report a valid
     * init. */
    if (imu.init(BMI160_ACCEL_ODR_100HZ, BMI160_GYRO_ODR_100HZ,
                 4 * sizeof(int16_t) * 3) != 0) {
        LOG_ERR("Could not find a valid BMX160 sensor, check wiring!");
        pm_device_runtime_put(ls_1_8);
        _active = false;
        return false;
    }

    k_work_init(&sensor.sensor_work, update_sensor);
    k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

    return true;
}

void IMU::start(int sample_rate_idx)
{
    if (!_active) return;

    t_sample_us = (uint32_t)(1e6 / sample_rates.true_sample_rates[sample_rate_idx]);

    /* Size the FIFO watermark so the timer fires ~every
     * CONFIG_SENSOR_LATENCY_MS worth of samples. A headered A+G frame
     * is 13 bytes (1 header + 6 accel + 6 gyro). The BMI160 FIFO is
     * 1024 bytes, so keep the buffered count within 1024/13 frames with
     * a small safety margin. */
    const int frame_bytes = 1 + 12;
    const int max_frames_in_fifo = 1024 / frame_bytes - 4;
    _num_samples_buffered = MIN(MAX(1, (int)(CONFIG_SENSOR_LATENCY_MS * 1e3 / t_sample_us)),
                                MIN(max_frames_in_fifo, IMU_MAX_SAMPLES_PER_READ));

    uint8_t odr_reg = sample_rates.reg_vals[sample_rate_idx];
    /* Re-init both ODRs (same reg value) and FIFO watermark in bytes. */
    imu.init(odr_reg, odr_reg, _num_samples_buffered * frame_bytes);
    imu.start();

    /* Fire at FIFO-drain rate. */
    k_timeout_t t = K_USEC(t_sample_us * _num_samples_buffered);
    k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);

    _running = true;
}

void IMU::stop()
{
    if (!_active) return;
    _active = false;
    _running = false;

    uint32_t avg = imu_stats.reads ? (uint32_t)(imu_stats.exec_total_us / imu_stats.reads) : 0;
    uint32_t samples_avg = imu_stats.reads ? imu_stats.total_samples / imu_stats.reads : 0;
    LOG_INF("imu: timer=%u reads=%u queue_drops=%u",
            imu_stats.timer_fires, imu_stats.reads, imu_queue_full_count);
    LOG_INF("imu: total_samples=%u avg=%u max=%u exec_avg=%u us exec_max=%u us",
            imu_stats.total_samples, samples_avg, imu_stats.max_samples,
            avg, imu_stats.exec_max_us);
    memset(&imu_stats, 0, sizeof(imu_stats));
    imu_queue_full_count = 0;

    k_timer_stop(&sensor.sensor_timer);
    k_work_cancel(&sensor.sensor_work);

    imu.stop();

    pm_device_runtime_put(ls_1_8);
}
