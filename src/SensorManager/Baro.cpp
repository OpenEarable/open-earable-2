#include "Baro.h"

#include "SensorManager.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(BMP388);

static struct sensor_msg msg_baro;

Adafruit_BMP3XX Baro::bmp;

Baro Baro::sensor;

// Initialisierung der SampleRateSettings für Baro (BMP3)
const SampleRateSetting<4> Baro::sample_rates = {
    { BMP3_ODR_25_HZ, BMP3_ODR_50_HZ, BMP3_ODR_100_HZ, BMP3_ODR_200_HZ }, // reg_vals
    { 25.0, 50.0, 100.0, 200.0 }, // sample_rates
    { 25.0, 50.0, 100.0, 200.0 }  // true_sample_rates
};

void Baro::update_sensor(struct k_work *work) {
	ARG_UNUSED(work);
	int ret;

    if (!sensor._running) return;
    const uint64_t read_start = micros();
    const int count = bmp.readFifo(sensor.samples, ARRAY_SIZE(sensor.samples));
    if (count < 0) {
        LOG_WRN("Pressure FIFO read failed");
        return;
    }
    if (count == 0) return;
    sensor.sample_clock.begin(read_start, count, sensor.sample_period_us,
                              ARRAY_SIZE(sensor.samples), CONFIG_SENSOR_CLOCK_ACCURACY);
    for (int i = 0; i < count; ++i) {
        msg_baro.sd = sensor._sd_logging;
        msg_baro.stream = sensor._ble_stream;
        msg_baro.data.id = ID_TEMP_BARO;
        msg_baro.data.size = 2 * sizeof(float);
        msg_baro.data.time = sensor.sample_clock.timestamp(i);
        const float data[2] = {static_cast<float>(sensor.samples[i].temperature),
                               static_cast<float>(sensor.samples[i].pressure)};
        memcpy(msg_baro.data.data, data, sizeof(data));
        ret = k_msgq_put(sensor_queue, &msg_baro, K_NO_WAIT);
        if (ret) {
            LOG_WRN("sensor msg queue full");
        }
    }
}

/**
* @brief Submit a k_work on timer expiry.
*/
void Baro::sensor_timer_handler(struct k_timer *dummy)
{
	ARG_UNUSED(dummy);
	k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
};

bool Baro::init(struct k_msgq * queue) {
	if (!_active) {
		pm_device_runtime_get(ls_1_8);
    	_active = true;
	}

    if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
		LOG_WRN("Could not find a valid BMP388 sensor, check wiring!");
		pm_device_runtime_put(ls_1_8);
		_active = false;
		return false;
    }

	sensor_queue = queue;
	
	k_work_init(&sensor.sensor_work, update_sensor);
	k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

	return true;
}

void Baro::start(int sample_rate_idx) {
	if (!_active) return;
	const uint8_t odr = sample_rates.reg_vals[sample_rate_idx];
	if (!bmp.startContinuous(odr)) {
		LOG_ERR("Failed to start pressure sampling");
		return;
	}

    sample_period_us =
        1000000.0 / static_cast<double>(sample_rates.true_sample_rates[sample_rate_idx]);
    sample_clock.reset();
    // Retain individual acquisition times while amortizing the bus reads.
    const k_timeout_t interval = K_USEC(MAX(20000, sample_period_us));
    _running = true;
    k_timer_start(&sensor.sensor_timer, interval, interval);
}

void Baro::stop() {
	if (!_active) return;
    _active = false;

	_running = false;

	k_timer_stop(&sensor.sensor_timer);
	struct k_work_sync sync;
	k_work_cancel_sync(&sensor.sensor_work, &sync);
	if (!bmp.stopContinuous()) {
		LOG_WRN("Failed to stop pressure sampling");
	}

    pm_device_runtime_put(ls_1_8);
}
