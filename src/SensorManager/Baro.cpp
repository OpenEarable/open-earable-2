#include "Baro.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(BMP388);

static struct sensor_msg msg_baro;

Adafruit_BMP3XX Baro::bmp;

Baro Baro::sensor;

const sample_rate_setting Baro::sample_rates[Baro::num_sample_rates] = {
    {BMP3_ODR_0_001_HZ, 0.001},  // 0.001 Hz
    {BMP3_ODR_0_003_HZ, 0.003},  // 0.003 Hz
    {BMP3_ODR_0_006_HZ, 0.006},  // 0.006 Hz
    {BMP3_ODR_0_01_HZ,  0.01},   // 0.01 Hz
    {BMP3_ODR_0_02_HZ,  0.02},   // 0.02 Hz
    {BMP3_ODR_0_05_HZ,  0.05},   // 0.05 Hz
    {BMP3_ODR_0_1_HZ,   0.1},    // 0.1 Hz
    {BMP3_ODR_0_2_HZ,   0.2},    // 0.2 Hz
    {BMP3_ODR_0_39_HZ,  0.39},   // 0.39 Hz
    {BMP3_ODR_0_78_HZ,  0.78},   // 0.78 Hz
    {BMP3_ODR_1_5_HZ,   1.5},    // 1.5 Hz
    {BMP3_ODR_3_1_HZ,   3.1},    // 3.1 Hz
    {BMP3_ODR_6_25_HZ,  6.25},   // 6.25 Hz
    {BMP3_ODR_12_5_HZ,  12.5},   // 12.5 Hz
    {BMP3_ODR_25_HZ,    25},     // 25 Hz
    {BMP3_ODR_50_HZ,    50},     // 50 Hz
    {BMP3_ODR_100_HZ,   100},    // 100 Hz
    {BMP3_ODR_200_HZ,   200}     // 200 Hz
};

void Baro::update_sensor(struct k_work *work) {
	int ret;

	bmp.performReading();

	msg_baro.sd = sensor._sd_logging;
	msg_baro.stream = sensor._ble_stream;

	msg_baro.data.id = ID_TEMP_BARO;
	msg_baro.data.size = 2 * sizeof(float);
	msg_baro.data.time = millis();
	msg_baro.data.data[0] = bmp.temperature;
	msg_baro.data.data[1] = bmp.pressure;

	ret = k_msgq_put(sensor_queue, &msg_baro, K_NO_WAIT);
	if (ret == -EAGAIN) {
		LOG_WRN("sensor msg queue full");
	}
}

/**
* @brief Submit a k_work on timer expiry.
*/
void Baro::sensor_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&sensor.sensor_work);
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
	sample_rate_setting setting = sample_rates[sample_rate_idx];
    k_timeout_t t = K_USEC(1e6 / setting.sample_rate);
    
    //bmp.set_interrogation_rate(setting.reg_val);
    //bmp.start();

	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);
}

void Baro::stop() {
	if (!_active) return;
    _active = false;

	k_timer_stop(&sensor.sensor_timer);

    pm_device_runtime_put(ls_1_8);
}