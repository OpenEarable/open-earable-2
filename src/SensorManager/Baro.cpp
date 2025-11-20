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

static int baro_initial_discard = 1;

// Initialisierung der SampleRateSettings für Baro (BMP3)
const SampleRateSetting<18> Baro::sample_rates = {
    { BMP3_ODR_0_001_HZ, BMP3_ODR_0_003_HZ, BMP3_ODR_0_006_HZ, BMP3_ODR_0_01_HZ, 
      BMP3_ODR_0_02_HZ, BMP3_ODR_0_05_HZ, BMP3_ODR_0_1_HZ, BMP3_ODR_0_2_HZ, 
      BMP3_ODR_0_39_HZ, BMP3_ODR_0_78_HZ, BMP3_ODR_1_5_HZ, BMP3_ODR_3_1_HZ, 
      BMP3_ODR_6_25_HZ, BMP3_ODR_12_5_HZ, BMP3_ODR_25_HZ, BMP3_ODR_50_HZ, 
      BMP3_ODR_100_HZ, BMP3_ODR_200_HZ },   // reg_vals

    { 0.001, 0.003, 0.006, 0.01, 0.02, 0.05, 0.1, 0.2, 
      0.39, 0.78, 1.5, 3.1, 6.25, 12.5, 25.0, 50.0, 
      100.0, 200.0 },  // sample_rates

    { 0.001, 0.003, 0.006, 0.01, 0.02, 0.05, 0.1, 0.2, 
      0.39, 0.78, 1.5, 3.1, 6.25, 12.5, 25.0, 50.0, 
      100.0, 200.0 }   // true_sample_rates
};

void Baro::update_sensor(struct k_work *work) {
	int ret;

	bmp.performReading();

	if (baro_initial_discard > 0) {
		baro_initial_discard--;
		return;
	}

	msg_baro.sd = sensor._sd_logging;
	msg_baro.stream = sensor._ble_stream;

	msg_baro.data.id = ID_TEMP_BARO;
	msg_baro.data.size = 2 * sizeof(float);
	msg_baro.data.time = micros();

	float data[2] = {bmp.temperature, bmp.pressure};

	memcpy(msg_baro.data.data, data, 2 * sizeof(float));

	ret = k_msgq_put(sensor_queue, &msg_baro, K_NO_WAIT);
	if (ret) {
		LOG_WRN("sensor msg queue full");
	}
}

/**
* @brief Submit a k_work on timer expiry.
*/
void Baro::sensor_timer_handler(struct k_timer *dummy)
{
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
	baro_initial_discard = 1;

    k_timeout_t t = K_USEC(1e6 / sample_rates.true_sample_rates[sample_rate_idx]);
    
    //bmp.set_interrogation_rate(setting.reg_val);
    //bmp.start();

	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);

	_running = true;
}

void Baro::stop() {
	if (!_active) return;
    _active = false;

	_running = false;

	k_timer_stop(&sensor.sensor_timer);

    pm_device_runtime_put(ls_1_8);
}