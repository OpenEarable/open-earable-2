#include "Temp_right_I2C2.h"

#include "SensorManager.h"

#include "math.h"
#include "stdlib.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(MLX90632);

Temp_right_I2C2 Temp_right_I2C2::sensor;

MLX90632 Temp_right_I2C2::temp;

static struct sensor_msg msg_temp_right;

const SampleRateSetting<8> Temp_right_I2C2::sample_rates = {
    { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 },  // reg_vals
    { 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0 },       // sample_rates
    { 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0 }        // true_sample_rates
};

bool Temp_right_I2C2::init(struct k_msgq * queue) {

    if (!_active) {
        pm_device_runtime_get(ls_1_8);
        pm_device_runtime_get(ls_3_3);
    	_active = true;
	}

    uint8_t deviceAddress = DT_REG_ADDR(DT_NODELABEL(mlx90632_right));
    MLX90632::status returnError;
    if (!temp.begin(deviceAddress, I2C2, returnError)) {   // hardware I2C mode on I2C2
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);

        _active = false;

		LOG_WRN("Could not find a valid Optical Temperature sensor (right), check wiring!");
		return false;
    }

	sensor_queue = queue;
	
	k_work_init(&sensor.sensor_work, update_sensor);
	k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

    _active = true;

	return true;
}

void Temp_right_I2C2::update_sensor(struct k_work *work) {
    if (!temp.dataAvailable()) return;

    MLX90632::status returnError;
    float temperature = temp.getObjectTemp(returnError);

    if (returnError != MLX90632::SENSOR_SUCCESS) {
        LOG_WRN("Error reading temperature (right)");
        return;
    }

    msg_temp_right.sd = sensor._sd_logging;
    msg_temp_right.stream = sensor._ble_stream;

    msg_temp_right.data.id = ID_OPTTEMP_right_I2C2;
    msg_temp_right.data.size = sizeof(float);
    msg_temp_right.data.time = micros();

    memcpy(msg_temp_right.data.data, &temperature, sizeof(float));

    int ret = k_msgq_put(sensor_queue, &msg_temp_right, K_NO_WAIT);
    if (ret) {
        LOG_WRN("sensor msg queue full");
    }
}

/**
* @brief Submit a k_work on timer expiry.
*/
void Temp_right_I2C2::sensor_timer_handler(struct k_timer *dummy) {
	k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
}

void Temp_right_I2C2::start(int sample_rate_idx) {
    if (!_active) return;

    k_timeout_t t = K_USEC(1e6 / sample_rates.true_sample_rates[sample_rate_idx]);

    temp.setSampleRateRegVal(sample_rates.reg_vals[sample_rate_idx]);
    temp.continuousMode();

	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);

    _running = true;
}

void Temp_right_I2C2::stop() {
    if (!_active) return;
    _active = false;

    _running = false;

	k_timer_stop(&sensor.sensor_timer);

    temp.sleepMode();

    pm_device_runtime_put(ls_1_8);
    pm_device_runtime_put(ls_3_3);
}
