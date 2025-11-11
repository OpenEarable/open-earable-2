#include "PPG_left_I2C1.h"

#include "SensorManager.h"

#include "math.h"
#include "stdlib.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(MAXM86161);

#define LATENCY_MS 40

PPG_left_I2C1 PPG_left_I2C1::sensor;

MAXM86161 PPG_left_I2C1::ppg(&I2C1);

static struct sensor_msg msg_ppg_left;

const SampleRateSetting<16> PPG_left_I2C1::sample_rates = {
    { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0A, 0x0B,
    0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13 },

    { 25, 50, 84, 100, 200, 400, 8, 16,
    32, 64, 128, 256, 512, 1024, 2048, 4096 },

    { 24.995, 50.027, 84.021, 99.902, 199.805, 399.610, 8.000, 16.000,
    32.000, 64.000, 128.000, 256.000, 512.000, 1024.000, 2048.000, 4096.000},
};

bool PPG_left_I2C1::init(struct k_msgq * queue) {
    if (!_active) {
        pm_device_runtime_get(ls_1_8);
        pm_device_runtime_get(ls_3_3);

        const struct gpio_dt_spec LDO_EN = {
            .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
            .pin = 6,
            .dt_flags = GPIO_ACTIVE_HIGH
        };

        int ret = gpio_pin_configure_dt(&LDO_EN, GPIO_OUTPUT_ACTIVE);
        if (ret != 0) {
            LOG_WRN("Failed to set GPOUT as input.\n");
            return false;
        }

        k_msleep(5);

    	_active = true;
	}
    
    if (ppg.init() != 0) {   // hardware I2C mode, can pass in address & alt Wire
		LOG_WRN("Could not find a valid PPG sensor, check wiring!");
        _active = false;
        pm_device_runtime_put(ls_1_8);
        pm_device_runtime_put(ls_3_3);
		return false;
    }

	sensor_queue = queue;
	
	k_work_init(&sensor.sensor_work, update_sensor);
	k_timer_init(&sensor.sensor_timer, sensor_timer_handler, NULL);

	return true;
}

void PPG_left_I2C1::update_sensor(struct k_work *work) {
    int int_status;
    int status;

    uint64_t _time_stamp = micros();

    PPG_left_I2C1::sensor._sample_count += (_time_stamp - PPG_left_I2C1::sensor._last_time_stamp) / PPG_left_I2C1::sensor.t_sample_us;
    PPG_left_I2C1::sensor._last_time_stamp = _time_stamp;

    if (PPG_left_I2C1::sensor._sample_count < PPG_left_I2C1::sensor._num_samples_buffered * (1.f - CONFIG_SENSOR_CLOCK_ACCURACY / 100.f)) {
        return;
    }
    
    status = ppg.read_interrupt_state(int_status);
    
    if (status != 0) {
        LOG_ERR("PPG read interrupt state failed: %d", status);
        return;
    }
    
    if(int_status & MAXM86161_INT_FULL) { // MAXM86161_INT_DATA_RDY
        int num_samples = ppg.read(sensor.data_buffer);

        PPG_left_I2C1::sensor._sample_count = MAX(0, PPG_left_I2C1::sensor._num_samples_buffered - num_samples);

        for (int i = 0; i < num_samples; i++) {
            msg_ppg_left.sd = sensor._sd_logging;
	        msg_ppg_left.stream = sensor._ble_stream;

            size_t size = sizeof(uint32_t);
            
            msg_ppg_left.data.id = ID_PPG_left_I2C1;
            msg_ppg_left.data.size = 4 * size;
            msg_ppg_left.data.time = _time_stamp - (num_samples - i) * PPG_left_I2C1::sensor.t_sample_us;

            memcpy(msg_ppg_left.data.data + 0 * size, &sensor.data_buffer[i][red_left], size);
            memcpy(msg_ppg_left.data.data + 1 * size, &sensor.data_buffer[i][ir_left], size);
            memcpy(msg_ppg_left.data.data + 2 * size, &sensor.data_buffer[i][green_left], size);
            memcpy(msg_ppg_left.data.data + 3 * size, &sensor.data_buffer[i][ambient_left], size);

            int ret = k_msgq_put(sensor_queue, &msg_ppg_left, K_NO_WAIT);
            if (ret) {
                LOG_WRN("sensor msg queue full");
            }
        }
    }
}

/**
* @brief Submit a k_work on timer expiry.
*/
void PPG_left_I2C1::sensor_timer_handler(struct k_timer *dummy) {
	k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
}

void PPG_left_I2C1::start(int sample_rate_idx) {
    if (!_active) return;

    t_sample_us = 1e6 / sample_rates.true_sample_rates[sample_rate_idx];

    k_timeout_t t = K_USEC(t_sample_us);

    _num_samples_buffered = MIN(MAX(1, (int) (CONFIG_SENSOR_LATENCY_MS * 1e3 / t_sample_us)), FIFO_SIZE / LED_NUM - 2);
    
    ppg.set_interrogation_rate(sample_rates.reg_vals[sample_rate_idx]);
    ppg.set_watermark(FIFO_SIZE - _num_samples_buffered * LED_NUM);
    ppg.start();

    k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);

    _running = true;
    _sample_count = 0;
    _last_time_stamp = micros();
}

void PPG_left_I2C1::stop() {
    if (!_active) return;
    _active = false;

    _running = false;

	k_timer_stop(&sensor.sensor_timer);

    ppg.stop();

    pm_device_runtime_put(ls_1_8);
    pm_device_runtime_put(ls_3_3);
}
