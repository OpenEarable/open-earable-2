#include "PPG.h"

#include "SensorManager.h"

#include "math.h"
#include "stdlib.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(MAXM86161);

#define LATENCY_MS 40

PPG PPG::sensor;

MAXM86161 PPG::ppg(&I2C2);

static struct sensor_msg msg_ppg;

const SampleRateSetting<16> PPG::sample_rates = {
    { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x0A, 0x0B,
    0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13 },

    { 25, 50, 84, 100, 200, 400, 8, 16,
    32, 64, 128, 256, 512, 1024, 2048, 4096 },

    { 25, 50, 84, 100, 200, 400, 8, 16,
    32, 64, 128, 256, 512, 1024, 2048, 4096 },
};

bool PPG::init(struct k_msgq * queue) {
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

void PPG::reset() {
    // Reset pulse oximeter state
}

void PPG::update_sensor(struct k_work *work) {
    int int_status;
    int status = ppg.read_interrupt_state(int_status);
    
    if (status != 0) {
        LOG_ERR("PPG read interrupt state failed: %d", status);
        return;
    }
    
    if((status == 0) && ((int_status & MAXM86161_INT_FULL))) { // MAXM86161_INT_DATA_RDY
        int num_samples = ppg.read(sensor.data_buffer);

        uint64_t time_stamp = micros();

        for (int i = 0; i < num_samples; i++) {
            msg_ppg.sd = sensor._sd_logging;
	        msg_ppg.stream = sensor._ble_stream;

            size_t size = sizeof(uint32_t);
            
            msg_ppg.data.id = ID_PPG;
            msg_ppg.data.size = 4 * size;
            msg_ppg.data.time = time_stamp - (num_samples - i) * PPG::sensor.t_sample_us;

            memcpy(msg_ppg.data.data + 0 * size, &sensor.data_buffer[i][red], size);
            memcpy(msg_ppg.data.data + 1 * size, &sensor.data_buffer[i][ir], size);
            memcpy(msg_ppg.data.data + 2 * size, &sensor.data_buffer[i][green], size);
            memcpy(msg_ppg.data.data + 3 * size, &sensor.data_buffer[i][ambient], size);

            int ret = k_msgq_put(sensor_queue, &msg_ppg, K_NO_WAIT);
            if (ret == -EAGAIN) {
                LOG_WRN("sensor msg queue full");
            }
        }
    }
}

/**
* @brief Submit a k_work on timer expiry.
*/
void PPG::sensor_timer_handler(struct k_timer *dummy) {
	k_work_submit_to_queue(&sensor_work_q, &sensor.sensor_work);
}

void PPG::start(int sample_rate_idx) {
    if (!_active) return;

    t_sample_us = 1e6 / sample_rates.true_sample_rates[sample_rate_idx];

    k_timeout_t t = K_USEC(t_sample_us);
    
    ppg.set_interrogation_rate(sample_rates.reg_vals[sample_rate_idx]);
    ppg.set_watermark(MAX(FIFO_SIZE - LED_NUM * MAX(1, (int) (LATENCY_MS * 1e3 / t_sample_us)), 0xF));
    ppg.start();

    _running = true;

	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);
}

void PPG::stop() {
    if (!_active) return;
    _active = false;

    _running = false;

	k_timer_stop(&sensor.sensor_timer);

    ppg.stop();

    pm_device_runtime_put(ls_1_8);
    pm_device_runtime_put(ls_3_3);
}