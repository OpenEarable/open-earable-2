#include "PPG.h"

#include "math.h"
#include "stdlib.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(MAXM86161);

PPG PPG::sensor;

MAXM86161 PPG::ppg(Wire1);

static struct sensor_msg msg_ppg;

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
    if((status == 0) && (int_status & MAXM86161_INT_DATA_RDY)) {
        int num_samples = ppg.read(sensor.data_buffer);

        for (int i = 0; i < num_samples; i++) {
            msg_ppg.sd = sensor._sd_logging;
	        msg_ppg.stream = sensor._ble_stream;
            
            msg_ppg.data.id = ID_PPG;
            msg_ppg.data.size = 4 * sizeof(uint32_t);
            msg_ppg.data.time = millis();
            msg_ppg.data.data[0]=sensor.data_buffer[i][red];
            msg_ppg.data.data[1]=sensor.data_buffer[i][green];
            msg_ppg.data.data[2]=sensor.data_buffer[i][ir];
            msg_ppg.data.data[3]=sensor.data_buffer[i][ambient];

            int ret = k_msgq_put(sensor_queue, &msg_ppg, K_NO_WAIT);
            if (ret == -EAGAIN) {
                //LOG_WRN("sensor msg queue full");
                LOG_WRN("sensor msg queue full");
            }
        }
    }
}

/**
* @brief Submit a k_work on timer expiry.
*/
void PPG::sensor_timer_handler(struct k_timer *dummy) {
	k_work_submit(&sensor.sensor_work);
}

void PPG::start(k_timeout_t t) {
    if (!_active) return;
    //ppg.setup(0x64, 1, 2, 400, 215, 8192);
    ppg.start();
	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);
}

void PPG::stop() {
    if (!_active) return;
    _active = false;

	k_timer_stop(&sensor.sensor_timer);

    ppg.stop();

    pm_device_runtime_put(ls_1_8);
    pm_device_runtime_put(ls_3_3);
}