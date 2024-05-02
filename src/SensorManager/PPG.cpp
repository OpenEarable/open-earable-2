#include "PPG.h"

#include "math.h"
#include "stdlib.h"

PPG PPG::sensor;

MAX30105 PPG::ppg;

static struct sensor_data msg_ppg;

bool PPG::init(struct k_msgq * queue) {
    if (!ppg.begin(Wire1)) {   // hardware I2C mode, can pass in address & alt Wire
		printk("Could not find a valid PPG sensor, check wiring!");
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
    //ppg.check();
    if(ppg.check() > 0) {
        while(ppg.available()) {
            //printk(" %i, %i\n", sensor.ppg.getFIFORed(), sensor.ppg.getFIFOIR());
            msg_ppg.id = ID_PPG;
            msg_ppg.size = 2 * sizeof(uint32_t);
            msg_ppg.time = k_cyc_to_ms_floor32(k_cycle_get_32());
            msg_ppg.data[0]=ppg.getFIFORed();
            msg_ppg.data[1]=ppg.getFIFOIR();

            int ret = k_msgq_put(sensor_queue, (void *)&msg_ppg, K_NO_WAIT);
            if (ret == -EAGAIN) {
                //LOG_WRN("sensor msg queue full");
                printk("sensor msg queue full");
            }
            ppg.nextSample();
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
    ppg.setup(0x64, 1, 2, 400, 215, 8192);
	k_timer_start(&sensor.sensor_timer, K_NO_WAIT, t);
}

void PPG::stop() {
	k_timer_stop(&sensor.sensor_timer);
}