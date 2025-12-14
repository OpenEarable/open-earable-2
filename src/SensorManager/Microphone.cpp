#include "Microphone.h"

#include "audio_datapath.h"
#include "streamctrl.h"

#include "SensorManager.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/device.h>

#include "ADAU1860.h"

//#include <data_fifo.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <data_fifo.h>
extern void init_fifo();
extern void empty_fifo();

#ifdef __cplusplus
}
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(microphone, CONFIG_LOG_DEFAULT_LEVEL);

extern struct data_fifo fifo_rx;

Microphone Microphone::sensor;

const SampleRateSetting<9> Microphone::sample_rates = {
    { 1, 2, 3, 4, 6, 8, 12, 16, 24 },

	{ 48000, 24000, 16000, 12000, 8000, 6000, 4000, 3000, 2000 },

	{ 48000.0, 24000.0, 16000.0, 12000.0, 8000.0, 6000.0, 4000.0, 3000.0, 2000.0 }
};

bool Microphone::init(struct k_msgq * queue) {

	_active = true;

	sensor_queue = queue;

	set_sensor_queue(queue);

	init_fifo();

	return true;
}

void Microphone::start(int sample_rate_idx) {
	//ARG_UNUSED(sample_rate_idx);

	int ret;

	if (!_active) return;

	//record_to_sd(true);

	LOG_INF("Starting Microphone at %d Hz", sample_rates.sample_rates[sample_rate_idx]);

	audio_datapath_decimator_init(sample_rates.reg_vals[sample_rate_idx]);

	audio_datapath_aquire(&fifo_rx);

	_running = true;
}

void Microphone::stop() {
    if (!_active) return;
    _active = false;

	if (!_running) return;

	//record_to_sd(false);

	audio_datapath_release();

	audio_datapath_decimator_cleanup();

	_running = false;
}

void Microphone::record(bool active) {
	record_to_sd(active);
}

// C wrapper functions for C code
extern "C" {
    void microphone_start(int sample_rate_idx) {
        Microphone::sensor.start(sample_rate_idx);
    }
    
    void microphone_stop(void) {
        Microphone::sensor.stop();
    }
}