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
//LOG_MODULE_DECLARE(BMX160);
LOG_MODULE_REGISTER(microphone, CONFIG_LOG_DEFAULT_LEVEL);

extern struct data_fifo fifo_rx;

Microphone Microphone::sensor;

const SampleRateSetting<1> Microphone::sample_rates = {
    { 0 },

	{ 48000 },

	{ 48000.0 }
};

bool Microphone::init(struct k_msgq * queue) {

	_active = true;

	sensor_queue = queue;

	set_sensor_queue(queue);

	init_fifo();

	return true;
}

void Microphone::start(int sample_rate_idx) {
	ARG_UNUSED(sample_rate_idx);

	int ret;

	if (!_active) return;

	record_to_sd(true);

	audio_datapath_aquire(&fifo_rx);

	_running = true;
}

void Microphone::stop() {
    if (!_active) return;
    _active = false;

	if (!_running) return;

	record_to_sd(false);

	audio_datapath_release();

	_running = false;
}