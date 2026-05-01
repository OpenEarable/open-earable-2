#include "MicrophoneBle.h"

#include "MicBleStreamer.h"
#include "audio_datapath.h"
#include "hw_codec.h"

#include <zephyr/kernel.h>

extern "C" {
#include <data_fifo.h>
extern void init_fifo(void);
}

extern struct data_fifo fifo_rx;

MicrophoneBle MicrophoneBle::inner(ID_MICRO_INNER);
MicrophoneBle MicrophoneBle::outer(ID_MICRO_OUTER);

const SampleRateSetting<1> MicrophoneBle::sample_rates = {
	{ 0 },

	{ 1000 },

	{ 1000.0 }
};

MicrophoneBle::MicrophoneBle(uint8_t sensor_id) : _sensor_id(sensor_id)
{
}

bool MicrophoneBle::init(struct k_msgq *queue)
{
	_active = true;

	sensor_queue = queue;

	set_sensor_queue(queue);
	mic_ble_streamer_set_sensor_queue(queue);
	init_fifo();

	return true;
}

void MicrophoneBle::start(int sample_rate_idx)
{
	ARG_UNUSED(sample_rate_idx);

	if (!_active || _running) {
		return;
	}

	int ret = mic_ble_streamer_enable(_sensor_id, true);
	if (ret != 0) {
		return;
	}

	ret = audio_datapath_aquire(&fifo_rx);
	if (ret != 0) {
		mic_ble_streamer_enable(_sensor_id, false);
		return;
	}

	ret = hw_codec_default_conf_enable();
	if (ret != 0) {
		audio_datapath_release();
		mic_ble_streamer_enable(_sensor_id, false);
		return;
	}

	_running = true;
}

void MicrophoneBle::stop()
{
	if (!_active) {
		return;
	}
	_active = false;

	if (!_running) {
		return;
	}

	mic_ble_streamer_enable(_sensor_id, false);
	audio_datapath_release();

	_running = false;
}
