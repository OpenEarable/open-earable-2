#include "MicBleStreamer.h"

#include "audio_i2s.h"
#include "openearable_common.h"

#include <dsp/filtering_functions.h>
#include <math.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(mic_ble_streamer, CONFIG_LOG_DEFAULT_LEVEL);

BUILD_ASSERT(CONFIG_AUDIO_SAMPLE_RATE_HZ == 48000,
	     "BLE microphone streamer expects 48 kHz I2S input");
BUILD_ASSERT(CONFIG_AUDIO_BIT_DEPTH_BITS == 16,
	     "BLE microphone streamer expects 16-bit PCM input");
BUILD_ASSERT(CONFIG_I2S_CH_NUM == 2,
	     "BLE microphone streamer expects stereo I2S input");

static constexpr uint32_t MIC_BLE_SAMPLE_RATE_HZ = 1000;
static constexpr uint32_t MIC_BLE_DECIMATION = CONFIG_AUDIO_SAMPLE_RATE_HZ / MIC_BLE_SAMPLE_RATE_HZ;
static constexpr uint32_t MIC_BLE_SAMPLE_PERIOD_US = 1000000 / MIC_BLE_SAMPLE_RATE_HZ;
static constexpr uint32_t MIC_BLE_LOW_PASS_HZ = 460;
static constexpr uint32_t MIC_BLE_FILTER_TAPS = 240;
static constexpr uint8_t MIC_BLE_SAMPLES_PER_PACKET =
	(SENSOR_DATA_FIXED_LENGTH - sizeof(uint16_t)) / sizeof(int16_t);
static constexpr uint64_t MIC_BLE_FILTER_DELAY_US =
	(((uint64_t)MIC_BLE_FILTER_TAPS - 1) * 1000000ULL) / (2ULL * CONFIG_AUDIO_SAMPLE_RATE_HZ);

BUILD_ASSERT(CONFIG_AUDIO_SAMPLE_RATE_HZ % MIC_BLE_SAMPLE_RATE_HZ == 0,
	     "BLE microphone sample rate must divide the audio sample rate");
BUILD_ASSERT(MIC_BLE_FILTER_TAPS % MIC_BLE_DECIMATION == 0,
	     "CMSIS FIR decimator requires taps to be a multiple of the decimation factor");
BUILD_ASSERT(BLOCK_SIZE_BYTES ==
	     (MIC_BLE_DECIMATION * CONFIG_I2S_CH_NUM * CONFIG_AUDIO_BIT_DEPTH_OCTETS),
	     "BLE microphone streamer expects one 1 ms I2S block per decimated sample");

enum mic_ble_channel {
	MIC_BLE_CHANNEL_INNER = 0,
	MIC_BLE_CHANNEL_OUTER = 1,
	MIC_BLE_CHANNEL_COUNT,
};

struct mic_ble_channel_state {
	bool enabled;
	bool decimator_ready;
	uint8_t sensor_id;
	uint8_t i2s_channel;
	arm_fir_decimate_instance_q15 decimator;
	q15_t filter_state[MIC_BLE_FILTER_TAPS + MIC_BLE_DECIMATION - 1];
	q15_t input[MIC_BLE_DECIMATION];
	int16_t packet[MIC_BLE_SAMPLES_PER_PACKET];
	uint8_t packet_count;
	uint64_t packet_time_us;
};

static K_MUTEX_DEFINE(mic_ble_lock);

static struct k_msgq *sensor_queue;
static bool filter_coeffs_ready;
static q15_t filter_coeffs[MIC_BLE_FILTER_TAPS];
static double filter_coeffs_float[MIC_BLE_FILTER_TAPS];

static struct mic_ble_channel_state channels[MIC_BLE_CHANNEL_COUNT] = {
	{
		.sensor_id = ID_MICRO_INNER,
		.i2s_channel = 1,
	},
	{
		.sensor_id = ID_MICRO_OUTER,
		.i2s_channel = 0,
	},
};

static void filter_coeffs_init(void)
{
	if (filter_coeffs_ready) {
		return;
	}

	const double pi = 3.14159265358979323846;
	const double normalized_cutoff = (double)MIC_BLE_LOW_PASS_HZ / (double)CONFIG_AUDIO_SAMPLE_RATE_HZ;
	const double center = ((double)MIC_BLE_FILTER_TAPS - 1.0) * 0.5;
	double sum = 0.0;

	for (uint32_t i = 0; i < MIC_BLE_FILTER_TAPS; i++) {
		const double x = (double)i - center;
		double sinc;

		if (fabs(x) < 1e-12) {
			sinc = 2.0 * normalized_cutoff;
		} else {
			sinc = sin(2.0 * pi * normalized_cutoff * x) / (pi * x);
		}

		const double window = 0.54 - (0.46 * cos((2.0 * pi * (double)i) /
							    ((double)MIC_BLE_FILTER_TAPS - 1.0)));
		filter_coeffs_float[i] = sinc * window;
		sum += filter_coeffs_float[i];
	}

	for (uint32_t i = 0; i < MIC_BLE_FILTER_TAPS; i++) {
		int32_t q15 = (int32_t)lrint((filter_coeffs_float[i] / sum) * 32767.0);

		if (q15 > INT16_MAX) {
			q15 = INT16_MAX;
		} else if (q15 < INT16_MIN) {
			q15 = INT16_MIN;
		}

		filter_coeffs[MIC_BLE_FILTER_TAPS - 1U - i] = (q15_t)q15;
	}

	filter_coeffs_ready = true;
}

static struct mic_ble_channel_state *channel_for_sensor(uint8_t sensor_id)
{
	for (uint8_t i = 0; i < MIC_BLE_CHANNEL_COUNT; i++) {
		if (channels[i].sensor_id == sensor_id) {
			return &channels[i];
		}
	}

	return nullptr;
}

static int channel_decimator_reset(struct mic_ble_channel_state *channel)
{
	arm_status status;

	filter_coeffs_init();
	memset(channel->filter_state, 0, sizeof(channel->filter_state));

	status = arm_fir_decimate_init_q15(&channel->decimator, MIC_BLE_FILTER_TAPS,
					   MIC_BLE_DECIMATION, filter_coeffs,
					   channel->filter_state, MIC_BLE_DECIMATION);
	if (status != ARM_MATH_SUCCESS) {
		LOG_ERR("Failed to initialize mic decimator: %d", status);
		channel->decimator_ready = false;
		return -EINVAL;
	}

	channel->packet_count = 0;
	channel->packet_time_us = 0;
	channel->decimator_ready = true;

	return 0;
}

static bool any_channel_active_locked(void)
{
	for (uint8_t i = 0; i < MIC_BLE_CHANNEL_COUNT; i++) {
		if (channels[i].enabled) {
			return true;
		}
	}

	return false;
}

static uint64_t timestamp_with_filter_delay(uint64_t sample_time_us)
{
	if (sample_time_us > MIC_BLE_FILTER_DELAY_US) {
		return sample_time_us - MIC_BLE_FILTER_DELAY_US;
	}

	return 0;
}

static void packet_emit(struct mic_ble_channel_state *channel)
{
	struct sensor_msg msg = {0};
	uint16_t dt_us = MIC_BLE_SAMPLE_PERIOD_US;

	msg.sd = false;
	msg.stream = true;
	msg.data.id = channel->sensor_id;
	msg.data.size = (channel->packet_count * sizeof(int16_t)) + sizeof(dt_us);
	msg.data.time = channel->packet_time_us;

	memcpy(msg.data.data, channel->packet, channel->packet_count * sizeof(int16_t));
	memcpy(&msg.data.data[msg.data.size - sizeof(dt_us)], &dt_us, sizeof(dt_us));

	if (sensor_queue != nullptr) {
		int ret = k_msgq_put(sensor_queue, &msg, K_NO_WAIT);
		if (ret != 0) {
			LOG_WRN("BLE mic sensor queue full");
		}
	}

	channel->packet_count = 0;
	channel->packet_time_us = 0;
}

static void packet_add_sample(struct mic_ble_channel_state *channel, int16_t sample,
			      uint64_t sample_time_us)
{
	if (channel->packet_count == 0) {
		channel->packet_time_us = sample_time_us;
	}

	channel->packet[channel->packet_count] = sample;
	channel->packet_count++;

	if (channel->packet_count >= MIC_BLE_SAMPLES_PER_PACKET) {
		packet_emit(channel);
	}
}

void mic_ble_streamer_set_sensor_queue(struct k_msgq *queue)
{
	k_mutex_lock(&mic_ble_lock, K_FOREVER);
	sensor_queue = queue;
	k_mutex_unlock(&mic_ble_lock);
}

int mic_ble_streamer_enable(uint8_t sensor_id, bool enable)
{
	int ret = 0;

	k_mutex_lock(&mic_ble_lock, K_FOREVER);

	struct mic_ble_channel_state *channel = channel_for_sensor(sensor_id);
	if (channel == nullptr) {
		ret = -EINVAL;
		goto out;
	}

	if (enable) {
		ret = channel_decimator_reset(channel);
		if (ret == 0) {
			channel->enabled = true;
		}
	} else {
		channel->enabled = false;
		channel->packet_count = 0;
		channel->packet_time_us = 0;
	}

out:
	k_mutex_unlock(&mic_ble_lock);
	return ret;
}

bool mic_ble_streamer_is_active(void)
{
	bool active;

	k_mutex_lock(&mic_ble_lock, K_FOREVER);
	active = any_channel_active_locked();
	k_mutex_unlock(&mic_ble_lock);

	return active;
}

void mic_ble_streamer_process_i2s_block(const void *pcm_block, size_t size,
					uint64_t first_sample_time_us)
{
	if (pcm_block == nullptr || size != BLOCK_SIZE_BYTES) {
		return;
	}

	k_mutex_lock(&mic_ble_lock, K_FOREVER);

	if (!any_channel_active_locked()) {
		k_mutex_unlock(&mic_ble_lock);
		return;
	}

	const int16_t *pcm = static_cast<const int16_t *>(pcm_block);
	const uint64_t sample_time_us = timestamp_with_filter_delay(first_sample_time_us);

	for (uint8_t channel_idx = 0; channel_idx < MIC_BLE_CHANNEL_COUNT; channel_idx++) {
		struct mic_ble_channel_state *channel = &channels[channel_idx];

		if (!channel->enabled || !channel->decimator_ready) {
			continue;
		}

		for (uint32_t i = 0; i < MIC_BLE_DECIMATION; i++) {
			channel->input[i] = pcm[(i * CONFIG_I2S_CH_NUM) + channel->i2s_channel];
		}

		q15_t output;
		arm_fir_decimate_q15(&channel->decimator, channel->input, &output,
				     MIC_BLE_DECIMATION);

		packet_add_sample(channel, output, sample_time_us);
	}

	k_mutex_unlock(&mic_ble_lock);
}
