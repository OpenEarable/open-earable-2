/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "audio_usb_device.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/usb/class/usb_audio.h>
#include <zephyr/usb/usb_device.h>

#include "zbus_common.h"
#include "audio_clock.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_usb_device, CONFIG_MODULE_AUDIO_USB_LOG_LEVEL);

#define USB_AUDIO_CHANNEL_COUNT 2
#define USB_AUDIO_FRAME_SIZE                                                                  \
	(((CONFIG_AUDIO_SAMPLE_RATE_HZ * CONFIG_AUDIO_BIT_DEPTH_OCTETS) / 1000) *                \
	 USB_AUDIO_CHANNEL_COUNT)
/* Microphone: one buffer in flight, one being filled and two ready to send. */
#define USB_MIC_BUF_COUNT	  4
/* Frames buffered before the first microphone frame is sent. This keeps one
 * frame of margin against USB request jitter, whatever the stream start phase.
 */
#define USB_MIC_PREFILL_FRAMES	  2
/* Playback holds buffers of the USB class pool (5 buffers), keep some for it. */
#define USB_PLAYBACK_QUEUE_DEPTH  2
#define USB_PLAYBACK_HOLD_TIME_MS 100
/* Frames buffered before playback starts, so the buffer starts near its target. */
#define USB_PLAYBACK_PREFILL_FRAMES 2

#define USB_VOLUME_STEP		  256 /* 1 dB in 1/256 dB */
/* The FDSP codec volume spans 0 (-60 dB) to 255 (0 dB). */
#define USB_VOLUME_RANGE_DB	  60
#define CODEC_VOLUME_MAX	  255
/* Feature unit channels: 0 = master, 1 = left, 2 = right. */
#define USB_VOLUME_CHANNELS	  (USB_AUDIO_CHANNEL_COUNT + 1)

/* The host stops polling the microphone when it closes the stream. */
#define USB_MIC_IDLE_TIME_MS	  20

/* Clock synchronization between the USB host and the I2S (HFCLKAUDIO) clock.
 * Neither direction has a rate feedback, so the audio clock follows the host:
 * the mean buffer level is regulated towards a target. One APLL step is about
 * 3.3 ppm.
 */
#define USB_FRAME_US		  1000
#define USB_SYNC_PERIOD_BLKS	  200 /* 100 ms of 0.5 ms I2S blocks */
#define USB_SYNC_PLAYBACK_TARGET_US 1250
#define USB_SYNC_MIC_TARGET_US	  1500
/* PI gains in APLL steps per us of buffer level error (scaled by 1/1000). */
#define USB_SYNC_KP_MILLI	  100
#define USB_SYNC_KI_MILLI	  2
#define USB_SYNC_MAX_STEPS	  150 /* about +-500 ppm */
/* The host sends a frame every 1 ms while it plays. */
#define USB_SYNC_DATA_TIMEOUT_MS  5
/* Over- and underruns only count while the host transfers every 1 ms. When a
 * stream ends, the buffers run full or empty without any audio being lost.
 */
#define USB_XRUN_WINDOW_MS	  2

ZBUS_CHAN_DECLARE(volume_chan);

/* Both queues carry net_buf pointers, so audio data is never copied twice. */
K_MSGQ_DEFINE(usb_mic_queue, sizeof(struct net_buf *), USB_MIC_BUF_COUNT, sizeof(void *));
K_MSGQ_DEFINE(usb_playback_queue, sizeof(struct net_buf *), USB_PLAYBACK_QUEUE_DEPTH,
	      sizeof(void *));
NET_BUF_POOL_FIXED_DEFINE(pool_mic, USB_MIC_BUF_COUNT, USB_AUDIO_FRAME_SIZE, 0, net_buf_destroy);

static uint32_t last_mic_request_ms;
static bool mic_streaming;
static bool mic_prefilled;
static bool microphone_muted;
static bool headphones_muted;
/* Attenuation per feature unit channel in whole dB, set by the USB host. */
static uint8_t playback_attenuation_db[USB_VOLUME_CHANNELS];
static uint32_t last_playback_data_ms;
static uint32_t queue_overruns;
static uint32_t playback_queue_overruns;
static uint32_t tx_underruns;
static uint32_t playback_underruns;

/* Partially filled microphone frame and partially played playback frame. Both
 * are only accessed from the I2S block complete handler.
 */
static struct net_buf *mic_frame;
static struct net_buf *playback_frame;
static size_t playback_frame_pos;
static bool playback_running;

static bool block_size_valid(size_t size)
{
	return (size != 0U) && (size <= USB_AUDIO_FRAME_SIZE) &&
	       ((USB_AUDIO_FRAME_SIZE % size) == 0U);
}

static void microphone_data_requested(const struct device *dev)
{
	struct net_buf *buf_out;
	int ret;

	bool prefill;

	last_mic_request_ms = k_uptime_get_32();

	prefill = !mic_prefilled;
	if (prefill && (k_msgq_num_used_get(&usb_mic_queue) >= USB_MIC_PREFILL_FRAMES)) {
		mic_prefilled = true;
		prefill = false;
	}

	ret = prefill ? -EAGAIN : k_msgq_get(&usb_mic_queue, &buf_out, K_NO_WAIT);
	if (ret) {
		/* Send silence while prefilling or when no frame is ready. */
		if (!prefill) {
			tx_underruns++;
			LOG_WRN_RATELIMIT("USB microphone underrun. Count: %u", tx_underruns);
		}

		buf_out = net_buf_alloc(&pool_mic, K_NO_WAIT);
		if (buf_out == NULL) {
			return;
		}

		memset(net_buf_add(buf_out, USB_AUDIO_FRAME_SIZE), 0, USB_AUDIO_FRAME_SIZE);
	}

	if (microphone_muted) {
		memset(buf_out->data, 0, USB_AUDIO_FRAME_SIZE);
	}

	ret = usb_audio_send(dev, buf_out, USB_AUDIO_FRAME_SIZE);
	if (ret) {
		/* No completion callback will release the buffer on a failed submission. */
		net_buf_unref(buf_out);
		if (ret != -EAGAIN) {
			LOG_WRN("USB microphone transfer failed: %d", ret);
		}
	}
}

static void headphones_data_received(const struct device *dev, struct net_buf *buffer,
				     size_t size)
{
	struct net_buf *stale;
	int ret;

	ARG_UNUSED(dev);

	if (buffer == NULL) {
		return;
	}

	if (size != USB_AUDIO_FRAME_SIZE) {
		LOG_WRN("Unexpected USB playback frame size: %u", (unsigned int)size);
		net_buf_unref(buffer);
		return;
	}

	/* The queue takes ownership of the buffer. */
	ret = k_msgq_put(&usb_playback_queue, &buffer, K_NO_WAIT);
	if (ret) {
		/* Keep latency bounded: discard the oldest frame and retain the newest. */
		if (k_msgq_get(&usb_playback_queue, &stale, K_NO_WAIT) == 0) {
			net_buf_unref(stale);
		}
		ret = k_msgq_put(&usb_playback_queue, &buffer, K_NO_WAIT);
		playback_queue_overruns++;
		LOG_WRN_RATELIMIT("USB playback overrun. Count: %u", playback_queue_overruns);
	}

	if (ret) {
		net_buf_unref(buffer);
		return;
	}

	last_playback_data_ms = k_uptime_get_32();
}

static void playback_volume_set(uint8_t channel, int16_t volume)
{
	struct volume_msg msg = {.event = VOLUME_SET};
	uint32_t attenuation;
	int ret;

	if (channel >= USB_VOLUME_CHANNELS) {
		return;
	}

	attenuation = (volume >= 0) ? 0U : (uint32_t)(-volume) / USB_VOLUME_STEP;
	playback_attenuation_db[channel] = MIN(attenuation, USB_VOLUME_RANGE_DB);

	/* The codec has a single output volume: combine the master volume with the
	 * average of the channel volumes.
	 */
	attenuation = playback_attenuation_db[0] +
		      ((playback_attenuation_db[1] + playback_attenuation_db[2] + 1U) / 2U);
	attenuation = MIN(attenuation, USB_VOLUME_RANGE_DB);

	msg.volume = CODEC_VOLUME_MAX - DIV_ROUND_CLOSEST(attenuation * CODEC_VOLUME_MAX,
							   USB_VOLUME_RANGE_DB);

	/* The codec is accessed over I2C, so its volume thread applies the value. */
	ret = zbus_chan_pub(&volume_chan, &msg, K_NO_WAIT);
	if (ret) {
		LOG_WRN("Failed to publish USB playback volume: %d", ret);
		return;
	}

	LOG_DBG("USB playback volume -%u dB, codec volume %u", attenuation, msg.volume);
}

static void feature_update(const struct device *dev, const struct usb_audio_fu_evt *evt)
{
	ARG_UNUSED(dev);

	if ((evt->cs == USB_AUDIO_FU_VOLUME_CONTROL) && (evt->dir == USB_AUDIO_OUT) &&
	    (evt->val != NULL) && (evt->val_len >= sizeof(int16_t))) {
		playback_volume_set(evt->channel, (int16_t)sys_get_le16(evt->val));
		return;
	}

	if ((evt->cs == USB_AUDIO_FU_MUTE_CONTROL) && (evt->val != NULL) &&
	    (evt->val_len >= sizeof(uint8_t))) {
		if (evt->dir == USB_AUDIO_IN) {
			microphone_muted = (*(const uint8_t *)evt->val != 0U);
			LOG_DBG("USB microphone mute: %d", microphone_muted);
		} else {
			headphones_muted = (*(const uint8_t *)evt->val != 0U);
			LOG_DBG("USB playback mute: %d", headphones_muted);
		}
	}
}

static const struct usb_audio_ops ops = {
	.data_request_cb = microphone_data_requested,
	.data_received_cb = headphones_data_received,
	.feature_update_cb = feature_update,
};

static bool mic_active(void)
{
	return (last_mic_request_ms != 0U) &&
	       ((uint32_t)(k_uptime_get_32() - last_mic_request_ms) < USB_MIC_IDLE_TIME_MS);
}

static void mic_buffers_release(void)
{
	struct net_buf *stale;

	if (mic_frame != NULL) {
		net_buf_unref(mic_frame);
		mic_frame = NULL;
	}

	while (k_msgq_get(&usb_mic_queue, &stale, K_NO_WAIT) == 0) {
		net_buf_unref(stale);
	}
}

int audio_usb_device_mic_write(const void *data, size_t size)
{
	struct net_buf *stale;

	if ((data == NULL) || !block_size_valid(size)) {
		return -EINVAL;
	}

	/* Only buffer samples while the host polls the microphone. Drop what is
	 * left when it stops, so a new stream does not start with old samples.
	 */
	if (!mic_active()) {
		if (mic_streaming) {
			mic_streaming = false;
			mic_prefilled = false;
			mic_buffers_release();
		}
		return 0;
	}
	mic_streaming = true;

	if (mic_frame == NULL) {
		mic_frame = net_buf_alloc(&pool_mic, K_NO_WAIT);
		if (mic_frame == NULL) {
			/* All buffers wait for the host: reuse the oldest one. */
			if (k_msgq_get(&usb_mic_queue, &stale, K_NO_WAIT)) {
				return -ENOMEM;
			}

			net_buf_reset(stale);
			mic_frame = stale;
			if ((uint32_t)(k_uptime_get_32() - last_mic_request_ms) < USB_XRUN_WINDOW_MS) {
				queue_overruns++;
				LOG_WRN_RATELIMIT("USB microphone overrun. Count: %u",
						  queue_overruns);
			}
		}
	}

	net_buf_add_mem(mic_frame, data, size);
	if (mic_frame->len < USB_AUDIO_FRAME_SIZE) {
		return 0;
	}

	/* Cannot fail: the queue holds as many entries as the pool has buffers. */
	(void)k_msgq_put(&usb_mic_queue, &mic_frame, K_NO_WAIT);
	mic_frame = NULL;

	return 0;
}

int audio_usb_device_playback_read(void *data, size_t size)
{
	uint32_t now_ms;

	if ((data == NULL) || !block_size_valid(size)) {
		return -EINVAL;
	}

	if ((playback_frame == NULL) &&
	    (playback_running ||
	     (k_msgq_num_used_get(&usb_playback_queue) >= USB_PLAYBACK_PREFILL_FRAMES))) {
		if (k_msgq_get(&usb_playback_queue, &playback_frame, K_NO_WAIT) == 0) {
			playback_frame_pos = 0;
			playback_running = true;
		} else {
			playback_frame = NULL;
		}
	}

	if (playback_frame != NULL) {
		if (headphones_muted) {
			memset(data, 0, size);
		} else {
			memcpy(data, &playback_frame->data[playback_frame_pos], size);
		}

		playback_frame_pos += size;
		if (playback_frame_pos >= USB_AUDIO_FRAME_SIZE) {
			net_buf_unref(playback_frame);
			playback_frame = NULL;
		}

		return 0;
	}

	/* Hold ownership briefly across normal USB packet jitter. This prevents a
	 * stale Bluetooth block from becoming audible between USB packets.
	 */
	now_ms = k_uptime_get_32();
	if ((last_playback_data_ms != 0U) &&
	    ((uint32_t)(now_ms - last_playback_data_ms) < USB_PLAYBACK_HOLD_TIME_MS)) {
		/* While the host still sends, an empty buffer is an underrun. */
		if (playback_running &&
		    ((uint32_t)(now_ms - last_playback_data_ms) < USB_XRUN_WINDOW_MS)) {
			playback_underruns++;
			LOG_WRN_RATELIMIT("USB playback underrun. Count: %u", playback_underruns);
		}

		memset(data, 0, size);
		return 0;
	}

	playback_running = false;

	return -ENOMSG;
}

/* Buffered audio in us, sampled once per I2S block. */
static int32_t playback_level_us(void)
{
	size_t bytes = k_msgq_num_used_get(&usb_playback_queue) * USB_AUDIO_FRAME_SIZE;

	if (playback_frame != NULL) {
		bytes += USB_AUDIO_FRAME_SIZE - playback_frame_pos;
	}

	return (int32_t)(bytes * USB_FRAME_US / USB_AUDIO_FRAME_SIZE);
}

static int32_t mic_level_us(void)
{
	size_t bytes = k_msgq_num_used_get(&usb_mic_queue) * USB_AUDIO_FRAME_SIZE;

	if (mic_frame != NULL) {
		bytes += mic_frame->len;
	}

	return (int32_t)(bytes * USB_FRAME_US / USB_AUDIO_FRAME_SIZE);
}

void audio_usb_device_clock_sync(bool allowed)
{
	static enum { SYNC_NONE, SYNC_PLAYBACK, SYNC_MIC } source;
	static int64_t level_sum;
	static uint32_t level_cnt;
	static int32_t integral;
	static int32_t steps;
	int32_t level;
	int32_t err;
	int32_t new_steps;
	int new_source = SYNC_NONE;

	if (allowed) {
		/* Only while the host delivers frames: when playback ends, its buffer
		 * drains and its level no longer reflects the clock offset.
		 */
		if (playback_running &&
		    ((uint32_t)(k_uptime_get_32() - last_playback_data_ms) < USB_SYNC_DATA_TIMEOUT_MS)) {
			new_source = SYNC_PLAYBACK;
		} else if (mic_streaming) {
			new_source = SYNC_MIC;
		}
	}

	if (new_source != source) {
		/* Restart the level average, the other buffer has an unrelated level.
		 * The integral holds the learned clock offset to the host, so a new
		 * stream starts locked. Bluetooth may have set the clock meanwhile.
		 */
		source = new_source;
		level_sum = 0;
		level_cnt = 0;
		if (source != SYNC_NONE) {
			steps = (integral * USB_SYNC_KI_MILLI) / 1000;
			(void)audio_clock_set(APLL_FREQ_CENTER + steps);
		}
	}

	if (source == SYNC_NONE) {
		return;
	}

	level_sum += (source == SYNC_PLAYBACK) ? playback_level_us() : mic_level_us();
	if (++level_cnt < USB_SYNC_PERIOD_BLKS) {
		return;
	}

	level = (int32_t)(level_sum / level_cnt);
	level_sum = 0;
	level_cnt = 0;

	/* More playback data than wanted: I2S is too slow, raise the clock.
	 * More microphone data than wanted: I2S is too fast, lower the clock.
	 */
	if (source == SYNC_PLAYBACK) {
		err = level - USB_SYNC_PLAYBACK_TARGET_US;
	} else {
		err = USB_SYNC_MIC_TARGET_US - level;
	}

	integral = CLAMP(integral + err, -USB_SYNC_MAX_STEPS * 1000 / USB_SYNC_KI_MILLI,
			 USB_SYNC_MAX_STEPS * 1000 / USB_SYNC_KI_MILLI);
	new_steps = (err * USB_SYNC_KP_MILLI + integral * USB_SYNC_KI_MILLI) / 1000;
	new_steps = CLAMP(new_steps, -USB_SYNC_MAX_STEPS, USB_SYNC_MAX_STEPS);

	if (new_steps != steps) {
		steps = new_steps;
		(void)audio_clock_set(APLL_FREQ_CENTER + steps);
	}

	LOG_DBG("USB sync %s: level %d us, steps %d", (source == SYNC_PLAYBACK) ? "hp" : "mic",
		level, steps);
}

int audio_usb_device_init(void)
{
	const struct device *headset_dev = DEVICE_DT_GET(DT_NODELABEL(usb_audio_headset));
	size_t frame_size;

	if (!device_is_ready(headset_dev)) {
		LOG_ERR("USB headset device is not ready");
		return -EIO;
	}

	frame_size = usb_audio_get_in_frame_size(headset_dev);
	if (frame_size != USB_AUDIO_FRAME_SIZE) {
		LOG_ERR("Unexpected USB microphone frame size: %u (expected %u)",
			(unsigned int)frame_size, (unsigned int)USB_AUDIO_FRAME_SIZE);
		return -EINVAL;
	}

	usb_audio_register(headset_dev, &ops);

	LOG_INF("Stereo USB headset registered (%u Hz, %u bit)", CONFIG_AUDIO_SAMPLE_RATE_HZ,
		CONFIG_AUDIO_BIT_DEPTH_BITS);

	return 0;
}
