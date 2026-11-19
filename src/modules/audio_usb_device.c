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
#include <zephyr/usb/class/usb_audio.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(audio_usb_device, CONFIG_MODULE_AUDIO_USB_LOG_LEVEL);

#define USB_AUDIO_CHANNEL_COUNT 2
#define USB_AUDIO_FRAME_SIZE                                                                  \
	(((CONFIG_AUDIO_SAMPLE_RATE_HZ * CONFIG_AUDIO_BIT_DEPTH_OCTETS) / 1000) *                \
	 USB_AUDIO_CHANNEL_COUNT)
/* Microphone: one buffer in flight, one being filled and one ready to send. */
#define USB_MIC_BUF_COUNT	  3
/* Playback holds buffers of the USB class pool (5 buffers), keep some for it. */
#define USB_PLAYBACK_QUEUE_DEPTH  2
#define USB_PLAYBACK_HOLD_TIME_MS 100

/* Both queues carry net_buf pointers, so audio data is never copied twice. */
K_MSGQ_DEFINE(usb_mic_queue, sizeof(struct net_buf *), USB_MIC_BUF_COUNT, sizeof(void *));
K_MSGQ_DEFINE(usb_playback_queue, sizeof(struct net_buf *), USB_PLAYBACK_QUEUE_DEPTH,
	      sizeof(void *));
NET_BUF_POOL_FIXED_DEFINE(pool_mic, USB_MIC_BUF_COUNT, USB_AUDIO_FRAME_SIZE, 0, net_buf_destroy);

static bool host_requested_data;
static bool microphone_muted;
static bool headphones_muted;
static uint32_t last_playback_data_ms;
static uint32_t queue_overruns;
static uint32_t playback_queue_overruns;
static uint32_t tx_underruns;

/* Partially filled microphone frame and partially played playback frame. Both
 * are only accessed from the I2S block complete handler.
 */
static struct net_buf *mic_frame;
static struct net_buf *playback_frame;
static size_t playback_frame_pos;

static bool block_size_valid(size_t size)
{
	return (size != 0U) && (size <= USB_AUDIO_FRAME_SIZE) &&
	       ((USB_AUDIO_FRAME_SIZE % size) == 0U);
}

static void microphone_data_requested(const struct device *dev)
{
	struct net_buf *buf_out;
	int ret;

	host_requested_data = true;

	ret = k_msgq_get(&usb_mic_queue, &buf_out, K_NO_WAIT);
	if (ret) {
		tx_underruns++;
		if ((tx_underruns % 1000U) == 1U) {
			LOG_WRN("USB microphone underrun. Count: %u", tx_underruns);
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
		if ((playback_queue_overruns % 1000U) == 1U) {
			LOG_WRN("USB playback overrun. Count: %u", playback_queue_overruns);
		}
	}

	if (ret) {
		net_buf_unref(buffer);
		return;
	}

	last_playback_data_ms = k_uptime_get_32();
}

static void feature_update(const struct device *dev, const struct usb_audio_fu_evt *evt)
{
	ARG_UNUSED(dev);

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

int audio_usb_device_mic_write(const void *data, size_t size)
{
	struct net_buf *stale;

	if ((data == NULL) || !block_size_valid(size)) {
		return -EINVAL;
	}

	/* Avoid retaining old samples until the host selects the streaming interface. */
	if (!host_requested_data) {
		if (mic_frame != NULL) {
			net_buf_unref(mic_frame);
			mic_frame = NULL;
		}
		return 0;
	}

	if (mic_frame == NULL) {
		mic_frame = net_buf_alloc(&pool_mic, K_NO_WAIT);
		if (mic_frame == NULL) {
			/* All buffers wait for the host: reuse the oldest one. */
			if (k_msgq_get(&usb_mic_queue, &stale, K_NO_WAIT)) {
				return -ENOMEM;
			}

			net_buf_reset(stale);
			mic_frame = stale;
			queue_overruns++;
			if ((queue_overruns % 1000U) == 1U) {
				LOG_WRN("USB microphone overrun. Count: %u", queue_overruns);
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

	if (playback_frame == NULL) {
		if (k_msgq_get(&usb_playback_queue, &playback_frame, K_NO_WAIT) == 0) {
			playback_frame_pos = 0;
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
		memset(data, 0, size);
		return 0;
	}

	return -ENOMSG;
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
