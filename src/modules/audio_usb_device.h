/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief OpenEarable as a bidirectional USB audio device (USB headset).
 *
 * The two ADAU1860 microphone channels are exposed as a stereo USB input and
 * stereo USB playback is routed to the codec output. Both directions run at
 * 48 kHz and 16 bit. The I2S datapath works on blocks that are shorter than a
 * USB frame (1 ms), so this module re-packs the data between both block sizes.
 */

#ifndef _AUDIO_USB_DEVICE_H_
#define _AUDIO_USB_DEVICE_H_

#include <stdbool.h>
#include <stddef.h>

#if !CONFIG_AUDIO_SAMPLE_RATE_48000_HZ
#error The OpenEarable USB audio device requires a 48 kHz sample rate
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Queue interleaved stereo microphone samples for the USB host.
 *
 * The data is copied, so the caller may release its source buffer immediately.
 * Safe to call from ISR context.
 *
 * @param data Pointer to the PCM data.
 * @param size Size of the PCM data in bytes. Must divide one USB frame (1 ms).
 * @return 0 on success, -EINVAL for an unsupported block size.
 */
int audio_usb_device_mic_write(const void *data, size_t size);

/**
 * @brief Read interleaved stereo USB playback samples.
 *
 * Safe to call from ISR context.
 *
 * @param data Destination for the PCM data.
 * @param size Size of the destination in bytes. Must divide one USB frame (1 ms).
 * @return 0 when USB owns the playback path, -ENOMSG when another source may play,
 *         -EINVAL for an unsupported block size.
 */
int audio_usb_device_playback_read(void *data, size_t size);

/**
 * @brief Let the audio clock follow the USB host clock.
 *
 * Call once per I2S block from the block complete handler. While a USB stream
 * is active, HFCLKAUDIO is adjusted to keep the USB buffers at a constant level.
 *
 * @param allowed False while another source (Bluetooth) controls the audio clock.
 */
void audio_usb_device_clock_sync(bool allowed);

/**
 * @brief Register the USB headset class callbacks.
 *
 * Must be called before usb_enable().
 *
 * @return 0 on success, a negative errno otherwise.
 */
int audio_usb_device_init(void);

#ifdef __cplusplus
}
#endif

#endif /* _AUDIO_USB_DEVICE_H_ */
