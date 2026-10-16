/*
 *  Copyright (c) 2021, PACKETCRAFT, INC.
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

/** @file
 * @defgroup audio_app_datapath Audio Datapath
 * @{
 * @brief Audio datapath and synchronization API for Audio applications.
 *
 * This module implements the audio synchronization functionality required for
 * True Wireless Stereo (TWS) operation.
 */

#ifndef _AUDIO_DATAPATH_H_
#define _AUDIO_DATAPATH_H_

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include <zephyr/net_buf.h>

#include "sw_codec_select.h"
#include "audio_defines.h"

#define SDU_REF_CH_DELTA_MAX_US (int)(CONFIG_AUDIO_FRAME_DURATION_US * 0.001)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Mixes a tone into the I2S TX stream.
 *
 * @param	freq		Tone frequency [Hz].
 * @param	dur_ms		Tone duration [ms]. (0 == forever)
 * @param	amplitude	Tone amplitude [0, 1].
 *
 * @return	0 if successful, error otherwise.
 */
int audio_datapath_tone_play(uint16_t freq, uint16_t dur_ms, float amplitude);

/**
 * @brief	Stop tone playback.
 */
void audio_datapath_tone_stop(void);

int audio_datapath_buffer_play(int16_t *buffer, int num_samples, bool loop, float amplitude,
			       void (*callback)(void));
void audio_datapath_buffer_stop(void);
void record_to_buffer(int16_t *buffer, int num_samples, int initial_drop, bool left, bool right,
		      void (*callback)(void));
void record_to_buffer_stop(void);
void record_to_sd(bool active);
void audio_datapath_stop_recording(void);
int audio_datapath_auxiliary_suspend(void);
int audio_datapath_auxiliary_resume(void);
void set_sensor_queue(struct k_msgq *queue);
/* Share I2S with microphone capture and local audio; NULL means no encoder input. */
int audio_datapath_aquire(struct k_msgq *queue_rx);
/* Call after disabling the encoder producer, before starting another session. */
void audio_datapath_encoder_reset(void);
int audio_datapath_release(void);

int audio_datapath_decimator_init(uint8_t factor);
int audio_datapath_decimator_process(const int16_t *input, int16_t *output,
				     uint32_t num_frames);
void audio_datapath_decimator_cleanup(void);
void audio_datapath_decimator_reset(void);
uint8_t audio_datapath_decimator_get_factor(void);

/**
 * @brief	Set the presentation delay.
 *
 * @param	delay_us	The presentation delay in µs.
 *
 * @return	0 if successful, error otherwise.
 */
int audio_datapath_pres_delay_us_set(uint32_t delay_us);

/**
 * @brief	Get the current presentation delay.
 *
 * @param	delay_us	The presentation delay in µs.
 */
void audio_datapath_pres_delay_us_get(uint32_t *delay_us);

/**
 * @brief	Input an audio data frame which is processed and outputted over I2S.
 *
 * @note	A frame of raw encoded audio data is inputted, and this data then is decoded
 *		and processed before being outputted over I2S. The audio is synchronized
 *		using sdu_ref_us.
 *
 * @param	audio_frame_in	Pointer to the coded audio input buffer.
 */
void audio_datapath_stream_out(struct net_buf *audio_frame_in);

/**
 * @brief	Start the audio datapath module.
 *
 * @note	The continuously running I2S is started.
 *
 * @param	queue_rx	Pointer to the queue structure where I2S RX data is put.
 *
 * @return	0 if successful, error otherwise.
 */
int audio_datapath_start(struct k_msgq *queue_rx);

/**
 * @brief	Stop the audio datapath module.
 *
 * @return	0 if successful, error otherwise.
 */
int audio_datapath_stop(void);

/**
 * @brief	Initialize the audio datapath module.
 *
 * @return	0 if successful, error otherwise.
 */
int audio_datapath_init(void);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* _AUDIO_DATAPATH_H_ */
