/*
 *  Copyright (c) 2021, PACKETCRAFT, INC.
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

#ifndef _AUDIO_DATAPATH_H_
#define _AUDIO_DATAPATH_H_

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>
#include <data_fifo.h>
#include <zephyr/sys/ring_buffer.h>

#include "sw_codec_select.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Mixes a tone into the I2S TX stream
 *
 * @param freq Tone frequency [Hz]
 * @param dur_ms Tone duration [ms]. 0 = forever
 * @param amplitude Tone amplitude [0, 1]
 *
 * @return 0 if successful, error otherwise
 */
int audio_datapath_tone_play(uint16_t freq, uint16_t dur_ms, float amplitude);

/**
 * @brief Stops tone playback
 */
void audio_datapath_tone_stop(void);





/**
 * @brief Plays a buffer over I2S TX stream
 *
 * @param buffer Pointer to audio buffer to play
 * @param num_samples Number of samples in buffer
 * @param loop Whether to loop the buffer continuously
 * @param amplitude Playback amplitude [0, 1]
 * @param callback Callback function called when playback completes (optional)
 *
 * @return 0 if successful, error otherwise
 */
int audio_datapath_buffer_play(int16_t *buffer, int num_samples, bool loop, float amplitude, void (*callback)(void));

/**
 * @brief Stops buffer playback
 */
void audio_datapath_buffer_stop(void);

/**
 * @brief Records audio data to a buffer
 *
 * @param buffer Pointer to buffer where audio will be stored
 * @param num_samples Number of samples to record
 * @param initial_drop Number of initial samples to drop before recording
 * @param left Whether to record left channel
 * @param right Whether to record right channel
 * @param callback Callback function called when recording completes (optional)
 */
void record_to_buffer(int16_t *buffer, int num_samples, int initial_drop, bool left, bool right, void (*callback)(void));

/**
 * @brief Set the presentation delay
 *
 * @param delay_us The presentation delay in µs
 *
 * @return 0 if successful, error otherwise
 */
int audio_datapath_pres_delay_us_set(uint32_t delay_us);

/**
 * @brief Get the current presentation delay
 *
 * @param delay_us  The presentation delay in µs
 */
void audio_datapath_pres_delay_us_get(uint32_t *delay_us);

/**
 * @brief Input an audio data frame which is processed and outputted over I2S
 *
 * @note A frame of raw encoded audio data is inputted, and this data then is decoded
 *       and processed before being outputted over I2S. The audio is synchronized
 *       using sdu_ref_us
 *
 * @param buf Pointer to audio data frame
 * @param size Size of audio data frame in bytes
 * @param sdu_ref_us ISO timestamp reference from BLE controller
 * @param bad_frame Indicating if the audio frame is bad or not
 * @param recv_frame_ts_us Timestamp of when audio frame was received
 */
void audio_datapath_stream_out(const uint8_t *buf, size_t size, uint32_t sdu_ref_us, bool bad_frame,
			       uint32_t recv_frame_ts_us);

/**
 * @brief Start the audio datapath module
 *
 * @note The continuously running I2S is started
 *
 * @param fifo_rx Pointer to FIFO structure where I2S RX data is put
 *
 * @return 0 if successful, error otherwise
 */
int audio_datapath_start(struct data_fifo *fifo_rx);

/**
 * @brief Stop the audio datapath module
 *
 * @return 0 if successful, error otherwise
 */
int audio_datapath_stop(void);

/**
 * @brief Initialize the audio datapath module
 *
 * @return 0 if successful, error otherwise
 */
int audio_datapath_init(void);

void start_data_thread(void);

void record_to_sd(bool active);

void set_sensor_queue(struct k_msgq *queue);

int audio_datapath_aquire(struct data_fifo *fifo_rx);
int audio_datapath_release();

//void set_ring_buffer(struct ring_buf *ring_buf);

/**
 * @brief C wrapper for decimator initialization
 */
int audio_datapath_decimator_init(uint8_t factor);

/**
 * @brief C wrapper for decimator processing
 */
int audio_datapath_decimator_process(const int16_t* input, int16_t* output, uint32_t num_frames);

/**
 * @brief C wrapper for decimator cleanup
 */
void audio_datapath_decimator_cleanup(void);

/**
 * @brief C wrapper for decimator reset
 */
void audio_datapath_decimator_reset(void);

/**
 * @brief Get current decimation factor
 * @return Current total decimation factor, or 0 if not initialized
 */
uint8_t audio_datapath_decimator_get_factor(void);


#ifdef __cplusplus
}
#endif

#endif /* _AUDIO_DATAPATH_H_ */
