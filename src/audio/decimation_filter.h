/*
 *  Copyright (c) 2025
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

#ifndef _DECIMATION_FILTER_H_
#define _DECIMATION_FILTER_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the decimation filter with CMSIS-DSP
 * 
 * This function sets up the anti-aliasing lowpass filter and decimation
 * parameters. Must be called before processing any audio data.
 *
 * @param input_sample_rate  Input sampling rate in Hz (e.g., 192000)
 * @param decimation_factor  Decimation factor (e.g., 4 for 192kHz -> 48kHz)
 *
 * @retval 0 if successful
 * @retval -EINVAL if parameters are invalid
 */
int decimation_filter_init(uint32_t input_sample_rate, uint8_t decimation_factor);

/**
 * @brief Process stereo audio data with decimation filter
 *
 * Applies anti-aliasing lowpass filter and decimates the signal.
 * Input and output buffers can be the same for in-place processing.
 *
 * @param input         Pointer to input buffer (interleaved stereo samples)
 * @param output        Pointer to output buffer (interleaved stereo samples)
 * @param num_frames    Number of stereo frames in input buffer
 *
 * @retval Number of output frames produced (num_frames / decimation_factor)
 * @retval -EINVAL if filter not initialized or invalid parameters
 */
int decimation_filter_process_stereo(const int16_t *input, int16_t *output, uint32_t num_frames);

/**
 * @brief Process stereo audio data with decimation filter (float version)
 *
 * Applies anti-aliasing lowpass filter and decimates the signal.
 * Uses floating point internally for better precision.
 *
 * @param input         Pointer to input buffer (interleaved stereo float samples)
 * @param output        Pointer to output buffer (interleaved stereo float samples)
 * @param num_frames    Number of stereo frames in input buffer
 *
 * @retval Number of output frames produced (num_frames / decimation_factor)
 * @retval -EINVAL if filter not initialized or invalid parameters
 */
int decimation_filter_process_stereo_f32(const float *input, float *output, uint32_t num_frames);

/**
 * @brief Process mono audio data with decimation filter
 *
 * Applies anti-aliasing lowpass filter and decimates the signal.
 * Input and output buffers can be the same for in-place processing.
 *
 * @param input         Pointer to input buffer (mono samples)
 * @param output        Pointer to output buffer (mono samples)
 * @param num_samples   Number of samples in input buffer
 *
 * @retval Number of output samples produced (num_samples / decimation_factor)
 * @retval -EINVAL if filter not initialized or invalid parameters
 */
int decimation_filter_process_mono(const int16_t *input, int16_t *output, uint32_t num_samples);

/**
 * @brief Process mono audio data with decimation filter (float version)
 *
 * Applies anti-aliasing lowpass filter and decimates the signal.
 * Uses floating point internally for better precision.
 *
 * @param input         Pointer to input buffer (mono float samples)
 * @param output        Pointer to output buffer (mono float samples)
 * @param num_samples   Number of samples in input buffer
 *
 * @retval Number of output samples produced (num_samples / decimation_factor)
 * @retval -EINVAL if filter not initialized or invalid parameters
 */
int decimation_filter_process_mono_f32(const float *input, float *output, uint32_t num_samples);

/**
 * @brief Reset the decimation filter state
 *
 * Clears the filter state variables. Useful when starting a new stream
 * or recovering from an error condition.
 */
void decimation_filter_reset(void);

/**
 * @brief Get current decimation factor
 *
 * @return Current decimation factor, or 0 if not initialized
 */
uint8_t decimation_filter_get_factor(void);

/**
 * @brief Check if decimation filter is initialized
 *
 * @return true if initialized, false otherwise
 */
bool decimation_filter_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif /* _DECIMATION_FILTER_H_ */
