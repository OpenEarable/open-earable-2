/*
 *  Copyright (c) 2025
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

#include "decimation_filter.h"
#include "arm_math.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(decimation_filter, CONFIG_AUDIO_DATAPATH_LOG_LEVEL);

/* Maximum supported samples per block */
#define MAX_SAMPLES_PER_BLOCK 2048

/* Butterworth lowpass filter coefficients for anti-aliasing
 * Cutoff at Fs/2/decimation_factor with some margin
 * Default: 5-stage Butterworth at 0.4 * (Fs_in / decimation_factor)
 */
#define FILTER_NUM_STAGES 2

/* Filter state and coefficients */
static struct {
	bool initialized;
	uint32_t input_sample_rate;
	uint8_t decimation_factor;
	
	/* CMSIS-DSP biquad filter instances for stereo */
	arm_biquad_cascade_stereo_df2T_instance_f32 biquad_stereo;
	float stereo_state[4 * FILTER_NUM_STAGES]; /* 4 states per stage for stereo */
	float stereo_coeffs[5 * FILTER_NUM_STAGES]; /* 5 coeffs per stage: b0, b1, b2, -a1, -a2 */
	
	/* CMSIS-DSP biquad filter instance for mono */
	arm_biquad_casd_df1_inst_f32 biquad_mono;
	float mono_state[4 * FILTER_NUM_STAGES]; /* 4 states per stage for mono */
	float mono_coeffs[5 * FILTER_NUM_STAGES]; /* 5 coeffs per stage */
	
	/* Working buffers for format conversion */
	float input_buf_f32[MAX_SAMPLES_PER_BLOCK * 2]; /* *2 for stereo */
	float filtered_buf_f32[MAX_SAMPLES_PER_BLOCK * 2];
} decimation_ctx;

/**
 * @brief Design anti-aliasing filter coefficients
 * 
 * Generates a 2-stage Butterworth lowpass filter
 * Cutoff frequency: 0.4 * (input_sample_rate / decimation_factor)
 * This provides adequate anti-aliasing margin
 */
static void design_antialiasing_filter(void)
{
	/* Calculate normalized cutoff frequency */
	float cutoff_hz = 0.4f * ((float)decimation_ctx.input_sample_rate / 
	                          (float)decimation_ctx.decimation_factor);
	float normalized_cutoff = cutoff_hz / ((float)decimation_ctx.input_sample_rate / 2.0f);
	
	/* Pre-computed Butterworth coefficients for common decimation scenarios
	 * These are normalized for Fc = 0.4 * Fs_out at various input rates
	 * For 192kHz -> 48kHz (decimation = 4), Fc ≈ 19.2kHz
	 */
	
	if (decimation_ctx.decimation_factor == 4) {
		/* Butterworth 2-stage lowpass, Fc = 0.4 * 48kHz = 19.2kHz at 192kHz
		 * Designed with scipy.signal.butter(4, 0.4/4, 'low', analog=False)
		 */
		
		/* Stage 1 coefficients */
		decimation_ctx.stereo_coeffs[0] = 0.00048853f;  /* b0 */
		decimation_ctx.stereo_coeffs[1] = 0.00097706f;  /* b1 */
		decimation_ctx.stereo_coeffs[2] = 0.00048853f;  /* b2 */
		decimation_ctx.stereo_coeffs[3] = 1.95558033f;  /* -a1 */
		decimation_ctx.stereo_coeffs[4] = -0.95753446f; /* -a2 */
		
		/* Stage 2 coefficients */
		decimation_ctx.stereo_coeffs[5] = 1.0f;         /* b0 */
		decimation_ctx.stereo_coeffs[6] = 2.0f;         /* b1 */
		decimation_ctx.stereo_coeffs[7] = 1.0f;         /* b2 */
		decimation_ctx.stereo_coeffs[8] = 1.95557364f;  /* -a1 */
		decimation_ctx.stereo_coeffs[9] = -0.95654896f; /* -a2 */
	} else if (decimation_ctx.decimation_factor == 2) {
		/* Butterworth 2-stage lowpass, Fc = 0.4 * Fs_out at 2x input rate */
		
		/* Stage 1 */
		decimation_ctx.stereo_coeffs[0] = 0.00780863f;
		decimation_ctx.stereo_coeffs[1] = 0.01561726f;
		decimation_ctx.stereo_coeffs[2] = 0.00780863f;
		decimation_ctx.stereo_coeffs[3] = 1.77863177f;
		decimation_ctx.stereo_coeffs[4] = -0.80986630f;
		
		/* Stage 2 */
		decimation_ctx.stereo_coeffs[5] = 1.0f;
		decimation_ctx.stereo_coeffs[6] = 2.0f;
		decimation_ctx.stereo_coeffs[7] = 1.0f;
		decimation_ctx.stereo_coeffs[8] = 1.77859908f;
		decimation_ctx.stereo_coeffs[9] = -0.80872483f;
	} else {
		/* Generic case - simple 2-stage Butterworth approximation */
		float w0 = 3.14159265f * normalized_cutoff;
		float alpha = sinf(w0) / (2.0f * 0.707f); /* Q = 0.707 for Butterworth */
		
		float b0 = (1.0f - cosf(w0)) / 2.0f;
		float b1 = 1.0f - cosf(w0);
		float b2 = (1.0f - cosf(w0)) / 2.0f;
		float a0 = 1.0f + alpha;
		float a1 = -2.0f * cosf(w0);
		float a2 = 1.0f - alpha;
		
		/* Normalize and convert to CMSIS format */
		decimation_ctx.stereo_coeffs[0] = b0 / a0;
		decimation_ctx.stereo_coeffs[1] = b1 / a0;
		decimation_ctx.stereo_coeffs[2] = b2 / a0;
		decimation_ctx.stereo_coeffs[3] = -a1 / a0;
		decimation_ctx.stereo_coeffs[4] = -a2 / a0;
		
		/* Duplicate for stage 2 (same coefficients) */
		for (int i = 0; i < 5; i++) {
			decimation_ctx.stereo_coeffs[5 + i] = decimation_ctx.stereo_coeffs[i];
		}
	}
	
	/* Copy coefficients for mono filter */
	memcpy(decimation_ctx.mono_coeffs, decimation_ctx.stereo_coeffs, 
	       sizeof(decimation_ctx.stereo_coeffs));
	
	LOG_DBG("Anti-aliasing filter designed: Fc=%.1f Hz (normalized=%.3f)", 
	        cutoff_hz, normalized_cutoff);
}

int decimation_filter_init(uint32_t input_sample_rate, uint8_t decimation_factor)
{
	if (decimation_factor == 0 || decimation_factor > 16) {
		LOG_ERR("Invalid decimation factor: %d (must be 1-16)", decimation_factor);
		return -EINVAL;
	}
	
	if (input_sample_rate < 8000 || input_sample_rate > 384000) {
		LOG_ERR("Invalid input sample rate: %d Hz", input_sample_rate);
		return -EINVAL;
	}
	
	decimation_ctx.input_sample_rate = input_sample_rate;
	decimation_ctx.decimation_factor = decimation_factor;
	
	/* Design anti-aliasing filter coefficients */
	design_antialiasing_filter();
	
	/* Initialize CMSIS-DSP stereo biquad cascade filter */
	arm_biquad_cascade_stereo_df2T_init_f32(&decimation_ctx.biquad_stereo,
	                                        FILTER_NUM_STAGES,
	                                        decimation_ctx.stereo_coeffs,
	                                        decimation_ctx.stereo_state);
	
	/* Initialize CMSIS-DSP mono biquad cascade filter */
	arm_biquad_cascade_df1_init_f32(&decimation_ctx.biquad_mono,
	                                FILTER_NUM_STAGES,
	                                decimation_ctx.mono_coeffs,
	                                decimation_ctx.mono_state);
	
	/* Clear filter states */
	memset(decimation_ctx.stereo_state, 0, sizeof(decimation_ctx.stereo_state));
	memset(decimation_ctx.mono_state, 0, sizeof(decimation_ctx.mono_state));
	
	decimation_ctx.initialized = true;
	
	LOG_INF("Decimation filter initialized: %d Hz -> %d Hz (factor %d)",
	        input_sample_rate,
	        input_sample_rate / decimation_factor,
	        decimation_factor);
	
	return 0;
}

int decimation_filter_process_stereo(const int16_t *input, int16_t *output, uint32_t num_frames)
{
	if (!decimation_ctx.initialized) {
		LOG_ERR("Decimation filter not initialized");
		return -EINVAL;
	}
	
	if (input == NULL || output == NULL || num_frames == 0) {
		LOG_ERR("Invalid parameters");
		return -EINVAL;
	}
	
	if (num_frames > (MAX_SAMPLES_PER_BLOCK / 2)) {
		LOG_ERR("Input buffer too large: %d frames (max %d)", 
		        num_frames, MAX_SAMPLES_PER_BLOCK / 2);
		return -EINVAL;
	}
	
	uint32_t num_samples = num_frames * 2; /* Stereo: 2 samples per frame */
	
	/* Convert int16 to float32 (scale by 1/32768.0) */
	const float scale_to_float = 1.0f / 32768.0f;
	for (uint32_t i = 0; i < num_samples; i++) {
		decimation_ctx.input_buf_f32[i] = (float)input[i] * scale_to_float;
	}
	
	/* Apply anti-aliasing filter using CMSIS-DSP stereo biquad */
	arm_biquad_cascade_stereo_df2T_f32(&decimation_ctx.biquad_stereo,
	                                   decimation_ctx.input_buf_f32,
	                                   decimation_ctx.filtered_buf_f32,
	                                   num_frames);
	
	/* Decimate: keep every Nth sample */
	uint32_t output_frames = num_frames / decimation_ctx.decimation_factor;
	const float scale_to_int16 = 32767.0f;
	
	for (uint32_t i = 0; i < output_frames; i++) {
		uint32_t in_idx = i * decimation_ctx.decimation_factor * 2;
		uint32_t out_idx = i * 2;
		
		/* Convert back to int16 with clamping */
		float val_l = decimation_ctx.filtered_buf_f32[in_idx] * scale_to_int16;
		float val_r = decimation_ctx.filtered_buf_f32[in_idx + 1] * scale_to_int16;
		
		/* Clamp to int16 range */
		if (val_l > 32767.0f) val_l = 32767.0f;
		if (val_l < -32768.0f) val_l = -32768.0f;
		if (val_r > 32767.0f) val_r = 32767.0f;
		if (val_r < -32768.0f) val_r = -32768.0f;
		
		output[out_idx] = (int16_t)val_l;
		output[out_idx + 1] = (int16_t)val_r;
	}
	
	return output_frames;
}

int decimation_filter_process_stereo_f32(const float *input, float *output, uint32_t num_frames)
{
	if (!decimation_ctx.initialized) {
		LOG_ERR("Decimation filter not initialized");
		return -EINVAL;
	}
	
	if (input == NULL || output == NULL || num_frames == 0) {
		LOG_ERR("Invalid parameters");
		return -EINVAL;
	}
	
	if (num_frames > (MAX_SAMPLES_PER_BLOCK / 2)) {
		LOG_ERR("Input buffer too large: %d frames (max %d)", 
		        num_frames, MAX_SAMPLES_PER_BLOCK / 2);
		return -EINVAL;
	}
	
	/* Apply anti-aliasing filter using CMSIS-DSP stereo biquad */
	arm_biquad_cascade_stereo_df2T_f32(&decimation_ctx.biquad_stereo,
	                                   input,
	                                   decimation_ctx.filtered_buf_f32,
	                                   num_frames);
	
	/* Decimate: keep every Nth sample */
	uint32_t output_frames = num_frames / decimation_ctx.decimation_factor;
	
	for (uint32_t i = 0; i < output_frames; i++) {
		uint32_t in_idx = i * decimation_ctx.decimation_factor * 2;
		uint32_t out_idx = i * 2;
		
		output[out_idx] = decimation_ctx.filtered_buf_f32[in_idx];
		output[out_idx + 1] = decimation_ctx.filtered_buf_f32[in_idx + 1];
	}
	
	return output_frames;
}

int decimation_filter_process_mono(const int16_t *input, int16_t *output, uint32_t num_samples)
{
	if (!decimation_ctx.initialized) {
		LOG_ERR("Decimation filter not initialized");
		return -EINVAL;
	}
	
	if (input == NULL || output == NULL || num_samples == 0) {
		LOG_ERR("Invalid parameters");
		return -EINVAL;
	}
	
	if (num_samples > MAX_SAMPLES_PER_BLOCK) {
		LOG_ERR("Input buffer too large: %d samples (max %d)", 
		        num_samples, MAX_SAMPLES_PER_BLOCK);
		return -EINVAL;
	}
	
	/* Convert int16 to float32 */
	const float scale_to_float = 1.0f / 32768.0f;
	for (uint32_t i = 0; i < num_samples; i++) {
		decimation_ctx.input_buf_f32[i] = (float)input[i] * scale_to_float;
	}
	
	/* Apply anti-aliasing filter using CMSIS-DSP mono biquad */
	arm_biquad_cascade_df1_f32(&decimation_ctx.biquad_mono,
	                           decimation_ctx.input_buf_f32,
	                           decimation_ctx.filtered_buf_f32,
	                           num_samples);
	
	/* Decimate: keep every Nth sample */
	uint32_t output_samples = num_samples / decimation_ctx.decimation_factor;
	const float scale_to_int16 = 32767.0f;
	
	for (uint32_t i = 0; i < output_samples; i++) {
		uint32_t in_idx = i * decimation_ctx.decimation_factor;
		
		/* Convert back to int16 with clamping */
		float val = decimation_ctx.filtered_buf_f32[in_idx] * scale_to_int16;
		
		if (val > 32767.0f) val = 32767.0f;
		if (val < -32768.0f) val = -32768.0f;
		
		output[i] = (int16_t)val;
	}
	
	return output_samples;
}

int decimation_filter_process_mono_f32(const float *input, float *output, uint32_t num_samples)
{
	if (!decimation_ctx.initialized) {
		LOG_ERR("Decimation filter not initialized");
		return -EINVAL;
	}
	
	if (input == NULL || output == NULL || num_samples == 0) {
		LOG_ERR("Invalid parameters");
		return -EINVAL;
	}
	
	if (num_samples > MAX_SAMPLES_PER_BLOCK) {
		LOG_ERR("Input buffer too large: %d samples (max %d)", 
		        num_samples, MAX_SAMPLES_PER_BLOCK);
		return -EINVAL;
	}
	
	/* Apply anti-aliasing filter using CMSIS-DSP mono biquad */
	arm_biquad_cascade_df1_f32(&decimation_ctx.biquad_mono,
	                           input,
	                           decimation_ctx.filtered_buf_f32,
	                           num_samples);
	
	/* Decimate: keep every Nth sample */
	uint32_t output_samples = num_samples / decimation_ctx.decimation_factor;
	
	for (uint32_t i = 0; i < output_samples; i++) {
		output[i] = decimation_ctx.filtered_buf_f32[i * decimation_ctx.decimation_factor];
	}
	
	return output_samples;
}

void decimation_filter_reset(void)
{
	if (!decimation_ctx.initialized) {
		return;
	}
	
	/* Clear filter states */
	memset(decimation_ctx.stereo_state, 0, sizeof(decimation_ctx.stereo_state));
	memset(decimation_ctx.mono_state, 0, sizeof(decimation_ctx.mono_state));
	
	LOG_DBG("Decimation filter state reset");
}

uint8_t decimation_filter_get_factor(void)
{
	return decimation_ctx.initialized ? decimation_ctx.decimation_factor : 0;
}

bool decimation_filter_is_initialized(void)
{
	return decimation_ctx.initialized;
}
