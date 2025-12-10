/*
 *  Copyright (c) 2025
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

#include "decimation_filter.h"
#include "arm_math.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(decimation_filter, CONFIG_AUDIO_DATAPATH_LOG_LEVEL);

#define MAX_FRAMES 512
#define NUM_STAGES 2

/* Float32 coefficients for stereo DF2T biquad (b0, b1, b2, -a1, -a2 per stage) */
// static const float32_t coeff_dec4[NUM_STAGES * 5] = {
// 	/* Stage 1: Butterworth LP, Fc=19.2kHz @ 192kHz */
// 	0.00512f, 0.01024f, 0.00512f, 1.14441f, -0.20489f,
// 	/* Stage 2 */
// 	1.0f, 2.0f, 1.0f, 1.11169f, -0.28652f
// };

// static const float32_t coeff_dec2[NUM_STAGES * 5] = {
// 	/* Stage 1: Butterworth LP, Fc=~24kHz @ 96kHz */
// 	0.04702f, 0.09404f, 0.04702f, 0.00000f, -0.01977f,
// 	/* Stage 2 */
// 	1.0f, 2.0f, 1.0f, 0.00000f, -0.22314f
// };

static const float32_t coeff_dec4[NUM_STAGES * 5] = {
	/* Stage 1: Butterworth LP, Fc=19.2kHz @ 192kHz */
	0.01020948, 0.02041896, 0.01020948, 0.85539793, -0.20971536,
	/* Stage 2 */
	1.0f, 2.0f, 1.0f, 1.11302985, -0.57406192
};

static const float32_t coeff_dec2[NUM_STAGES * 5] = {
	/* Stage 1: Butterworth LP, Fc=19.2kHz @ 192kHz */
	9.39808514e-02, 1.87961703e-01, 9.39808514e-02, 1.38777878e-16,  -3.95661299e-02,
	/* Stage 2 */
	1.0f, 2.0f, 1.0f, 1.11022302e-16,  -4.46462692e-01
};

static const float32_t coeff_dec3[NUM_STAGES * 5] = {
	/* Stage 1: Butterworth LP, Fc=19.2kHz @ 192kHz */
	0.01020948, 0.02041896, 0.01020948, 0.85539793, -0.20971536,
	/* Stage 2 */
	1.0f, 2.0f, 1.0f, 0.75108142,  -0.50216284
};

static struct {
	bool init;
	uint8_t factor;
	arm_biquad_cascade_stereo_df2T_instance_f32 biquad;
	float32_t state[4 * NUM_STAGES]; /* 4 states per stage for stereo DF2T */
	float32_t temp_f32[MAX_FRAMES * 2]; /* temp buffer for float conversion */
} ctx;

int decimation_filter_init(uint8_t decimation_factor)
{
	if (decimation_factor != 2 && decimation_factor != 4 && decimation_factor != 8) {
		return -EINVAL;
	}
	
	ctx.factor = decimation_factor;
	
	const float32_t *coeffs = (decimation_factor == 4) ? coeff_dec4 : coeff_dec2;
	
	arm_biquad_cascade_stereo_df2T_init_f32(&ctx.biquad, NUM_STAGES, coeffs, ctx.state);
	memset(ctx.state, 0, sizeof(ctx.state));
	
	ctx.init = true;
	LOG_INF("Decimation filter init: factor=%d", decimation_factor);
	
	return 0;
}

int decimation_filter_process(const int16_t *input, int16_t *output, uint32_t num_frames)
{
	if (!ctx.init || !input || !output || num_frames == 0 || num_frames > MAX_FRAMES) {
		return -EINVAL;
	}
	
	uint32_t num_samples = num_frames * 2;
	const float32_t scale_to_f32 = 1.0f / 32768.0f;
	const float32_t scale_to_i16 = 32767.0f;
	
	/* Convert int16 to float32 */
	for (uint32_t i = 0; i < num_samples; i++) {
		ctx.temp_f32[i] = (float32_t)input[i]; // * scale_to_f32;
	}
	
	/* Apply anti-aliasing filter using stereo DF2T */
	static float32_t filtered[MAX_FRAMES * 2];
	arm_biquad_cascade_stereo_df2T_f32(&ctx.biquad, ctx.temp_f32, filtered, num_frames);

	arm_clip_f32(filtered, filtered, -32768.0f, 32767.0f, num_samples);
	
	/* Decimate and convert back to int16 */
	uint32_t out_frames = num_frames / ctx.factor;
	uint32_t step = ctx.factor * 2;
	
	for (uint32_t i = 0; i < out_frames; i++) {
		float32_t val_l = filtered[i * step]; // * scale_to_i16;
		float32_t val_r = filtered[i * step + 1]; // * scale_to_i16;
		
		output[i * 2] = (int16_t)val_l;
		output[i * 2 + 1] = (int16_t)val_r;
	}
	
	return out_frames;
}

void decimation_filter_reset(void)
{
	if (ctx.init) {
		memset(ctx.state, 0, sizeof(ctx.state));
	}
}
