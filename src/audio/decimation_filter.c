/*
 *  Copyright (c) 2025
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

#include "decimation_filter.h"
#include "arm_math.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(decimation_filter, CONFIG_AUDIO_DATAPATH_LOG_LEVEL);

// C interface using CascadedDecimator internally
#ifdef __cplusplus
static CascadedDecimator* g_decimator = nullptr;
#else
// Fallback for pure C compilation
#define MAX_FRAMES 512
#define NUM_STAGES 2

static const float32_t coeff_dec4[NUM_STAGES * 5] = {
	/* Stage 1: Butterworth LP, Fc=19.2kHz @ 192kHz */
	0.01020948f, 0.02041896f, 0.01020948f, 0.85539793f, -0.20971536f,
	/* Stage 2 */
	1.0f, 2.0f, 1.0f, 1.11302985f, -0.57406192f
};

static struct {
	bool init;
	uint8_t factor;
	arm_biquad_cascade_stereo_df2T_instance_f32 biquad;
	float32_t state[4 * NUM_STAGES];
	float32_t temp_f32[MAX_FRAMES * 2];
} ctx;
#endif

int decimation_filter_init(uint8_t decimation_factor)
{
#ifdef __cplusplus
	if (g_decimator) {
		delete g_decimator;
	}
	
	g_decimator = new CascadedDecimator(decimation_factor);
	if (!g_decimator) {
		LOG_ERR("Failed to create CascadedDecimator");
		return -ENOMEM;
	}
	
	int ret = g_decimator->init();
	if (ret != 0) {
		delete g_decimator;
		g_decimator = nullptr;
		LOG_ERR("Failed to initialize CascadedDecimator: %d", ret);
		return ret;
	}
	
	LOG_INF("Decimation filter initialized with factor %d", decimation_factor);
	return 0;
#else
	// Fallback C implementation
	if (decimation_factor != 4) {
		LOG_ERR("C fallback only supports factor 4");
		return -EINVAL;
	}
	
	ctx.factor = decimation_factor;
	
	arm_biquad_cascade_stereo_df2T_init_f32(&ctx.biquad, NUM_STAGES, coeff_dec4, ctx.state);
	memset(ctx.state, 0, sizeof(ctx.state));
	
	ctx.init = true;
	LOG_INF("Decimation filter init: factor=%d", decimation_factor);
	
	return 0;
#endif
}

int decimation_filter_process(const int16_t *input, int16_t *output, uint32_t num_frames)
{
#ifdef __cplusplus
	if (!g_decimator) {
		LOG_ERR("Decimator not initialized");
		return -EINVAL;
	}
	
	return g_decimator->process(input, output, num_frames);
#else
	// Fallback C implementation
	if (!ctx.init || !input || !output || num_frames == 0 || num_frames > MAX_FRAMES) {
		return -EINVAL;
	}
	
	uint32_t num_samples = num_frames * 2;
	
	/* Convert int16 to float32 */
	for (uint32_t i = 0; i < num_samples; i++) {
		ctx.temp_f32[i] = (float32_t)input[i];
	}
	
	/* Apply anti-aliasing filter using stereo DF2T */
	static float32_t filtered[MAX_FRAMES * 2];
	arm_biquad_cascade_stereo_df2T_f32(&ctx.biquad, ctx.temp_f32, filtered, num_frames);

	arm_clip_f32(filtered, filtered, -32768.0f, 32767.0f, num_samples);
	
	/* Decimate and convert back to int16 */
	uint32_t out_frames = num_frames / ctx.factor;
	uint32_t step = ctx.factor * 2;
	
	for (uint32_t i = 0; i < out_frames; i++) {
		output[i * 2] = (int16_t)filtered[i * step];
		output[i * 2 + 1] = (int16_t)filtered[i * step + 1];
	}
	
	return out_frames;
#endif
}

void decimation_filter_reset(void)
{
#ifdef __cplusplus
	if (g_decimator) {
		g_decimator->reset();
	}
#else
	if (ctx.init) {
		memset(ctx.state, 0, sizeof(ctx.state));
	}
#endif
}
