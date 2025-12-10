/*
 *  Copyright (c) 2025
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

#include "decimation_filter.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(decimation_filter, CONFIG_AUDIO_DATAPATH_LOG_LEVEL);

#define MAX_FRAMES 512
#define NUM_STAGES 2

static const q15_t coeff_dec4[NUM_STAGES * 6] = {
	/* Stage 1: Butterworth LP, Fc=19.2kHz @ 192kHz */
	167, 0, 335, 167, 14015, -3436,
	/* Stage 2 */
	16384, 0, 32767, 16384, 18236, -9405
};

static const q15_t coeff_dec3[NUM_STAGES * 6] = {
	/* Stage 1: Butterworth LP, Fc=19.2kHz @ 192kHz */
	427, 0, 855, 427, 9102, -1819,
	/* Stage 2 */
	16384, 0, 32767, 16384, 12306, -8227
};

static const q15_t coeff_dec2[NUM_STAGES * 6] = {
	/* Stage 1: Butterworth LP, Fc=19.2kHz @ 192kHz */
	1540, 0, 3080, 1540, 0, -648,
	/* Stage 2 */
	16384, 0, 32767, 16384, 0, -7315
};

const int post_shift = 1;

static struct {
	bool init;
	uint8_t factor;
	arm_biquad_casd_df1_inst_q15 biquad;
	q15_t state[4 * NUM_STAGES];
	q15_t postshift;
} ctx;

int decimation_filter_init(uint8_t decimation_factor)
{
	if (decimation_factor != 2 && decimation_factor != 4 && decimation_factor != 8) {
		return -EINVAL;
	}
	
	ctx.factor = decimation_factor;
	
	const q15_t *coeffs = (decimation_factor == 4) ? coeff_dec4 : coeff_dec2;
	ctx.postshift = post_shift; /* Shift for Q1.15 */
	
	arm_biquad_cascade_df1_init_q15(&ctx.biquad, NUM_STAGES, coeffs, ctx.state, ctx.postshift);
	memset(ctx.state, 0, sizeof(ctx.state));
	
	ctx.init = true;
	LOG_INF("Decimation filter init: factor=%d", decimation_factor);
	
	return 0;
}

int decimation_filter_process(const q15_t *input, q15_t *output, uint32_t num_frames)
{
	if (!ctx.init || !input || !output || num_frames == 0 || num_frames > MAX_FRAMES) {
		return -EINVAL;
	}
	
	static q15_t temp[MAX_FRAMES * 2];
	uint32_t num_samples = num_frames * 2;
	
	/* Apply anti-aliasing filter */
	arm_biquad_cascade_df1_q15(&ctx.biquad, (q15_t*)input, temp, num_samples);
	
	/* Decimate */
	uint32_t out_frames = num_frames / ctx.factor;
	uint32_t step = ctx.factor * 2;
	
	for (uint32_t i = 0; i < out_frames; i++) {
		output[i * 2] = temp[i * step];
		output[i * 2 + 1] = temp[i * step + 1];
	}
	
	return out_frames;
}

void decimation_filter_reset(void)
{
	if (ctx.init) {
		memset(ctx.state, 0, sizeof(ctx.state));
	}
}
