/*
 *  Copyright (c) 2025
 *
 *  SPDX-License-Identifier: LicenseRef-PCFT
 */

#ifndef _DECIMATION_FILTER_H_
#define _DECIMATION_FILTER_H_

#include <stdint.h>
#include <stdbool.h>

#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize decimation filter
 * @param decimation_factor Decimation factor (2, 4, or 8)
 * @return 0 on success, -EINVAL on error
 */
int decimation_filter_init(uint8_t decimation_factor);

/**
 * @brief Process stereo Q15 audio with decimation
 * @param input Input buffer (interleaved stereo Q15)
 * @param output Output buffer (interleaved stereo Q15)
 * @param num_frames Number of input stereo frames
 * @return Number of output frames, or negative on error
 */
int decimation_filter_process(const q15_t *input, q15_t *output, uint32_t num_frames);

/**
 * @brief Reset filter state
 */
void decimation_filter_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* _DECIMATION_FILTER_H_ */
