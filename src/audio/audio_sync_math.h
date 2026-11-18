/* SPDX-License-Identifier: Apache-2.0 */
#ifndef AUDIO_SYNC_MATH_H_
#define AUDIO_SYNC_MATH_H_

#include <stddef.h>
#include <stdint.h>

/* Hardware-independent arithmetic of the LE Audio playback synchronizer. Both
 * earphones run it independently against the same Bluetooth SDU reference, so
 * every microsecond of error here is heard as an interaural time difference.
 * All timestamps are free-running 32-bit microsecond counters; unsigned
 * subtraction followed by a signed cast is exact across their wrap.
 */

/* Maximum number of values audio_sync_median() accepts. */
#define AUDIO_SYNC_MEDIAN_MAX 8U

/* Phase of the I2S block grid relative to the SDU reference, folded into
 * [-block/2, block/2]. 2^32 is not a multiple of the block period, so the
 * difference must be taken modulo 2^32 before it is reduced to one block.
 */
static inline int32_t audio_sync_phase_err_us(uint32_t sdu_ref_us, uint32_t frame_start_ts_us,
					      int32_t blk_period_us)
{
	int32_t err_us = (int32_t)(sdu_ref_us - frame_start_ts_us) % blk_period_us;

	if (err_us > (blk_period_us / 2)) {
		err_us -= blk_period_us;
	} else if (err_us < -(blk_period_us / 2)) {
		err_us += blk_period_us;
	}

	return err_us;
}

/* Median of a few samples. A block-complete interrupt that is served late
 * reads a later frame start, and a single concealed SDU can carry a shifted
 * reference; neither may steer the audio clock on its own.
 */
static inline int32_t audio_sync_median(const int32_t *values, size_t count)
{
	int32_t sorted[AUDIO_SYNC_MEDIAN_MAX];

	if (count == 0U) {
		return 0;
	}
	if (count > AUDIO_SYNC_MEDIAN_MAX) {
		count = AUDIO_SYNC_MEDIAN_MAX;
	}

	for (size_t i = 0; i < count; i++) {
		size_t j = i;

		while (j > 0U && sorted[j - 1U] > values[i]) {
			sorted[j] = sorted[j - 1U];
			j--;
		}
		sorted[j] = values[i];
	}

	return sorted[count / 2U];
}

/* Audio PLL frequency word offset that shifts the block phase by 1.5 times
 * err_us within period_us. Callers pass half the measured error, giving a
 * loop gain of 0.75 per period. One word step is 3.31 ppm, hence the 331. A
 * shorter period needs a proportionally larger offset for the same gain.
 */
static inline int32_t audio_sync_freq_adj(int32_t err_us, uint32_t period_us)
{
	return (int32_t)(-((int64_t)err_us * 150000000LL) / (331LL * (int64_t)period_us));
}

/* Delay still to be added (positive) or removed (negative) so that a block
 * rendered rendered_dly_us after its own SDU reference meets the target.
 */
static inline int32_t audio_sync_pres_err_us(uint32_t target_dly_us, uint32_t rendered_dly_us)
{
	return (int32_t)(target_dly_us - rendered_dly_us);
}

/* Whole blocks nearest to a delay error; half a block rounds away from zero. */
static inline int32_t audio_sync_pres_adj_blks(int32_t err_us, int32_t blk_period_us)
{
	if (err_us >= 0) {
		return (err_us + (blk_period_us / 2)) / blk_period_us;
	}

	return (err_us - (blk_period_us / 2)) / blk_period_us;
}

#endif /* AUDIO_SYNC_MATH_H_ */
