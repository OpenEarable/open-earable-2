/* SPDX-License-Identifier: Apache-2.0 */
#include <stdint.h>
#include <unity.h>
#include "audio_sync_math.h"

#define BLK_US	 500
#define FRAME_US 10000U

void setUp(void)
{
}

void test_phase_error_is_folded_to_half_a_block(void)
{
	TEST_ASSERT_EQUAL_INT32(0, audio_sync_phase_err_us(40000U, 20000U, BLK_US));
	TEST_ASSERT_EQUAL_INT32(7, audio_sync_phase_err_us(40007U, 20000U, BLK_US));
	TEST_ASSERT_EQUAL_INT32(-7, audio_sync_phase_err_us(20000U, 40007U, BLK_US));
	TEST_ASSERT_EQUAL_INT32(250, audio_sync_phase_err_us(40250U, 20000U, BLK_US));
	TEST_ASSERT_EQUAL_INT32(-249, audio_sync_phase_err_us(40251U, 20000U, BLK_US));
	TEST_ASSERT_EQUAL_INT32(249, audio_sync_phase_err_us(20000U, 40251U, BLK_US));
}

void test_phase_error_is_continuous_across_the_timestamp_wrap(void)
{
	/* The frame started 20 ms after an SDU reference taken 13 us before the
	 * counter wrapped. 2^32 is not a multiple of 500, so a 64-bit difference
	 * of the raw values reports -217 us here and unlocks the audio clock.
	 */
	uint32_t sdu_ref_us = UINT32_MAX - 12U;
	uint32_t frame_start_ts_us = sdu_ref_us + 20000U + 13U;

	TEST_ASSERT_TRUE(frame_start_ts_us < sdu_ref_us);
	TEST_ASSERT_EQUAL_INT32(-13,
				audio_sync_phase_err_us(sdu_ref_us, frame_start_ts_us, BLK_US));

	/* Every sample of a steady 13 us phase reads the same through the wrap. */
	for (uint32_t n = 0; n < 8U; n++) {
		uint32_t ref = UINT32_MAX - 3U * FRAME_US + n * FRAME_US;

		TEST_ASSERT_EQUAL_INT32(
			-13, audio_sync_phase_err_us(ref, ref + 40000U + 13U, BLK_US));
	}
}

void test_median_ignores_late_interrupts_and_single_reference_glitches(void)
{
	/* Two block-complete interrupts served one and three samples late. */
	const int32_t late_isr[] = {-4, -25, -3, -66, -4};
	/* One concealed SDU carrying a reference shifted by 180 us. */
	const int32_t glitch[] = {5, 6, 185, 5, 4};
	const int32_t even[] = {4, 1, 3, 2};
	const int32_t wide[] = {9, 8, 7, 6, 5, 4, 3, 2, 1000, 1000};

	TEST_ASSERT_EQUAL_INT32(-4, audio_sync_median(late_isr, 5));
	TEST_ASSERT_EQUAL_INT32(5, audio_sync_median(glitch, 5));
	TEST_ASSERT_EQUAL_INT32(3, audio_sync_median(even, 4));
	TEST_ASSERT_EQUAL_INT32(42, audio_sync_median(&(int32_t){42}, 1));
	TEST_ASSERT_EQUAL_INT32(0, audio_sync_median(glitch, 0));
	/* Only the supported number of samples is read. */
	TEST_ASSERT_EQUAL_INT32(6, audio_sync_median(wide, 10));
}

void test_presentation_error_uses_the_rendered_blocks_own_reference(void)
{
	const uint32_t target_us = 40000U;
	uint32_t sdu_ref_us = UINT32_MAX - 25000U;

	/* Rendered exactly on time, 1 ms early and 1.5 ms late, with the render
	 * timestamp beyond the counter wrap.
	 */
	TEST_ASSERT_EQUAL_INT32(
		0, audio_sync_pres_err_us(target_us, (sdu_ref_us + 40000U) - sdu_ref_us));
	TEST_ASSERT_EQUAL_INT32(
		1000, audio_sync_pres_err_us(target_us, (sdu_ref_us + 39000U) - sdu_ref_us));
	TEST_ASSERT_EQUAL_INT32(
		-1500, audio_sync_pres_err_us(target_us, (sdu_ref_us + 41500U) - sdu_ref_us));
}

void test_delivery_jitter_does_not_reach_the_presentation_error(void)
{
	/* Fifteen measurements of frames that are all rendered exactly 40 ms
	 * after their reference. The controller hands frames 0 to 7 to the
	 * application 1 ms after their reference and later ones 9 ms after it.
	 */
	const uint32_t target_us = 40000U;
	const int lag = (int)(target_us / FRAME_US);
	int32_t sum_rendered = 0;
	int32_t sum_received = 0;

	for (int n = lag; n < lag + 15; n++) {
		uint32_t arriving_ref = 1000000U + (uint32_t)n * FRAME_US;
		uint32_t arriving_rx = arriving_ref + (n < 8 ? 1000U : 9000U);
		uint32_t rendered_ref = arriving_ref - target_us;
		uint32_t rendered_rx = rendered_ref + (n - lag < 8 ? 1000U : 9000U);
		uint32_t render_ts = rendered_ref + target_us;

		sum_rendered += audio_sync_pres_err_us(target_us, render_ts - rendered_ref);
		/* Previous measurement: transport delay of the arriving frame
		 * plus the time since the rendered frame was received.
		 */
		sum_received += (int32_t)(target_us - (arriving_rx - arriving_ref)) -
				(int32_t)(render_ts - rendered_rx);
	}

	TEST_ASSERT_EQUAL_INT32(0, audio_sync_pres_adj_blks(sum_rendered / 15, BLK_US));
	/* The same perfectly aligned stream used to be moved by 2 ms. */
	TEST_ASSERT_EQUAL_INT32(-4, audio_sync_pres_adj_blks(sum_received / 15, BLK_US));
}

void test_adjustment_rounds_to_the_nearest_block(void)
{
	TEST_ASSERT_EQUAL_INT32(0, audio_sync_pres_adj_blks(0, BLK_US));
	TEST_ASSERT_EQUAL_INT32(0, audio_sync_pres_adj_blks(249, BLK_US));
	TEST_ASSERT_EQUAL_INT32(0, audio_sync_pres_adj_blks(-249, BLK_US));
	TEST_ASSERT_EQUAL_INT32(1, audio_sync_pres_adj_blks(250, BLK_US));
	TEST_ASSERT_EQUAL_INT32(-1, audio_sync_pres_adj_blks(-250, BLK_US));
	TEST_ASSERT_EQUAL_INT32(1, audio_sync_pres_adj_blks(564, BLK_US));
	TEST_ASSERT_EQUAL_INT32(20, audio_sync_pres_adj_blks(10000, BLK_US));
	TEST_ASSERT_EQUAL_INT32(-20, audio_sync_pres_adj_blks(-10030, BLK_US));
	TEST_ASSERT_EQUAL_INT32(3, audio_sync_pres_adj_blks(2600, 1000));
}

void test_frequency_correction_matches_the_regular_period_formula(void)
{
	/* The synchronizer has always used -(err * 1000) / 331 per 150 ms. */
	for (int32_t err_us = -125; err_us <= 125; err_us++) {
		TEST_ASSERT_EQUAL_INT32(-(err_us * 1000) / 331,
					audio_sync_freq_adj(err_us, 150000U));
	}

	/* The largest phase error stays inside the PLL range of +-3030 words
	 * even in the short muted period.
	 */
	TEST_ASSERT_EQUAL_INT32(-2832, audio_sync_freq_adj(125, 20000U));
	TEST_ASSERT_EQUAL_INT32(2832, audio_sync_freq_adj(-125, 20000U));
}

/* Phase after the clock ran freq_adj words off center for period_us. */
static int32_t phase_after(int32_t err_us, int32_t freq_adj, uint32_t period_us)
{
	return err_us + (int32_t)(((int64_t)freq_adj * 331LL * (int64_t)period_us) / 100000000LL);
}

void test_muted_startup_finds_the_block_phase_within_three_short_periods(void)
{
	const uint32_t fast_us = 20000U;
	const uint32_t regular_us = 150000U;
	const int32_t lock_us = 16;
	const int32_t unlock_us = 32;

	for (int32_t start_us = -250; start_us <= 250; start_us++) {
		int32_t err_us = start_us;
		int periods = 0;

		while (err_us / 2 >= lock_us || err_us / 2 <= -lock_us) {
			err_us = phase_after(err_us, audio_sync_freq_adj(err_us / 2, fast_us),
					     fast_us);
			periods++;
			TEST_ASSERT_TRUE(periods <= 2);
		}

		/* The correction that hands over to the locked state lasts a
		 * regular period and must be sized for it.
		 */
		int32_t handed_over_us = phase_after(
			err_us, audio_sync_freq_adj(err_us / 2, regular_us), regular_us);

		TEST_ASSERT_TRUE(handed_over_us / 2 < lock_us && handed_over_us / 2 > -lock_us);

		/* Sized for the short period it would throw the lock away. */
		if (err_us >= 24 || err_us <= -24) {
			int32_t overshoot_us = phase_after(
				err_us, audio_sync_freq_adj(err_us / 2, fast_us), regular_us);

			TEST_ASSERT_TRUE(overshoot_us / 2 > unlock_us ||
					 overshoot_us / 2 < -unlock_us);
		}
	}
}

extern int unity_main(void);

int main(void)
{
	(void)unity_main();
	return 0;
}
