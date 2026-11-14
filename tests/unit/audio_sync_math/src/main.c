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

extern int unity_main(void);

int main(void)
{
	(void)unity_main();
	return 0;
}
