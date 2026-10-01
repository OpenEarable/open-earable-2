/* SPDX-License-Identifier: Apache-2.0 */
#include <limits.h>
#include <unity.h>
#include "audio_sdu_timing.h"

void setUp(void) {}

void test_requires_anchor_but_accepts_valid_zero_timestamp(void)
{
	struct audio_sdu_timing anchor = {0};
	uint32_t resolved = 123;

	TEST_ASSERT_FALSE(audio_sdu_timing_resolve(&anchor, false, 1, 0, 0, 10000, &resolved));
	TEST_ASSERT_EQUAL_UINT32(123, resolved);
	TEST_ASSERT_TRUE(audio_sdu_timing_resolve(&anchor, true, 1, 0, 0, 10000, &resolved));
	TEST_ASSERT_EQUAL_UINT32(0, resolved);
}

void test_conceals_gaps_without_extending_the_anchor(void)
{
	struct audio_sdu_timing anchor = {100000, 500, 40, true};
	uint32_t resolved;

	TEST_ASSERT_TRUE(audio_sdu_timing_resolve(&anchor, false, 41, 0, 510, 10000, &resolved));
	TEST_ASSERT_EQUAL_UINT32(110000, resolved);
	TEST_ASSERT_TRUE(audio_sdu_timing_resolve(&anchor, false, 50, 0, 600, 10000, &resolved));
	TEST_ASSERT_EQUAL_UINT32(200000, resolved);
	TEST_ASSERT_EQUAL_UINT16(40, anchor.sequence);
	TEST_ASSERT_FALSE(audio_sdu_timing_resolve(&anchor, false, 51, 0, 610, 10000, &resolved));
}

void test_rejects_stale_duplicate_backward_and_reset_references(void)
{
	struct audio_sdu_timing anchor = {100000, 500, 40, true};
	uint32_t resolved;

	TEST_ASSERT_FALSE(audio_sdu_timing_resolve(&anchor, false, 41, 0, 701, 10000, &resolved));
	TEST_ASSERT_FALSE(audio_sdu_timing_resolve(&anchor, false, 40, 0, 510, 10000, &resolved));
	TEST_ASSERT_FALSE(audio_sdu_timing_resolve(&anchor, false, 39, 0, 510, 10000, &resolved));
	TEST_ASSERT_FALSE(audio_sdu_timing_resolve(&anchor, false, 41, 0, 510, 0, &resolved));
	anchor = (struct audio_sdu_timing){0};
	TEST_ASSERT_FALSE(audio_sdu_timing_resolve(&anchor, false, 41, 0, 510, 10000, &resolved));
}

void test_handles_sequence_timestamp_and_uptime_wraps(void)
{
	struct audio_sdu_timing anchor = {UINT32_MAX - 9999U, UINT32_MAX - 9U,
					UINT16_MAX, true};
	uint32_t resolved;

	TEST_ASSERT_TRUE(audio_sdu_timing_resolve(&anchor, false, 0, 0, 0, 10000, &resolved));
	TEST_ASSERT_EQUAL_UINT32(0, resolved);
	TEST_ASSERT_TRUE(audio_sdu_timing_resolve(&anchor, true, 1, 12000, 10, 7500, &resolved));
	TEST_ASSERT_TRUE(audio_sdu_timing_resolve(&anchor, false, 2, 0, 18, 7500, &resolved));
	TEST_ASSERT_EQUAL_UINT32(19500, resolved);
}
