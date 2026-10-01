/* SPDX-License-Identifier: Apache-2.0 */
#include <limits.h>
#include <unity.h>
#include "audio_startup.h"

void setUp(void)
{
}

void test_requires_twenty_consecutive_stable_blocks(void)
{
	struct audio_startup gate = {0};

	for (int i = 0; i < 19; ++i) {
		TEST_ASSERT_FALSE(audio_startup_ready(&gate, true, true));
	}
	/* A gap, concealed frame, or loss of lock restarts the settling period. */
	TEST_ASSERT_FALSE(audio_startup_ready(&gate, true, false));
	for (int i = 0; i < 19; ++i) {
		TEST_ASSERT_FALSE(audio_startup_ready(&gate, true, true));
	}
	TEST_ASSERT_TRUE(audio_startup_ready(&gate, true, true));
}

void test_open_gate_survives_later_loss_of_lock(void)
{
	struct audio_startup gate = {.open = true};

	TEST_ASSERT_TRUE(audio_startup_ready(&gate, true, false));
}

void test_stream_stop_resets_gate_and_fade_without_stopping_i2s(void)
{
	struct audio_startup gate = {.stable_blocks = 20, .fade_frames = 240, .open = true};

	TEST_ASSERT_FALSE(audio_startup_ready(&gate, false, true));
	TEST_ASSERT_FALSE(gate.open);
	TEST_ASSERT_EQUAL_UINT32(0, gate.stable_blocks);
	TEST_ASSERT_EQUAL_UINT32(0, gate.fade_frames);
	TEST_ASSERT_FALSE(audio_startup_ready(&gate, true, true));
}

void test_fade_handles_full_scale_pcm_and_stops_at_unity_gain(void)
{
	const int32_t samples[] = {INT32_MIN, INT32_MAX, INT16_MIN, INT16_MAX, 0};
	struct audio_startup gate = {0};

	/* Five milliseconds at 48 kHz, with equal gain on both stereo channels. */
	for (uint32_t n = 1; n <= 240; ++n) {
		uint32_t gain = audio_startup_gain(&gate, 240);
		TEST_ASSERT_EQUAL_UINT32(n, gain);
		for (unsigned int i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
			TEST_ASSERT_EQUAL_INT32((int64_t)samples[i] * n / 240,
				audio_startup_scale(samples[i], gain, 240));
		}
	}
	TEST_ASSERT_EQUAL_UINT32(240, audio_startup_gain(&gate, 240));
	TEST_ASSERT_EQUAL_INT32(INT32_MIN, audio_startup_scale(INT32_MIN, 240, 240));
	TEST_ASSERT_EQUAL_INT32(INT16_MAX, audio_startup_scale(INT16_MAX, 240, 240));
}

extern int unity_main(void);

int main(void)
{
	(void)unity_main();
	return 0;
}
