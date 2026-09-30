/* SPDX-License-Identifier: Apache-2.0 */
#ifndef AUDIO_STARTUP_H_
#define AUDIO_STARTUP_H_

#include <stdbool.h>
#include <stdint.h>

/* Called once per 1 ms I2S block. Once opened, normal presentation checks
 * must not mute playback again. A stream stop explicitly resets the gate.
 */
struct audio_startup {
	uint32_t stable_blocks;
	uint32_t fade_frames;
	bool open;
};

static inline bool audio_startup_ready(struct audio_startup *startup, bool stable)
{
	if (!startup->open) {
		startup->stable_blocks = stable ? startup->stable_blocks + 1U : 0U;
		startup->open = startup->stable_blocks >= 20U;
	}
	return startup->open;
}

/* Advance once per stereo sample frame, using the same gain for both
 * channels. int64_t also makes scaling INT32_MIN well-defined.
 */
static inline uint32_t audio_startup_gain(struct audio_startup *startup, uint32_t fade_frames)
{
	if (startup->fade_frames < fade_frames) {
		startup->fade_frames++;
	}
	return startup->fade_frames;
}

static inline int32_t audio_startup_scale(int32_t sample, uint32_t gain, uint32_t full_gain)
{
	return (int32_t)(((int64_t)sample * gain) / full_gain);
}

#endif /* AUDIO_STARTUP_H_ */
