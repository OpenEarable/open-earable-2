/* SPDX-License-Identifier: Apache-2.0 */
#ifndef AUDIO_STARTUP_H_
#define AUDIO_STARTUP_H_

#include <stdbool.h>
#include <stdint.h>

struct audio_startup {
	uint32_t stable_us;
	uint32_t fade_frames;
	bool open;
};

/* Require 20 ms of consecutive valid, synchronized PCM at startup. Measure
 * time rather than calls: the SDK can change the I2S block duration. Once open,
 * later loss of lock must not mute music again.
 * A stopped Bluetooth stream resets the gate even if recording keeps I2S on.
 */
static inline bool audio_startup_ready(struct audio_startup *startup,
				       bool streaming, bool stable, uint32_t block_us)
{
	if (!streaming) {
		*startup = (struct audio_startup){0};
		return false;
	}
	if (!startup->open) {
		startup->stable_us = stable ? startup->stable_us + block_us : 0U;
		startup->open = startup->stable_us >= 20000U;
	}
	return startup->open;
}

/* Advance once per stereo sample frame; use the same gain for both channels. */
static inline uint32_t audio_startup_gain(struct audio_startup *startup, uint32_t fade_frames)
{
	if (startup->fade_frames < fade_frames) {
		startup->fade_frames++;
	}
	return startup->fade_frames;
}

/* 64-bit multiplication keeps full-scale 16- and 32-bit PCM within range. */
static inline int32_t audio_startup_scale(int32_t sample, uint32_t gain, uint32_t full_gain)
{
	return (int32_t)(((int64_t)sample * gain) / full_gain);
}

#endif /* AUDIO_STARTUP_H_ */
