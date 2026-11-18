/* SPDX-License-Identifier: Apache-2.0 */
#ifndef AUDIO_SDU_TIMING_H_
#define AUDIO_SDU_TIMING_H_

#include <stdbool.h>
#include <stdint.h>

struct audio_sdu_timing {
	uint32_t reference;
	uint32_t received_ms;
	uint16_t sequence;
	bool valid;
};

/* A missing ISO timestamp is not zero. Conceal at most ten frames from a
 * recent timestamp, without promoting an estimate to a new anchor. Reset
 * this state at each stream start/stop. Unsigned differences handle wraps.
 */
static inline bool audio_sdu_timing_resolve(struct audio_sdu_timing *anchor,
		bool timestamp_valid, uint16_t sequence, uint32_t reference,
		uint32_t now_ms, uint32_t frame_us, uint32_t *resolved)
{
	if (timestamp_valid) {
		*anchor = (struct audio_sdu_timing){reference, now_ms, sequence, true};
		*resolved = reference;
		return true;
	}

	uint16_t frames = (uint16_t)(sequence - anchor->sequence);
	if (!anchor->valid || !frame_us || frames == 0 || frames > 10 ||
	    (uint32_t)(now_ms - anchor->received_ms) > 200U) {
		return false;
	}
	*resolved = anchor->reference + frames * frame_us;
	return true;
}

#endif
