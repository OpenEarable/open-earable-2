#ifndef AUDIO_SDU_TIMING_H
#define AUDIO_SDU_TIMING_H

#include <stdbool.h>
#include <stdint.h>

/** Last controller-provided timestamp for one receive stream. */
struct audio_sdu_timing {
	uint32_t reference;
	uint32_t received;
	uint16_t sequence;
	bool valid;
};

/** Resolve an SDU reference without treating absent ISO timestamps as zero.
 *
 * A valid timestamp (including zero at wraparound) replaces the anchor. For
 * lost SDUs, the sequence number permits at most 100 ms of concealment from
 * that anchor. Reject stale anchors across a stop/restart, duplicates and
 * backwards sequence numbers. All times share the local controller epoch.
 */
static inline bool audio_sdu_timing_resolve(struct audio_sdu_timing *anchor,
					  bool timestamp_valid, uint16_t sequence,
					  uint32_t reference, uint32_t received,
					  uint32_t frame_us, uint32_t *resolved)
{
	if (timestamp_valid) {
		*anchor = (struct audio_sdu_timing){reference, received, sequence, true};
		*resolved = reference;
		return true;
	}

	uint16_t frames = (uint16_t)(sequence - anchor->sequence);
	uint32_t age = received - anchor->received;
	if (!anchor->valid || frames == 0 || frames > 10 || age > 200000U) {
		return false;
	}
	*resolved = anchor->reference + frames * frame_us;
	return true;
}

#endif
