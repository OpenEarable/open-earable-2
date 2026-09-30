/* SPDX-License-Identifier: LicenseRef-Nordic-5-Clause */
#ifndef AUDIO_SYNC_CLOCK_H_
#define AUDIO_SYNC_CLOCK_H_

#include <stdint.h>

/* TIMER and RTC are anchored by the same RTC tick event. The captured frame
 * may precede that anchor (including across either counter's wrap). Keeping
 * the high-frequency offset signed avoids inventing a 30.5 us tick at edges.
 */
static inline uint32_t audio_sync_clock_from_anchor(uint32_t rtc_ticks,
                                                    uint32_t rtc_overflows,
                                                    uint32_t anchor_us,
                                                    uint32_t captured_us)
{
    uint64_t rtc_us = (uint64_t)rtc_ticks * 1000000U / 32768U +
                      (uint64_t)rtc_overflows * 512000000U;
    return (uint32_t)((int64_t)rtc_us + (int32_t)(captured_us - anchor_us));
}
#endif
