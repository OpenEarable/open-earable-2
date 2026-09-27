#ifndef FIFO_TIMESTAMPS_H
#define FIFO_TIMESTAMPS_H

#include <stdint.h>

// FIFO read time is an upper bound, not an exact sample timestamp. Keep
// adjacent batches disjoint despite polling jitter and sensor clock drift.
// If nominal spacing would overlap the last batch, distribute this batch
// over the elapsed interval instead of moving samples into the future.
class FifoTimestamps {
public:
    void reset() { last_us = 0; }

    uint64_t begin(uint64_t read_us, uint32_t count, uint32_t nominal_period_us) {
        period_us = nominal_period_us ? nominal_period_us : 1;
        if (count == 0) return read_us;
        if (last_us && (read_us <= last_us || read_us - last_us < count)) {
            // A host clock correction can move the wall clock backwards.
            read_us = last_us + count;
        }
        if (last_us && (read_us - last_us) / count < period_us) {
            period_us = static_cast<uint32_t>((read_us - last_us) / count);
        }
        uint64_t span = static_cast<uint64_t>(count - 1) * period_us;
        if (span > read_us) span = read_us;
        last_us = read_us;
        return read_us - span;
    }

    uint32_t period() const { return period_us; }

private:
    uint64_t last_us = 0;
    uint32_t period_us = 1;
};

#endif
