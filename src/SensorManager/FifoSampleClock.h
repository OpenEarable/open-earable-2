#ifndef FIFO_SAMPLE_CLOCK_H
#define FIFO_SAMPLE_CLOCK_H

#include <stdint.h>
#include <algorithm>

// FIFO polling times are noisy observations of an independently clocked stream.
// Advance by samples, retaining fractional microseconds, and track clock error
// gradually instead of independently backdating each FIFO read.
class FifoSampleClock {
public:
    void reset() { initialized = false; }

    void begin(uint64_t read_us, uint32_t count, double nominal_us,
               uint32_t capacity, double accuracy_percent) {
        const double span = count * nominal_us;
        if (!initialized || read_us < previous_read) {
            anchor(read_us, span, nominal_us);
        } else {
            const double observed = static_cast<double>(read_us - epoch) - span;
            const double error = observed - next;
            // A pause longer than the FIFO can retain may have lost samples.
            // Re-anchor forward, preserving the gap instead of filling it in.
            if (read_us - previous_read > capacity * nominal_us &&
                error > 2 * nominal_us) {
                anchor(read_us, span, nominal_us);
            } else {
                const double correction = error / std::max<double>(count, 100000.0 / nominal_us);
                const double limit = nominal_us * accuracy_percent / 100.0;
                period_us = nominal_us + std::max(-limit, std::min(limit, correction));
            }
        }
        start = next;
        next += count * period_us;
        previous_read = read_us;
        initialized = true;
    }

    uint64_t timestamp(uint32_t index) const {
        return epoch + static_cast<uint64_t>(start + index * period_us);
    }

    double period() const { return period_us; }

private:
    void anchor(uint64_t read_us, double span, double nominal_us) {
        const uint64_t back = static_cast<uint64_t>(span);
        epoch = read_us > back ? read_us - back : 0;
        next = 0;
        period_us = nominal_us;
    }

    bool initialized = false;
    uint64_t epoch = 0;
    uint64_t previous_read = 0;
    double next = 0;
    double start = 0;
    double period_us = 0;
};

#endif
