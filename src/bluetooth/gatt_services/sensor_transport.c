#include "sensor_transport.h"
#include <string.h>

#define MAX_BATCH_TIME_ERROR_US 32

unsigned oe_sensor_sample_size(uint8_t id)
{
    switch (id) {
    case 0: return 36;
    case 1: return 8;
    case 4: return 16;
    case 6: return 4;
    case 7: return 6;
    default: return 0;
    }
}

unsigned oe_sensor_sample_count(uint8_t id, unsigned size)
{
    unsigned width = oe_sensor_sample_size(id);
    if (!width) return 0;
    if (size == width) return 1;
    return size >= width + 2 && (size - 2) % width == 0 ? (size - 2) / width : 0;
}

bool oe_sensor_batch_append(struct oe_sensor_batch *b, uint8_t id,
                            const uint8_t *sample, uint64_t time, unsigned limit)
{
    unsigned width = oe_sensor_sample_size(id);
    if (!width || !sample || limit > OE_SENSOR_PACKET_MAX) return false;
    unsigned len = 10 + (b->count + 1) * width + (b->count ? 2 : 0);
    if (len > limit) return false;
    if (b->count) {
        if (b->data[0] != id || time <= b->last_time) return false;
        uint64_t delta = time - b->last_time;
        if (delta > UINT16_MAX) return false;
        if (b->count == 1) {
            b->period = (uint16_t)delta;
        } else {
            /* Keep the period fixed and bound each reconstructed timestamp's
             * error against the packet anchor, so errors cannot accumulate. */
            uint64_t first = 0;
            for (unsigned i = 0; i < 8; ++i)
                first |= (uint64_t)b->data[2 + i] << (8 * i);
            uint64_t span = (uint64_t)b->count * b->period;
            if (first > UINT64_MAX - span) return false;
            uint64_t expected = first + span;
            uint64_t error = time > expected ? time - expected : expected - time;
            if (error > MAX_BATCH_TIME_ERROR_US) return false;
        }
    } else {
        b->data[0] = id;
        for (unsigned i = 0; i < 8; ++i) b->data[2 + i] = time >> (8 * i);
        b->period = 0;
    }
    memcpy(b->data + 10 + b->count * width, sample, width);
    b->count++;
    b->len = len;
    b->data[1] = len - 10;
    b->last_time = time;
    if (b->count > 1) {
        b->data[len - 2] = b->period;
        b->data[len - 1] = b->period >> 8;
    }
    return true;
}
