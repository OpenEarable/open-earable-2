#include "sensor_transport.h"
#include "ppg_protocol.h"
#include <string.h>

#define MAX_BATCH_TIME_ERROR_US 32

/* BLE-only encoding. Acquisition records and SD/.oe samples remain 16 bytes. */
static bool compact_ppg(const uint8_t *sample, uint8_t packed[10])
{
    uint32_t v[4];
    for (unsigned i = 0; i < 4; ++i) {
        const uint8_t *p = sample + 4 * i;
        v[i] = (uint32_t)p[0] | (uint32_t)p[1] << 8 |
               (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
        if (v[i] > 0x7ffff) return false;
    }
    const ppg_compact_sample_t encoded = {
        .bits_0_31 = v[0] | (v[1] << 19),
        .bits_32_63 = (v[1] >> 13) | (v[2] << 6) | (v[3] << 25),
        .bits_64_79 = v[3] >> 7,
    };
    size_t written;
    return ppg_compact_sample_encode(&encoded, packed, 10, &written) == PROTOCOL_OK;
}

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
    uint8_t packed[10];
    if (id == 4) {
        if (!compact_ppg(sample, packed)) return false;
        sample = packed;
        width = sizeof(packed);
    }
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
