#include "sensor_transport.h"
#include <assert.h>
#include <string.h>

int main(void)
{
    const uint8_t ids[] = {0, 1, 4, 6, 7};
    uint8_t sample[36];
    memset(sample, 0x5a, sizeof(sample));
    for (unsigned k = 0; k < sizeof(ids); ++k) {
        unsigned width = oe_sensor_sample_size(ids[k]);
        for (unsigned limit = 20; limit <= OE_SENSOR_PACKET_MAX; ++limit) {
            struct oe_sensor_batch b = {0};
            unsigned n = 0;
            while (oe_sensor_batch_append(&b, ids[k], sample, 100000 + n * 1953, limit)) {
                ++n;
                assert(b.len <= limit && b.data[1] == b.len - 10);
                assert(b.count == n);
                assert(oe_sensor_sample_count(ids[k], b.data[1]) == n);
                for (unsigned j = 0; j < n * width; ++j) assert(b.data[10 + j] == 0x5a);
                if (n > 1) assert((b.data[b.len - 2] | b.data[b.len - 1] << 8) == 1953);
            }
            if (n) {
                struct oe_sensor_batch old = b;
                assert(!oe_sensor_batch_append(&b, ids[k], sample, b.last_time, limit));
                assert(memcmp(&old, &b, sizeof(b)) == 0);
            }
        }
    }
    struct oe_sensor_batch b = {0};
    assert(oe_sensor_batch_append(&b, 7, sample, 0, 244));
    assert(!oe_sensor_batch_append(&b, 7, sample, 65536, 244));
    assert(oe_sensor_batch_append(&b, 7, sample, 65535, 244));
    assert(!oe_sensor_batch_append(&b, 7, sample, 65536, 244));
    assert(!oe_sensor_batch_append(&b, 4, sample, 131070, 244));
    assert(!oe_sensor_sample_count(4, 17));
    assert(!oe_sensor_sample_count(99, 16));
    /* Only existing sensor IDs and byte-for-byte sample representations. */
    assert(!oe_sensor_sample_size(0x80));
    b = (struct oe_sensor_batch){0};
    const uint64_t timestamp = UINT64_C(0x0102030405060708);
    assert(oe_sensor_batch_append(&b, 0, sample, timestamp, 244));
    assert(b.data[0] == 0 && b.data[1] == 36 && b.len == 46);
    for (unsigned i = 0; i < 8; ++i)
        assert(b.data[2 + i] == (uint8_t)(timestamp >> (8 * i)));
    assert(memcmp(b.data + 10, sample, 36) == 0);
    assert(oe_sensor_batch_append(&b, 0, sample, timestamp + 10000, 244));
    assert(b.data[1] == 74 && b.len == 84);
    assert(memcmp(b.data + 46, sample, 36) == 0);
    assert((b.data[82] | b.data[83] << 8) == 10000);
    return 0;
}
