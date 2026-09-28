#include "sensor_transport.h"
#include <assert.h>
#include <math.h>
#include <string.h>

int main(void)
{
    const uint8_t ids[] = {0, 1, 4, 6, 7, OE_SENSOR_COMPACT_IMU};
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
    /* Exhaustively verify all raw accel/gyro values survive float transport
     * conversion without losing a bit of original sensor resolution. */
    float values[9] = {0};
    uint8_t compact[24];
    for (int raw = INT16_MIN; raw <= INT16_MAX; ++raw) {
        for (unsigned axis = 0; axis < 6; ++axis)
            values[axis] = raw * (axis < 3 ? (2.0f * 9.80665f) / 32768.0f : 2000.0f / 32768.0f);
        values[6] = -123.456f; values[7] = 5.5f; values[8] = 999.25f;
        assert(oe_sensor_compact_imu((uint8_t *)values, compact));
        for (unsigned axis = 0; axis < 6; ++axis)
            assert((int16_t)(compact[2 * axis] | compact[2 * axis + 1] << 8) == raw);
        assert(memcmp(compact + 12, values + 6, 12) == 0);
    }
    values[0] = NAN;
    assert(!oe_sensor_compact_imu((uint8_t *)values, compact));
    values[0] = 100;
    assert(!oe_sensor_compact_imu((uint8_t *)values, compact));
    values[0] = 0.123456f;
    assert(!oe_sensor_compact_imu((uint8_t *)values, compact));
    return 0;
}
