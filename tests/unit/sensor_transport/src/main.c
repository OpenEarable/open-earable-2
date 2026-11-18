#include <unity.h>
#include <string.h>
#include <math.h>
#include "sensor_transport.h"

void setUp(void) {}

void test_compact_ppg_preserves_full_scale_and_header(void)
{
    struct oe_sensor_batch batch = {0};
    const uint8_t sample[16] = {255,255,7,0,255,255,7,0,255,255,7,0,255,255,7,0};
    const uint8_t expected[20] = {4,10,1,2,3,4,5,6,7,0,
        255,255,255,255,255,255,255,255,255,15};
    TEST_ASSERT_TRUE(oe_sensor_batch_append(&batch, 4, sample, 0x07060504030201ULL, 20));
    TEST_ASSERT_EQUAL_UINT(20, batch.len);
    TEST_ASSERT_EQUAL_MEMORY(expected, batch.data, sizeof(expected));
    /* Internal/SD layout stays four uint32 channels, including batched records. */
    TEST_ASSERT_EQUAL_UINT(16, oe_sensor_sample_size(4));
    TEST_ASSERT_EQUAL_UINT(1, oe_sensor_sample_count(4, 16));
    TEST_ASSERT_EQUAL_UINT(3, oe_sensor_sample_count(4, 50));
    TEST_ASSERT_EQUAL_UINT(0, oe_sensor_sample_count(4, 10));
}

void test_compact_ppg_fits_twenty_three_samples_without_changing_timestamps(void)
{
    struct oe_sensor_batch batch = {0};
    const uint8_t sample[16] = {1,0,0,0,2,0,0,0,3,0,0,0,4,0,0,0};
    const uint8_t packed[10] = {1,0,16,0,192,0,0,8,0,0};
    for (unsigned i = 0; i < 23; ++i) {
        TEST_ASSERT_TRUE(oe_sensor_batch_append(&batch, 4, sample, 100000 + 1953 * i, 244));
        TEST_ASSERT_EQUAL_MEMORY(packed, batch.data + 10 + 10 * i, 10);
    }
    TEST_ASSERT_EQUAL_UINT(23, batch.count);
    TEST_ASSERT_EQUAL_UINT(242, batch.len);
    TEST_ASSERT_EQUAL_UINT(232, batch.data[1]);
    TEST_ASSERT_EQUAL_UINT(1953 & 255, batch.data[240]);
    TEST_ASSERT_EQUAL_UINT(1953 >> 8, batch.data[241]);
    struct oe_sensor_batch before = batch;
    TEST_ASSERT_FALSE(oe_sensor_batch_append(&batch, 4, sample, 144919, 244));
    TEST_ASSERT_EQUAL_MEMORY(&before, &batch, sizeof(batch));
}

void test_invalid_ppg_is_rejected_instead_of_losing_high_bits(void)
{
    struct oe_sensor_batch batch = {0};
    const uint8_t sample[16] = {0,0,8,0};
    TEST_ASSERT_FALSE(oe_sensor_batch_append(&batch, 4, sample, 100, 244));
    TEST_ASSERT_EQUAL_UINT(0, batch.count);
}

void test_other_sensor_samples_keep_the_existing_wire_bytes(void)
{
    const uint8_t ids[] = {1, 6, 7};
    uint8_t sample[36];
    for (unsigned i = 0; i < sizeof(sample); ++i) sample[i] = i;
    for (unsigned n = 0; n < sizeof(ids); ++n) {
        struct oe_sensor_batch batch = {0};
        unsigned width = oe_sensor_sample_size(ids[n]);
        TEST_ASSERT_TRUE(oe_sensor_batch_append(&batch, ids[n], sample, 123456, 244));
        TEST_ASSERT_EQUAL_UINT(width + 10, batch.len);
        TEST_ASSERT_EQUAL_MEMORY(sample, batch.data + 10, width);
    }
}

void test_compact_imu_preserves_every_raw_value_and_sd_sample(void)
{
    for (int raw = INT16_MIN; raw <= INT16_MAX; ++raw) {
        const float a = raw * ((2.0f * 9.80665f) / 32768.0f);
        const float g = raw * (2000.0f / 32768.0f);
        const float values[9] = {a,a,a,g,g,g,12.345f,-67.89f,-0.0f};
        uint8_t sample[36];
        memcpy(sample, values, sizeof(sample));
        struct oe_sensor_batch batch = {0};
        TEST_ASSERT_TRUE(oe_sensor_batch_append(&batch, 0, sample, 123, 244));
        TEST_ASSERT_EQUAL_UINT(34, batch.len);
        for (unsigned axis = 0; axis < 6; ++axis) {
            const uint8_t *v = batch.data + 10 + axis * 2;
            TEST_ASSERT_EQUAL_UINT16((uint16_t)raw, (uint16_t)(v[0] | v[1] << 8));
        }
        TEST_ASSERT_EQUAL_MEMORY(sample + 24, batch.data + 22, 12);
        TEST_ASSERT_EQUAL_MEMORY(values, sample, sizeof(sample));
    }
    TEST_ASSERT_EQUAL_UINT(36, oe_sensor_sample_size(0));
    TEST_ASSERT_EQUAL_UINT(1, oe_sensor_sample_count(0, 36));
    TEST_ASSERT_EQUAL_UINT(3, oe_sensor_sample_count(0, 110));
    TEST_ASSERT_EQUAL_UINT(0, oe_sensor_sample_count(0, 24));
}

void test_compact_imu_fits_nine_samples_and_preserves_timestamps(void)
{
    const float sample[9] = {0};
    struct oe_sensor_batch batch = {0};
    for (unsigned i = 0; i < 9; ++i)
        TEST_ASSERT_TRUE(oe_sensor_batch_append(&batch, 0, (const uint8_t *)sample,
                                               123456 + 10000 * i, 244));
    TEST_ASSERT_EQUAL_UINT(9, batch.count);
    TEST_ASSERT_EQUAL_UINT(228, batch.len);
    TEST_ASSERT_EQUAL_UINT(218, batch.data[1]);
    TEST_ASSERT_EQUAL_UINT(10000 & 255, batch.data[226]);
    TEST_ASSERT_EQUAL_UINT(10000 >> 8, batch.data[227]);
    struct oe_sensor_batch before = batch;
    TEST_ASSERT_FALSE(oe_sensor_batch_append(&batch, 0, (const uint8_t *)sample, 213456, 244));
    TEST_ASSERT_EQUAL_MEMORY(&before, &batch, sizeof(batch));
}

void test_compact_imu_rejects_values_that_cannot_be_encoded_losslessly(void)
{
    const float invalid[] = {NAN, INFINITY, -INFINITY, 100000.0f, 0.001f, -0.0f};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        struct oe_sensor_batch batch = {0};
        float sample[9] = {0};
        sample[i % 6] = invalid[i];
        TEST_ASSERT_FALSE(oe_sensor_batch_append(&batch, 0, (const uint8_t *)sample, 123, 244));
        TEST_ASSERT_EQUAL_UINT(0, batch.count);
    }
}

extern int unity_main(void);
int main(void) { return unity_main(); }
