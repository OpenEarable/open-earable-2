#include <unity.h>
#include <string.h>
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
    const uint8_t ids[] = {0, 1, 6, 7};
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

extern int unity_main(void);
int main(void) { return unity_main(); }
