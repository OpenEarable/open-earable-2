#include <unity.h>
#include "FifoTimestamps.h"

void setUp(void) {}

void test_nominal_batch_spacing(void) {
    FifoTimestamps clock;
    TEST_ASSERT_EQUAL_UINT64(910000, clock.begin(1000000, 10, 10000));
    TEST_ASSERT_EQUAL_UINT32(10000, clock.period());
    TEST_ASSERT_EQUAL_UINT64(1010000, clock.begin(1100000, 10, 10000));
}

void test_clock_drift_and_jitter_do_not_overlap_batches(void) {
    FifoTimestamps clock;
    uint64_t previous = 0;
    uint64_t now = 1000000;
    for (unsigned i = 0; i < 10000; ++i) {
        const unsigned count = i % 3 == 0 ? 11 : 10;
        now += 100000 + (i % 2 ? 83 : 0);
        const auto first = clock.begin(now, count, 10000);
        TEST_ASSERT_TRUE(first > previous);
        previous = first + (count - 1) * clock.period();
        TEST_ASSERT_TRUE(previous <= now);
    }
}

void test_read_delay_preserves_real_gap(void) {
    FifoTimestamps clock;
    clock.begin(1000000, 10, 10000);
    TEST_ASSERT_EQUAL_UINT64(1310000, clock.begin(1400000, 10, 10000));
}

void test_empty_read_does_not_advance_clock(void) {
    FifoTimestamps clock;
    clock.begin(1000000, 10, 10000);
    clock.begin(5000000, 0, 10000);
    TEST_ASSERT_EQUAL_UINT64(1010000, clock.begin(1100000, 10, 10000));
}

void test_reset_and_backward_time_correction(void) {
    FifoTimestamps clock;
    clock.begin(1000000, 10, 10000);
    TEST_ASSERT_EQUAL_UINT64(1000001, clock.begin(900000, 10, 10000));
    clock.reset();
    TEST_ASSERT_EQUAL_UINT64(810000, clock.begin(900000, 10, 10000));
}

void test_low_rate_period_is_not_truncated_to_16_bits(void) {
    FifoTimestamps clock;
    TEST_ASSERT_EQUAL_UINT64(875000, clock.begin(1000000, 2, 125000));
    TEST_ASSERT_EQUAL_UINT32(125000, clock.period());
}


extern "C" int unity_c_suite_teardown(int failures) asm("test_suiteTearDown");

int test_suiteTearDown(int failures)
{
    return unity_c_suite_teardown(failures);
}

extern int unity_main(void);

int main(void)
{
    (void)unity_main();
    return 0;
}
