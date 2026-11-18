// SPDX-License-Identifier: MIT

#include <unity.h>

#include "RingBuffer.h"

using arduino::RingBufferN;

void setUp(void)
{
}

void test_new_buffer_is_empty(void)
{
    RingBufferN<4> buffer;

    TEST_ASSERT_EQUAL_INT(0, buffer.available());
    TEST_ASSERT_EQUAL_INT(4, buffer.availableForStore());
    TEST_ASSERT_EQUAL_INT(-1, buffer.peek());
    TEST_ASSERT_EQUAL_INT(-1, buffer.read_char());
    TEST_ASSERT_FALSE(buffer.isFull());
}

void test_reads_values_in_fifo_order_across_wraparound(void)
{
    RingBufferN<4> buffer;

    buffer.store_char(1);
    buffer.store_char(2);
    buffer.store_char(3);
    TEST_ASSERT_EQUAL_INT(1, buffer.read_char());
    TEST_ASSERT_EQUAL_INT(2, buffer.read_char());

    buffer.store_char(4);
    buffer.store_char(5);

    TEST_ASSERT_EQUAL_INT(3, buffer.read_char());
    TEST_ASSERT_EQUAL_INT(4, buffer.read_char());
    TEST_ASSERT_EQUAL_INT(5, buffer.read_char());
    TEST_ASSERT_EQUAL_INT(-1, buffer.read_char());
}

void test_full_buffer_rejects_new_values(void)
{
    RingBufferN<4> buffer;

    buffer.store_char(1);
    buffer.store_char(2);
    buffer.store_char(3);
    buffer.store_char(4);

    TEST_ASSERT_TRUE(buffer.isFull());
    TEST_ASSERT_EQUAL_INT(0, buffer.availableForStore());

    buffer.store_char(5);

    TEST_ASSERT_EQUAL_INT(4, buffer.available());
    TEST_ASSERT_EQUAL_INT(1, buffer.read_char());
    TEST_ASSERT_EQUAL_INT(2, buffer.read_char());
    TEST_ASSERT_EQUAL_INT(3, buffer.read_char());
    TEST_ASSERT_EQUAL_INT(4, buffer.read_char());
}

void test_peek_does_not_remove_value(void)
{
    RingBufferN<4> buffer;

    buffer.store_char(42);

    TEST_ASSERT_EQUAL_INT(42, buffer.peek());
    TEST_ASSERT_EQUAL_INT(1, buffer.available());
    TEST_ASSERT_EQUAL_INT(42, buffer.read_char());
}

void test_clear_discards_values_and_restores_capacity(void)
{
    RingBufferN<4> buffer;

    buffer.store_char(1);
    buffer.store_char(2);
    buffer.clear();

    TEST_ASSERT_EQUAL_INT(0, buffer.available());
    TEST_ASSERT_EQUAL_INT(4, buffer.availableForStore());
    TEST_ASSERT_EQUAL_INT(-1, buffer.read_char());
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
