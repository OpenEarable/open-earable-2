// SPDX-License-Identifier: MIT

#include <unity.h>

#include <cstdint>
#include <cstring>

#include "SensorComponent.h"

static SensorComponent components[] = {
    {.name = "x", .unit = "m/s2", .parseType = PARSE_TYPE_FLOAT},
    {.name = "quality", .unit = "%", .parseType = PARSE_TYPE_UINT8},
};

static SensorComponentGroup group = {
    .name = "acceleration",
    .componentCount = sizeof(components) / sizeof(components[0]),
    .components = components,
};

void setUp(void)
{
}

extern "C" int unity_c_suite_teardown(int failures) asm("test_suiteTearDown");

int test_suiteTearDown(int failures)
{
    return unity_c_suite_teardown(failures);
}

void test_size_matches_serialized_size(void)
{
    char buffer[64] = {0};

    const size_t expected_size = getSensorComponentGroupSize(&group);
    const ssize_t bytes_written = serializeSensorComponentGroup(&group, buffer, sizeof(buffer));

    TEST_ASSERT_EQUAL_INT64(static_cast<ssize_t>(expected_size), bytes_written);
}

void test_serializes_all_component_fields(void)
{
    char buffer[64] = {0};
    const char *cursor = buffer;

    const ssize_t bytes_written = serializeSensorComponentGroup(&group, buffer, sizeof(buffer));

    TEST_ASSERT_GREATER_THAN_INT64(0, bytes_written);

    for (size_t i = 0; i < group.componentCount; ++i) {
        const SensorComponent *component = &components[i];
        uint8_t length;

        TEST_ASSERT_EQUAL_UINT8(component->parseType, static_cast<uint8_t>(*cursor++));

        length = static_cast<uint8_t>(*cursor++);
        TEST_ASSERT_EQUAL_UINT8(strlen(group.name), length);
        TEST_ASSERT_EQUAL_MEMORY(group.name, cursor, length);
        cursor += length;

        length = static_cast<uint8_t>(*cursor++);
        TEST_ASSERT_EQUAL_UINT8(strlen(component->name), length);
        TEST_ASSERT_EQUAL_MEMORY(component->name, cursor, length);
        cursor += length;

        length = static_cast<uint8_t>(*cursor++);
        TEST_ASSERT_EQUAL_UINT8(strlen(component->unit), length);
        TEST_ASSERT_EQUAL_MEMORY(component->unit, cursor, length);
        cursor += length;
    }

    TEST_ASSERT_EQUAL_INT64(bytes_written, cursor - buffer);
}

void test_accepts_exactly_sized_buffer(void)
{
    char buffer[64] = {0};
    const size_t required_size = getSensorComponentGroupSize(&group);

    const ssize_t bytes_written = serializeSensorComponentGroup(&group, buffer, required_size);

    TEST_ASSERT_EQUAL_INT64(static_cast<ssize_t>(required_size), bytes_written);
}

void test_rejects_buffer_one_byte_too_small(void)
{
    char buffer[64];
    const size_t required_size = getSensorComponentGroupSize(&group);

    memset(buffer, 0x5a, sizeof(buffer));
    const ssize_t result = serializeSensorComponentGroup(&group, buffer, required_size - 1);

    TEST_ASSERT_EQUAL_INT64(-1, result);
    for (const char byte : buffer) {
        TEST_ASSERT_EQUAL_HEX8(0x5a, byte);
    }
}

void test_empty_group_has_empty_serialization(void)
{
    SensorComponentGroup empty_group = {
        .name = "empty",
        .componentCount = 0,
        .components = nullptr,
    };
    char buffer = 0x5a;

    TEST_ASSERT_EQUAL_UINT64(0, getSensorComponentGroupSize(&empty_group));
    TEST_ASSERT_EQUAL_INT64(0, serializeSensorComponentGroup(&empty_group, &buffer, 0));
    TEST_ASSERT_EQUAL_HEX8(0x5a, buffer);
}

/* Zephyr reserves nonzero returns from main, so ignore Unity's return value. */
extern int unity_main(void);

int main(void)
{
    (void)unity_main();
    return 0;
}
