/* SPDX-License-Identifier: Apache-2.0 */
#include <unity.h>
#include "csip_identity.h"

void setUp(void)
{
}

void test_fota_preserves_legacy_paired_sirk_byte_for_byte(void)
{
	/* Golden 2.2.9 wire value: ASCII hex, including eight zero pad bytes. */
	const uint8_t expected[16] = {'B', 'A', '4', '5', '3', '3', '7', 'F'};
	uint8_t left[16];
	uint8_t right[16];

	memset(left, 0xAA, sizeof(left));
	memset(right, 0x55, sizeof(right));
	openearable_csip_identity_encode(0xBA45337F, UINT64_C(0x1234567801020304), left);
	openearable_csip_identity_encode(0xBA45337F, UINT64_C(0x87654321BB47307B), right);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, left, sizeof(expected));
	TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, right, sizeof(expected));
}

void test_unpaired_sirk_uses_low_chip_id_with_leading_zeroes(void)
{
	const uint8_t expected[16] = {'0', '0', 'A', 'B', '0', '1', 'E', 'F'};
	uint8_t actual[16];

	openearable_csip_identity_encode(UINT32_MAX, UINT64_C(0xDEADBEEF00AB01EF), actual);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(expected));
}

void test_programmed_zero_is_not_treated_as_erased_uicr(void)
{
	const uint8_t expected[16] = {'0', '0', '0', '0', '0', '0', '0', '0'};
	uint8_t actual[16];

	openearable_csip_identity_encode(0, UINT64_C(0x123456789ABCDEF0), actual);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(expected));
}

void test_standalone_identity_keeps_existing_chip_id_sirk(void)
{
	const uint8_t expected[16] = {'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
	uint8_t before[16];
	uint8_t after[16];

	/* Standalone startup persists the low chip ID into UICR. */
	openearable_csip_identity_encode(UINT32_MAX, UINT64_C(0x1234567889ABCDEF), before);
	openearable_csip_identity_encode(0x89ABCDEF, UINT64_C(0x1234567889ABCDEF), after);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, before, sizeof(expected));
	TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, after, sizeof(expected));
}

void test_peer_pairing_xor_matches_existing_wire_format(void)
{
	const uint32_t left_id = 0x10203040;
	const uint32_t right_id = 0xB0E0F00D;
	const uint8_t expected[16] = {'A', '0', 'C', '0', 'C', '0', '4', 'D'};
	uint8_t left[16];
	uint8_t right[16];

	openearable_csip_identity_encode(left_id ^ right_id, left_id, left);
	openearable_csip_identity_encode(right_id ^ left_id, right_id, right);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, left, sizeof(expected));
	TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, right, sizeof(expected));
}

void test_different_pairs_do_not_share_the_demo_identity(void)
{
	const uint8_t expected[16] = {'F', 'F', 'F', 'F', 'F', 'F', 'F', 'E'};
	uint8_t first[16];
	uint8_t second[16];

	openearable_csip_identity_encode(0xFFFFFFFE, 0, first);
	openearable_csip_identity_encode(0x00000001, 0, second);
	TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, first, sizeof(expected));
	TEST_ASSERT_NOT_EQUAL(0, memcmp(first, second, sizeof(first)));
}

extern int unity_main(void);

int main(void)
{
	return unity_main();
}
