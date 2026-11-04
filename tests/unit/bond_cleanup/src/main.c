/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <string.h>
#include <unity.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/settings/settings.h>

#include "settings.h"
#include "bt_mgmt_bond_storage.h"

static struct bt_bond_info bonds[2];
static bool stored[2];
static unsigned int ram_count;
static unsigned int delete_calls;
static unsigned int unpair_calls;
static unsigned int failed_delete_call;
static int unpair_error;
static int load_error;
static bool keep_stored_keys;
static bool keep_ram_keys;
static bool orphaned_key;

void setUp(void)
{
	memset(bonds, 0, sizeof(bonds));
	bonds[0].addr.a.val[0] = 1;
	bonds[1].addr.a.val[0] = 2;
	stored[0] = stored[1] = true;
	ram_count = 2;
	delete_calls = unpair_calls = failed_delete_call = 0;
	unpair_error = load_error = 0;
	keep_stored_keys = keep_ram_keys = orphaned_key = false;
}

void bt_foreach_bond(uint8_t id, void (*func)(const struct bt_bond_info *, void *),
		     void *data)
{
	TEST_ASSERT_EQUAL_UINT8(BT_ID_DEFAULT, id);
	for (unsigned int i = 0; i < ram_count; i++) {
		func(&bonds[i], data);
	}
}

int bt_settings_delete_keys(uint8_t id, const bt_addr_le_t *addr)
{
	TEST_ASSERT_EQUAL_UINT8(BT_ID_DEFAULT, id);
	delete_calls++;
	if (delete_calls == failed_delete_call) {
		return -ENOSPC;
	}
	if (!keep_stored_keys) {
		stored[addr->a.val[0] - 1] = false;
	}
	return 0;
}

int bt_unpair(uint8_t id, const bt_addr_le_t *addr)
{
	TEST_ASSERT_EQUAL_UINT8(BT_ID_DEFAULT, id);
	TEST_ASSERT_NULL(addr);
	unpair_calls++;
	if (!unpair_error && !keep_ram_keys) {
		/* Model the SDK: RAM can become empty regardless of flash contents. */
		ram_count = 0;
	}
	return unpair_error;
}

int settings_load_subtree_direct(const char *subtree, settings_load_direct_cb cb, void *data)
{
	TEST_ASSERT_EQUAL_STRING("bt/keys", subtree);
	if (load_error) {
		return load_error;
	}
	for (unsigned int i = 0; i < 2; i++) {
		cb("peer", stored[i] ? 52 : 0, NULL, NULL, data);
	}
	if (orphaned_key) {
		cb("unloaded-peer", 52, NULL, NULL, data);
	}
	/* As in the SDK, the direct-load API does not return callback errors. */
	return 0;
}

void test_success_requires_empty_ram_and_flash(void)
{
	TEST_ASSERT_EQUAL_INT(0, bt_mgmt_bond_storage_clear());
	TEST_ASSERT_EQUAL_UINT(2, delete_calls);
	TEST_ASSERT_EQUAL_UINT(1, unpair_calls);
	TEST_ASSERT_EQUAL_UINT(0, ram_count);
	TEST_ASSERT_FALSE(stored[0]);
	TEST_ASSERT_FALSE(stored[1]);
}

void test_flash_delete_failure_is_propagated_before_ram_keys_are_lost(void)
{
	failed_delete_call = 1;
	TEST_ASSERT_EQUAL_INT(-ENOSPC, bt_mgmt_bond_storage_clear());
	TEST_ASSERT_EQUAL_UINT(0, unpair_calls);
	TEST_ASSERT_EQUAL_UINT(2, ram_count);
	TEST_ASSERT_TRUE(stored[0]);
	TEST_ASSERT_TRUE(stored[1]);
}

void test_partial_deletion_is_a_failure_and_can_be_retried(void)
{
	failed_delete_call = 2;
	TEST_ASSERT_EQUAL_INT(-ENOSPC, bt_mgmt_bond_storage_clear());
	TEST_ASSERT_EQUAL_UINT(0, unpair_calls);
	TEST_ASSERT_FALSE(stored[0]);
	TEST_ASSERT_TRUE(stored[1]);
	failed_delete_call = 0;
	TEST_ASSERT_EQUAL_INT(0, bt_mgmt_bond_storage_clear());
}

void test_empty_ram_does_not_hide_a_persisted_bond(void)
{
	keep_stored_keys = true;
	TEST_ASSERT_EQUAL_INT(-EIO, bt_mgmt_bond_storage_clear());
	TEST_ASSERT_EQUAL_UINT(0, ram_count);
	TEST_ASSERT_TRUE(stored[0]);
}

void test_unloaded_persisted_bond_blocks_success(void)
{
	orphaned_key = true;
	TEST_ASSERT_EQUAL_INT(-EIO, bt_mgmt_bond_storage_clear());
}

void test_unpair_failure_is_propagated(void)
{
	unpair_error = -EBUSY;
	TEST_ASSERT_EQUAL_INT(-EBUSY, bt_mgmt_bond_storage_clear());
}

void test_remaining_ram_bond_blocks_success(void)
{
	keep_ram_keys = true;
	TEST_ASSERT_EQUAL_INT(-EIO, bt_mgmt_bond_storage_clear());
}

void test_reported_storage_load_failure_is_propagated(void)
{
	load_error = -EIO;
	TEST_ASSERT_EQUAL_INT(-EIO, bt_mgmt_bond_storage_clear());
}

void test_already_empty_storage_and_tombstones_succeed(void)
{
	ram_count = 0;
	stored[0] = stored[1] = false;
	TEST_ASSERT_EQUAL_INT(0, bt_mgmt_bond_storage_clear());
	TEST_ASSERT_EQUAL_UINT(0, delete_calls);
}

extern int unity_main(void);
int main(void)
{
	return unity_main();
}
