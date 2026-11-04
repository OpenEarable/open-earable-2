/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/settings/settings.h>

#include "settings.h"
#include "bt_mgmt_bond_storage.h"

static void delete_stored_bond(const struct bt_bond_info *info, void *data)
{
	int *err = data;

	if (*err == 0) {
		*err = bt_settings_delete_keys(BT_ID_DEFAULT, &info->addr);
	}
}

static void find_ram_bond(const struct bt_bond_info *info, void *data)
{
	(void)info;
	*(bool *)data = true;
}

static int find_stored_bond(const char *name, size_t len, settings_read_cb read_cb,
			    void *cb_arg, void *data)
{
	(void)name;
	(void)read_cb;
	(void)cb_arg;
	if (len != 0) {
		*(bool *)data = true;
	}
	return 0;
}

int bt_mgmt_bond_storage_clear(void)
{
	int err = 0;
	bool remains = false;

	/* bt_unpair() discards key-deletion errors and clears the RAM key pool
	 * anyway. Delete each persisted key first, while its address is available,
	 * and propagate failures before allowing unpair/shutdown to proceed.
	 */
	bt_foreach_bond(BT_ID_DEFAULT, delete_stored_bond, &err);
	if (err) {
		return err;
	}

	err = bt_unpair(BT_ID_DEFAULT, NULL);
	if (err) {
		return err;
	}

	bt_foreach_bond(BT_ID_DEFAULT, find_ram_bond, &remains);
	if (remains) {
		return -EIO;
	}

	/* Check flash independently of the now-empty RAM pool, including keys
	 * that were not loaded. Direct-load callbacks must record their result:
	 * this SDK does not propagate their return value to the caller.
	 */
	err = settings_load_subtree_direct("bt/keys", find_stored_bond, &remains);
	return err ? err : (remains ? -EIO : 0);
}
