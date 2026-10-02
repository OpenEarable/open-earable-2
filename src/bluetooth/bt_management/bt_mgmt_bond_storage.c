/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <stddef.h>
#include <zephyr/sys/byteorder.h>

#include "keys.h"
#include "settings.h"
#include "bt_mgmt_bond_storage.h"

/* NCS 3.4.1 prepends four configuration bytes to the NCS 3.0.1 bond record.
 * Keep the flash representation readable by firmware 2.2.9. The new SDK already
 * accepts this legacy representation and adds its header when loading it.
 * Reject build/configuration changes until their storage layout is reviewed.
 */
#define BOND_CONFIG_SIZE (offsetof(struct bt_keys, enc_size) - \
			 offsetof(struct bt_keys, storage_start))
BUILD_ASSERT(BOND_CONFIG_SIZE == 4 && BT_KEYS_STORAGE_LEN == 56);
BUILD_ASSERT(IS_ENABLED(CONFIG_BT_SMP_SC_PAIR_ONLY) &&
	     !IS_ENABLED(CONFIG_BT_SIGNING) && !IS_ENABLED(CONFIG_BT_KEYS_OVERWRITE_OLDEST));

int __real_bt_settings_store_keys(uint8_t id, const bt_addr_le_t *addr,
				 const void *value, size_t val_len);

/* Wrap all SDK bond writes, including newly paired phones and key updates. */
int __wrap_bt_settings_store_keys(uint8_t id, const bt_addr_le_t *addr,
				 const void *value, size_t val_len)
{
	const uint8_t *record = value;

	if (val_len != BT_KEYS_STORAGE_LEN || record[0] != 17 ||
	    sys_get_le24(&record[1]) != BT_KEYS_CFG_SC_PAIR_ONLY) {
		return -EINVAL;
	}

	return __real_bt_settings_store_keys(id, addr, record + BOND_CONFIG_SIZE,
					    val_len - BOND_CONFIG_SIZE);
}

static void store_legacy_bond(struct bt_keys *keys, void *data)
{
	int *err = data;

	if (*err == 0) {
		*err = bt_keys_store(keys);
	}
}

int bt_mgmt_bond_storage_compat(void)
{
	int err = 0;

	/* Also convert bonds written by an earlier 2.2.10 build, before advertising.
	 * NVS skips writes when the stored value is already identical.
	 */
	bt_keys_foreach_type(BT_KEYS_ALL, store_legacy_bond, &err);
	return err;
}
