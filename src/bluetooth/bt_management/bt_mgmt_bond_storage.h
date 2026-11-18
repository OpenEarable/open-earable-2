/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BT_MGMT_BOND_STORAGE_H_
#define BT_MGMT_BOND_STORAGE_H_

/**
 * Persist loaded bonds in the format shared by firmware 2.2.9 and 2.2.10.
 * Call after settings_load(), before advertising. Does not change in-memory
 * keys or Bluetooth identities. Returns zero on success, or a storage error.
 */
int bt_mgmt_bond_storage_compat(void);

/**
 * Delete default-identity bond keys with checked storage writes, unpair, and
 * verify that no bonds remain in RAM or persistent storage. Returns zero on
 * success or an error; callers must not shut down after a failed cleanup.
 * Call with advertising stopped and pairing prevented.
 */
int bt_mgmt_bond_storage_clear(void);

#endif
