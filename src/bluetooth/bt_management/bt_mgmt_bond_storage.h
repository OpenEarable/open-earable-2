/* SPDX-License-Identifier: Apache-2.0 */

#ifndef BT_MGMT_BOND_STORAGE_H_
#define BT_MGMT_BOND_STORAGE_H_

/**
 * Persist loaded bonds in the format shared by firmware 2.2.9 and 2.2.10.
 * Call after settings_load(), before advertising. Does not change in-memory
 * keys or Bluetooth identities. Returns zero on success, or a storage error.
 */
int bt_mgmt_bond_storage_compat(void);

#endif
