/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "bt_mgmt.h"

#include <zephyr/zbus/zbus.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <nrfx.h>

#include "macros_common.h"
#include "zbus_common.h"
//#include "button_handler.h"
//#include "button_manager.h"
#include "bt_mgmt_ctlr_cfg_internal.h"
#include "bt_mgmt_adv_internal.h"
#include "bt_mgmt_dfu_internal.h"
#if CONFIG_BOARD_NRF5340_AUDIO_DK_NRF5340_CPUAPP
#include "button_assignments.h"
#endif

#include "BootState.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_mgmt, CONFIG_BT_MGMT_LOG_LEVEL);

ZBUS_CHAN_DEFINE(bt_mgmt_chan, struct bt_mgmt_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

/* The bt_enable should take less than 15 ms.
 * Buffer added as this will not add to bootup time
 */
#define BT_ENABLE_TIMEOUT_MS 100

K_SEM_DEFINE(sem_bt_enabled, 0, 1);

/**
 * @brief	Iterative function used to find connected conns
 *
 * @param[in]	conn	The connection to check
 * @param[out]	data	Pointer to store number of valid conns
 */
static void conn_state_connected_check(struct bt_conn *conn, void *data)
{
	int ret;
	uint8_t *num_conn = (uint8_t *)data;
	struct bt_conn_info info;

	ret = bt_conn_get_info(conn, &info);
	if (ret) {
		LOG_ERR("Failed to get conn info for %p: %d", (void *)conn, ret);
		return;
	}

	if (info.state != BT_CONN_STATE_CONNECTED) {
		return;
	}

	(*num_conn)++;
}

static void exchange_func(struct bt_conn *conn, uint8_t att_err,
			  struct bt_gatt_exchange_params *params)
{
	struct bt_conn_info info = {0};
	int err;

	LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
	LOG_INF("MTU size is: %d", bt_gatt_get_mtu(conn));

	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_WRN("Failed to get connection info %d\n", err);
		return;
	}

	/*if (info.role == BT_CONN_ROLE_MASTER) {
		instruction_print();
		//test_ready = true;
	}*/
}

static void le_data_length_updated(struct bt_conn *conn,
				   struct bt_conn_le_data_len_info *info)
{
	LOG_INF("LE data len updated: TX (len: %d time: %d)"
	       " RX (len: %d time: %d)", info->tx_max_len,
	       info->tx_max_time, info->rx_max_len, info->rx_max_time);
}

static struct bt_le_conn_param *conn_param = BT_LE_CONN_PARAM(CONFIG_BLE_ACL_CONN_INTERVAL, CONFIG_BLE_ACL_CONN_INTERVAL, 0, 400);

//callback
static void conn_params_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	struct bt_mgmt_msg msg;
	int ret;

	LOG_INF("Conn params updated: interval %d unit, latency %d, timeout: %d0 ms",interval, latency, timeout);

	msg.event = BT_MGMT_CONNECTED;
	msg.conn = conn;

	ret = zbus_chan_pub(&bt_mgmt_chan, &msg, K_NO_WAIT);
	ERR_CHK(ret);
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	int ret;
	char addr[BT_ADDR_LE_STR_LEN] = {0};
	struct bt_mgmt_msg msg;

	if (err == BT_HCI_ERR_ADV_TIMEOUT && IS_ENABLED(CONFIG_BT_PERIPHERAL)) {
		LOG_INF("Directed adv timed out with no connection, reverting to normal adv");

		bt_mgmt_dir_adv_timed_out(0);
		return;
	}

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		if (err == BT_HCI_ERR_UNKNOWN_CONN_ID) {
			LOG_WRN("ACL connection to addr: %s timed out, will try again", addr);
		} else {
			LOG_ERR("ACL connection to addr: %s, conn: %p, failed, error %d %s", addr,
				(void *)conn, err, bt_hci_err_to_str(err));
		}

		bt_conn_unref(conn);

		if (IS_ENABLED(CONFIG_BT_CENTRAL)) {
			ret = bt_mgmt_scan_start(0, 0, BT_MGMT_SCAN_TYPE_CONN, NULL,
						 BRDCAST_ID_NOT_USED);
			if (ret && ret != -EALREADY) {
				LOG_ERR("Failed to restart scanning: %d", ret);
			}
		}

		if (IS_ENABLED(CONFIG_BT_PERIPHERAL)) {
			ret = bt_mgmt_adv_start(0, NULL, 0, NULL, 0, true);
			if (ret) {
				LOG_ERR("Failed to restart advertising: %d", ret);
			}
		}

		return;
	}

	/* ACL connection established */
	/* NOTE: The string below is used by the Nordic CI system */
	LOG_INF("Connected: %s", addr);

	msg.event = BT_MGMT_CONNECTED;
	msg.conn = conn;

	ret = zbus_chan_pub(&bt_mgmt_chan, &msg, K_NO_WAIT);
	ERR_CHK(ret);

	static struct bt_gatt_exchange_params exchange_params;
	exchange_params.func = exchange_func;

	LOG_INF("MTU size is: %d", bt_gatt_get_mtu(conn));

	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		LOG_WRN("MTU exchange failed (err %d)", err);
	} else {
		LOG_INF("MTU exchange pending");
	}

	err = bt_conn_le_phy_update(conn, BT_CONN_LE_PHY_PARAM_2M);
	if (err) {
		LOG_ERR("Phy update request failed: %d",  err);
	}

	err = bt_conn_le_data_len_update(conn, BT_LE_DATA_LEN_PARAM_MAX);
	if (err) {
		LOG_ERR("LE data length update request failed: %d",  err);
	}

	err = bt_conn_le_param_update(conn, conn_param);
	if (err) {
		LOG_ERR("Cannot update conneciton parameter (err: %d)", err);
		return err;
	}
	LOG_INF("Connection parameters update requested");

	if (IS_ENABLED(CONFIG_BT_CENTRAL)) {
		ret = bt_conn_set_security(conn, BT_SECURITY_L2);
		if (ret) {
			LOG_ERR("Failed to set security to L2: %d", ret);
		}
	}
}

K_MUTEX_DEFINE(mtx_duplicate_scan);

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	int ret;
	char addr[BT_ADDR_LE_STR_LEN];
	struct bt_mgmt_msg msg;

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	/* NOTE: The string below is used by the Nordic CI system */
	LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

	if (IS_ENABLED(CONFIG_BT_CENTRAL)) {
		bt_conn_unref(conn);
	}

	/* Publish disconnected */
	msg.event = BT_MGMT_DISCONNECTED;
	msg.conn = conn;

	ret = zbus_chan_pub(&bt_mgmt_chan, &msg, K_NO_WAIT);
	ERR_CHK(ret);

	if (IS_ENABLED(CONFIG_BT_PERIPHERAL)) {
		ret = bt_mgmt_adv_start(0, NULL, 0, NULL, 0, true);
		ERR_CHK(ret);
	}

	/* The mutex for preventing the racing condition if two headset disconnected too close,
	 * cause the disconnected_cb() triggered in short time leads to duplicate scanning
	 * operation.
	 */
	k_mutex_lock(&mtx_duplicate_scan, K_FOREVER);
	if (IS_ENABLED(CONFIG_BT_CENTRAL)) {
		ret = bt_mgmt_scan_start(0, 0, BT_MGMT_SCAN_TYPE_CONN, NULL, BRDCAST_ID_NOT_USED);
		if (ret && ret != -EALREADY) {
			LOG_ERR("Failed to restart scanning: %d", ret);
		}
	}
	k_mutex_unlock(&mtx_duplicate_scan);
}

#if defined(CONFIG_BT_SMP)
static void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	int ret;
	struct bt_mgmt_msg msg;

	if (err) {
		LOG_WRN("Security failed: level %d err %d %s", level, err,
			bt_security_err_to_str(err));
		ret = bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
		if (ret) {
			LOG_WRN("Failed to disconnect %d", ret);
		}
	} else {
		LOG_DBG("Security changed: level %d", level);
		/* Publish connected */
		msg.event = BT_MGMT_SECURITY_CHANGED;
		msg.conn = conn;

		ret = zbus_chan_pub(&bt_mgmt_chan, &msg, K_NO_WAIT);
		ERR_CHK(ret);
	}
}
#endif /* defined(CONFIG_BT_SMP) */

static struct bt_conn_cb conn_callbacks = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.le_param_updated = conn_params_updated,
	.le_data_len_updated = le_data_length_updated,
#if defined(CONFIG_BT_SMP)
	.security_changed = security_changed_cb,
#endif /* defined(CONFIG_BT_SMP) */
};

static void bt_enabled_cb(int err)
{
	if (err) {
		LOG_ERR("Bluetooth init failed (err code: %d)", err);
		ERR_CHK(err);
	}

	k_sem_give(&sem_bt_enabled);

	LOG_DBG("Bluetooth initialized");
}

static int bonding_clear_check(void)
{
	int ret;
	/*bool pressed;

	ret = button_pressed(BUTTON_EARABLE, &pressed); //BUTTON_5
	if (ret) {
		return ret;
	}*/

	if (oe_boot_state.timer_reset) {
		LOG_INF("Device Count: %i", bonded_device_count);
		/*if (bonded_device_count == 0) {
			LOG_INF("Clearing SIRK");
			enum audio_channel channel;

			//backup channel
			channel_assignment_get(&channel);
			//int ret = nrfx_nvmc_page_erase((uint32_t)NRF_UICR); //
			//int ret = nrfx_nvmc_uicr_erase();
			nvmc_erase_mode_set();
			nrf_nvmc_uicr_erase_start(NRF_NVMC);
			while (!nrf_nvmc_ready_check(NRF_NVMC))
			{}
			nvmc_readonly_mode_set();
			int ret = NRFX_SUCCESS;
			if (ret != NRFX_SUCCESS) {
				LOG_ERR("Failed to erase UICR: %i", ret);
			}
			channel_assignment_set(channel);
			
			// reboot?

			// uint8_t data = BT_HCI_ERR_REMOTE_USER_TERM_CONN;
			//bt_conn_foreach(BT_CONN_TYPE_ALL, bt_disconnect_handler, &data);

			//ret = bt_le_adv_stop();

			//ret = bt_mgmt_stop_watchdog();
			//ERR_CHK(ret);

			if (!ret) sys_reboot(SYS_REBOOT_COLD);
		}*/
		ret = bt_mgmt_bonding_clear();
		return ret;
	}

	return 0;
}

static int ficr_static_addr_set(void)
{
	int ret;
	static bt_addr_le_t addr;

	if ((NRF_FICR->INFO.DEVICEID[0] != UINT32_MAX) ||
	    ((NRF_FICR->INFO.DEVICEID[1] & UINT16_MAX) != UINT16_MAX)) {
		/* Put the device ID from FICR into address */
		sys_put_le32(NRF_FICR->INFO.DEVICEID[0], &addr.a.val[0]);
		sys_put_le16(NRF_FICR->INFO.DEVICEID[1], &addr.a.val[4]);

		/* The FICR value is a just a random number, with no knowledge
		 * of the Bluetooth Specification requirements for random
		 * static addresses.
		 */
		BT_ADDR_SET_STATIC(&addr.a);

		addr.type = BT_ADDR_LE_RANDOM;

		ret = bt_id_create(&addr, NULL);
		if (ret < 0) {
			LOG_ERR("Failed to create ID %d", ret);
			return ret;
		}

		return 0;
	}

	/* If no address can be created (e.g. based on
	 * FICR), then a random address is created
	 */
	LOG_WRN("Unable to read from FICR");

	return 0;
}

/* This function generates a random address for bonding testing */
static int random_static_addr_set(void)
{
	int ret;
	static bt_addr_le_t addr;

	ret = bt_addr_le_create_static(&addr);
	if (ret < 0) {
		LOG_ERR("Failed to create address %d", ret);
		return ret;
	}

	ret = bt_id_create(&addr, NULL);
	if (ret < 0) {
		LOG_ERR("Failed to create ID %d", ret);
		return ret;
	}

	return 0;
}

static int local_identity_addr_print(void)
{
	size_t num_ids = 0;
	bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_id_get(NULL, &num_ids);
	if (num_ids != CONFIG_BT_ID_MAX) {
		LOG_ERR("The default config supports %d ids, but %d was found", CONFIG_BT_ID_MAX,
			num_ids);
		return -ENOMEM;
	}

	bt_id_get(addrs, &num_ids);

	for (int i = 0; i < num_ids; i++) {
		(void)bt_addr_le_to_str(&(addrs[i]), addr_str, BT_ADDR_LE_STR_LEN);
		LOG_INF("Local identity addr: %s", addr_str);
	}

	return 0;
}

void bt_mgmt_num_conn_get(uint8_t *num_conn)
{
	bt_conn_foreach(BT_CONN_TYPE_LE, conn_state_connected_check, (void *)num_conn);
}

int bt_mgmt_bonding_clear(void)
{
	int ret;

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		LOG_INF("Clearing all bonds");

		ret = bt_unpair(BT_ID_DEFAULT, NULL);
		if (ret) {
			LOG_ERR("Failed to clear bonding: %d", ret);
			return ret;
		}
	}

	return 0;
}

int bt_mgmt_pa_sync_delete(struct bt_le_per_adv_sync *pa_sync)
{
	if (IS_ENABLED(CONFIG_BT_PER_ADV_SYNC)) {
		int ret;

		ret = bt_le_per_adv_sync_delete(pa_sync);
		if (ret) {
			LOG_ERR("Failed to delete PA sync");
			return ret;
		}
	} else {
		LOG_WRN("Periodic advertisement sync not enabled");
		return -ENOTSUP;
	}

	return 0;
}

int bt_mgmt_conn_disconnect(struct bt_conn *conn, uint8_t reason)
{
	if (IS_ENABLED(CONFIG_BT_CONN)) {
		int ret;

		ret = bt_conn_disconnect(conn, reason);
		if (ret) {
			LOG_ERR("Failed to disconnect connection %p (%d)", (void *)conn, ret);
			return ret;
		}
	} else {
		LOG_WRN("BT conn not enabled");
		return -ENOTSUP;
	}

	return 0;
}

int bonded_device_count = 0;

void count_bonds(const struct bt_bond_info *info, void *user_data) {
	bonded_device_count++;
}

int bt_mgmt_init(void)
{
	int ret;

	if (!IS_ENABLED(CONFIG_BT_PRIVACY)) {
		ret = ficr_static_addr_set();
		if (ret) {
			return ret;
		}
	}

	ret = bt_enable(bt_enabled_cb);
	if (ret) {
		return ret;
	}

	ret = k_sem_take(&sem_bt_enabled, K_MSEC(BT_ENABLE_TIMEOUT_MS));
	if (ret) {
		LOG_ERR("bt_enable timed out");
		return ret;
	}

	if (IS_ENABLED(CONFIG_TESTING_BLE_ADDRESS_RANDOM)) {
		ret = random_static_addr_set();
		if (ret) {
			return ret;
		}
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		ret = settings_load();
		if (ret) {
			return ret;
		}

		bonded_device_count = 0;

		bt_foreach_bond(BT_ID_DEFAULT, count_bonds, NULL);

		ret = bonding_clear_check();
		if (ret) {
			return ret;
		}

		if (IS_ENABLED(CONFIG_TESTING_BLE_ADDRESS_RANDOM)) {
			ret = bt_mgmt_bonding_clear();
			if (ret) {
				return ret;
			}
		}
	}

#if defined(CONFIG_AUDIO_BT_MGMT_DFU)
	bool pressed;

	ret = button_pressed(BUTTON_6, &pressed);
	if (ret) {
		return ret;
	}

	if (pressed) {
		ret = bt_mgmt_ctlr_cfg_init(false);
		if (ret) {
			return ret;
		}
		/* This call will not return */
		bt_mgmt_dfu_start();
	}

#endif /* CONFIG_AUDIO_BT_MGMT_DFU */

	ret = bt_mgmt_ctlr_cfg_init(IS_ENABLED(CONFIG_WDT_CTLR));
	if (ret) {
		return ret;
	}

	ret = local_identity_addr_print();
	if (ret) {
		return ret;
	}

	if (IS_ENABLED(CONFIG_BT_CONN)) {
		bt_conn_cb_register(&conn_callbacks);
	}

	if (IS_ENABLED(CONFIG_BT_PERIPHERAL) || IS_ENABLED(CONFIG_BT_BROADCASTER)) {
		bt_mgmt_adv_init();
	}

	return 0;
}
