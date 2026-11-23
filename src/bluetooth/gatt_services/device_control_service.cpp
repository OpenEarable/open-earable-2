/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "PowerManager.h"
#include "channel_assignment.h"
#include "earable_pairing.h"
#include "device_control_service.h"

LOG_MODULE_REGISTER(device_control, CONFIG_BLE_LOG_LEVEL);

// Versioned, confirmed terminal commands.
#define CONTROL_UUID(n) BT_UUID_DECLARE_128( \
	BT_UUID_128_ENCODE(0x8b5d1000 + (n), 0x4c7a, 0x4d8b, 0x9a61, 0x6c9e2f183a70))

static constexpr uint8_t protocol_version = 1;
enum control_command : uint8_t {
	clear_bonds_and_shutdown = 1,
	pair_current_side = 2,
	restore_provisioned = 3,
	enter_standalone = 4,
	pair_left = 5,
	pair_right = 6,
};
static uint8_t accepted[] = {protocol_version, 0, 0};
static bool confirmed;
static struct bt_gatt_indicate_params indication;
static void control_worker(struct k_work *);
K_WORK_DELAYABLE_DEFINE(execute_command, control_worker);

static void indication_confirmed(struct bt_conn *, struct bt_gatt_indicate_params *, uint8_t err)
{
	confirmed = err == 0;
}

static void indication_finished(struct bt_gatt_indicate_params *)
{
	// Execute only after the phone confirms receipt of command acceptance.
	// The delay also gives the ATT write response time to complete.
	if (confirmed) {
		k_work_reschedule(&execute_command, K_MSEC(250));
	} else {
		earable_pairing_release();
	}
}

static ssize_t read_control(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				&protocol_version, sizeof(protocol_version));
}

static ssize_t write_control(struct bt_conn *conn, const struct bt_gatt_attr *,
			    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset != 0) return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	if (len != 2) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	if (flags & (BT_GATT_WRITE_FLAG_PREPARE | BT_GATT_WRITE_FLAG_CMD)) {
		return BT_GATT_ERR(BT_ATT_ERR_WRITE_NOT_PERMITTED);
	}
	const auto *request = static_cast<const uint8_t *>(buf);
	if (request[0] != protocol_version || request[1] < clear_bonds_and_shutdown ||
	    request[1] > pair_right) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
	const struct bt_gatt_attr *status = bt_gatt_find_by_uuid(nullptr, 0, CONTROL_UUID(2));
	if (!status || !bt_gatt_is_subscribed(conn, status, BT_GATT_CCC_INDICATE)) {
		return BT_GATT_ERR(BT_ATT_ERR_CCC_IMPROPER_CONF);
	}
	if (!earable_pairing_claim()) return BT_GATT_ERR(BT_ATT_ERR_PROCEDURE_IN_PROGRESS);

	accepted[1] = request[1];
	confirmed = false;
	indication = {};
	indication.attr = status;
	indication.func = indication_confirmed;
	indication.destroy = indication_finished;
	indication.data = accepted;
	indication.len = sizeof(accepted);
	int err = bt_gatt_indicate(conn, &indication);
	if (err) {
		earable_pairing_release();
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
	return len;
}

static struct bt_gatt_attr control_attrs[] = {
	BT_GATT_PRIMARY_SERVICE(CONTROL_UUID(0)),
	BT_GATT_CHARACTERISTIC(CONTROL_UUID(1), BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
		read_control, write_control, nullptr),
	BT_GATT_CHARACTERISTIC(CONTROL_UUID(2), BT_GATT_CHRC_INDICATE,
		BT_GATT_PERM_NONE, nullptr, nullptr, nullptr),
	BT_GATT_CCC(nullptr, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT)
};
static struct bt_gatt_service control_svc = BT_GATT_SERVICE(control_attrs);

int device_control_service_register(void)
{
	return bt_gatt_service_register(&control_svc);
}

static void control_worker(struct k_work *)
{
	if (accepted[1] == clear_bonds_and_shutdown) {
		int err = power_manager.clear_bonds_and_power_down();
		LOG_ERR("Bond cleanup/shutdown failed: %d; restart required", err);
		return;
	}

	enum audio_channel side;
	channel_assignment_get(&side);
	enum earable_pair_mode mode = EARABLE_SEEKING;
	switch (accepted[1]) {
	case pair_current_side:
		break;
	case pair_left:
		side = AUDIO_CH_L;
		break;
	case pair_right:
		side = AUDIO_CH_R;
		break;
	case enter_standalone:
		mode = EARABLE_STANDALONE;
		break;
	case restore_provisioned:
		mode = EARABLE_PROVISIONED;
		break;
	}

	int err = earable_pairing_set_mode(mode, side);
	if (err) {
		LOG_ERR("Could not store earable pairing command: %d", err);
		earable_pairing_release();
		return;
	}
	earable_pairing_reboot();
}
