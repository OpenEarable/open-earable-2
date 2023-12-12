#include "button_service.h"

static uint8_t button_state = 0;

static bool notify_enabled;

static void button_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t read_button_state(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &button_state,
					 sizeof(button_state));
}

BT_GATT_SERVICE_DEFINE(button_service,
BT_GATT_PRIMARY_SERVICE(BT_UUID_BUTTON),
BT_GATT_CHARACTERISTIC(BT_UUID_BUTTON_STATE,
            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
            BT_GATT_PERM_READ,
            read_button_state, NULL, &button_state),
BT_GATT_CCC(button_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

int bt_send_button_state(enum button_action _button_state)
{
	button_state = _button_state;

	if (!notify_enabled) {
		return -EACCES;
	}

	return bt_gatt_notify(NULL, &button_service.attrs[2],
			      &button_state,
			      sizeof(button_state));
}