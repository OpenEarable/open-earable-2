#include "battery_service.h"

static uint8_t battery_level = 100;
static uint8_t charging_state = 2;

static ssize_t read_battery_level(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	//battery_level = (uint8_t) fuel_gauge.state_of_charge();
	battery_level = (uint8_t) get_battery_level();

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &battery_level,
					 sizeof(battery_level));
}

static ssize_t read_charging_state(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &charging_state,
					 sizeof(charging_state));
}

BT_GATT_SERVICE_DEFINE(battery_service,
BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),
BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
            BT_GATT_CHRC_READ,
            BT_GATT_PERM_READ,
            read_battery_level, NULL, &battery_level),
BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_POWER_STATE,
            BT_GATT_CHRC_READ,
            BT_GATT_PERM_READ,
            read_charging_state, NULL, &charging_state),
);