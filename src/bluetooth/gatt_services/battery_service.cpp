#include "battery_service.h"

#include "../Battery/BQ27220.h"

#include "../../Battery/PowerManager.h"

#include "macros_common.h"
#include "nrf5340_audio_common.h"

#include "macros_custom.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(battery_service, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);

static struct battery_data msg;
static bool notify_enabled;

DEFINE_GATT_SERVICE(battery, bt_send_battery_level, &msg); //(power_manager);

struct battery_level_status bat_status;
struct battery_energy_status en_status;

static void battery_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
#ifdef CONFIG_LOG
	if (notify_enabled) LOG_INF("subscribe to battery level");
	else LOG_INF("unsubscribe from battery level");
#endif
}

static ssize_t read_battery_level(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	msg.battery_level = (uint8_t) fuel_gauge.state_of_charge();

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &msg.battery_level,
					 sizeof(msg.battery_level));
}

static ssize_t read_charging_state(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	power_manager.get_battery_status(bat_status);
	//bat_status.flags = 0; //no level no addition
	//bat_status.power_state = msg.charging_state;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &bat_status,
					 sizeof(bat_status));
}

static ssize_t read_energy_state(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	power_manager.get_energy_status(en_status);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &en_status,
					 sizeof(en_status));
}

BT_GATT_SERVICE_DEFINE(battery_service,
BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),
BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
            BT_GATT_PERM_READ,
            read_battery_level, NULL, &msg.battery_level),
BT_GATT_CCC(battery_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
/*BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_POWER_STATE,
            BT_GATT_CHRC_READ,
            BT_GATT_PERM_READ,
            read_charging_state, NULL, &msg.charging_state),*/
BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL_STATUS,
            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
            BT_GATT_PERM_READ,
            read_charging_state, NULL, &bat_status),
BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_ENERGY_STATUS,
            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
            BT_GATT_PERM_READ,
            read_energy_state, NULL, &en_status),
);

int bt_send_battery_level(struct battery_data * data)
{
	if (!notify_enabled) {
		LOG_WRN("battery level not subscribed");
		return -EACCES;
	}

	LOG_INF("notify battery level change");

	return bt_gatt_notify(NULL, &battery_service.attrs[2], &msg.battery_level, sizeof(msg.battery_level));
}