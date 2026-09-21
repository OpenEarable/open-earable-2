#include "battery_service.h"

#include "../../Battery/BQ27220.h"
#include "../../Battery/PowerManager.h"

#include "macros_common.h"
#include "openearable_common.h"

#include "macros_custom.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(battery_service, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);

static struct battery_data msg;
static bool notify_enabled;

static struct k_thread thread_data;
static k_tid_t thread_id;

ZBUS_SUBSCRIBER_DEFINE(battery_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);
ZBUS_CHAN_DECLARE(battery_chan);

static K_THREAD_STACK_DEFINE(thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE);

static void write_battery_gatt(void)
{
	int ret;
	const struct zbus_channel *chan;

	while (1) {
		ret = zbus_sub_wait(&battery_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		ret = zbus_chan_read(chan, &msg, ZBUS_READ_TIMEOUT_MS);
		ERR_CHK(ret);

		if (notify_enabled) {
        	bt_send_battery_level(&msg);
		}

		STACK_USAGE_PRINT("battery_msg_thread", &thread_data);
	}
}

int init_battery_service() {
    int ret;

	thread_id = k_thread_create(
		&thread_data, thread_stack,
		K_THREAD_STACK_SIZEOF(thread_stack), (k_thread_entry_t)write_battery_gatt, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_BUTTON_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);

	ret = k_thread_name_set(thread_id, "battery_gatt_sub");
	if (ret) {
		LOG_ERR("Failed to create  battery_msg thread");
		return ret;
	}

    ret = zbus_chan_add_obs(&battery_chan, &battery_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("Failed to add battery sub");
		return ret;
	}

    return 0;
}

struct battery_level_status bat_status;
struct battery_energy_status en_status;
struct battery_health_status health_status;

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

static ssize_t read_health_state(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	power_manager.get_health_status(health_status);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &health_status,
					 sizeof(health_status));
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
BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_HEALTH_STATUS,
            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
            BT_GATT_PERM_READ,
            read_health_state, NULL, &health_status),
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