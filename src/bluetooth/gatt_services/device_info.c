#include "device_info.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

static char device_identifier[] = "abcd";
static char device_generation[] = "2.0.0";
static char firmware[] = "2.0.0";

static ssize_t read_device_identifier(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, device_identifier,
					 sizeof(device_identifier));
}

static ssize_t read_device_generation(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, device_generation,
					 sizeof(device_generation));
}

static ssize_t read_firmware(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, firmware,
					 sizeof(firmware));
}

BT_GATT_SERVICE_DEFINE(device_svc,
BT_GATT_PRIMARY_SERVICE(BT_UUID_DEVICE_INFO),
BT_GATT_CHARACTERISTIC(BT_UUID_IDENT,
            BT_GATT_CHRC_READ,
            BT_GATT_PERM_READ,
            read_device_identifier, NULL, device_identifier),
BT_GATT_CHARACTERISTIC(BT_UUID_GENERATION,
            BT_GATT_CHRC_READ,
            BT_GATT_PERM_READ,
            read_device_generation, NULL, device_generation),
BT_GATT_CHARACTERISTIC(BT_UUID_FIRMWARE,
            BT_GATT_CHRC_READ,
            BT_GATT_PERM_READ,
            read_firmware, NULL, firmware),
);