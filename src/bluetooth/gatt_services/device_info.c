#include "device_info.h"
#include <generated/version.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

#include "../../Battery/BootState.h"
#include "../../utils/uicr.h"

static char device_identifier[sizeof(uint64_t) * 2 + 3];
char device_generation[16];
static char firmware[] = FIRMWARE_VERSION;

static ssize_t read_device_identifier(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	snprintf(device_identifier, sizeof(device_identifier), "0x%08X", oe_boot_state.device_id);
	
	return bt_gatt_attr_read(conn, attr, buf, len, offset, device_identifier,
					 sizeof(device_identifier));
}

static ssize_t read_device_generation(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	uicr_hw_revision_get(device_generation);
	
	return bt_gatt_attr_read(conn, attr, buf, len, offset, device_generation,
					 strlen(device_generation));
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