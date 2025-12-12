#include "seal_check_service.h"
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>
#include "../../SensorManager/SensorManager.h"

LOG_MODULE_REGISTER(seal_check_service, CONFIG_LOG_DEFAULT_LEVEL);

// Service state
static uint8_t seal_check_start_value = 0x00;
static struct seal_check_data seal_check_result_data;
static bool ccc_enabled = false;

// Function prototypes
extern int audio_datapath_multitone_play(uint16_t dur_ms, float amplitude);
extern int hw_codec_volume_set(uint8_t volume);
extern int seal_check_mic_index;
extern int16_t seal_check_mic[];

// Callback for start characteristic write
static ssize_t write_seal_check_start(struct bt_conn *conn,
				      const struct bt_gatt_attr *attr,
				      const void *buf, uint16_t len, 
				      uint16_t offset, uint8_t flags)
{
	if (offset + len > sizeof(seal_check_start_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t value = *((uint8_t*)buf);
	
	if (value == 0xFF) {
		LOG_INF("Seal check started via BLE");
		seal_check_start_value = 0xFF;
		
		// Set volume and start multitone
		hw_codec_volume_set(0xB0);
		
		// Configure sensor for recording
		struct sensor_config mic = {ID_MICRO, 6, 2}; // DATA_STORAGE = 2
		config_sensor(&mic);
		
		// Start multitone playback (1000ms, 1.0 amplitude)
		int ret = audio_datapath_multitone_play(1000, 1.0f);
		
		if (ret != 0) {
			LOG_ERR("Failed to start seal check: %d", ret);
			seal_check_start_value = 0x00;
			return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
		}
		
		LOG_INF("Seal check started successfully");
	}
	
	return len;
}

// Callback for start characteristic read
static ssize_t read_seal_check_start(struct bt_conn *conn,
				     const struct bt_gatt_attr *attr,
				     void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, 
				&seal_check_start_value, sizeof(seal_check_start_value));
}

// Callback for result characteristic CCC write
static void seal_check_result_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ccc_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Seal check result notifications %s", ccc_enabled ? "enabled" : "disabled");
}

// GATT service definition
BT_GATT_SERVICE_DEFINE(seal_check_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_SEAL_CHECK_SERVICE),
	
	// Start Test Characteristic
	BT_GATT_CHARACTERISTIC(BT_UUID_SEAL_CHECK_START,
			      BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			      BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			      read_seal_check_start, write_seal_check_start, 
			      &seal_check_start_value),
			      
	// Result Data Characteristic
	BT_GATT_CHARACTERISTIC(BT_UUID_SEAL_CHECK_RESULT,
			      BT_GATT_CHRC_NOTIFY,
			      BT_GATT_PERM_NONE,
			      NULL, NULL, &seal_check_result_data),
	BT_GATT_CCC(seal_check_result_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// Function to notify result data
int seal_check_notify_result(const struct seal_check_data *data)
{
	if (!ccc_enabled) {
		LOG_WRN("Seal check result notifications not enabled");
		return -ENOENT;
	}
	
	// Copy data to local storage
	memcpy(&seal_check_result_data, data, sizeof(seal_check_result_data));
	
	// Reset start value to indicate test completion
	seal_check_start_value = 0x00;
	
	// Send notification
	int err = bt_gatt_notify(NULL, &seal_check_svc.attrs[4], 
			        &seal_check_result_data, sizeof(seal_check_result_data));
	
	if (err) {
		LOG_ERR("Failed to notify seal check result: %d", err);
		return err;
	}
	
	LOG_INF("Seal check result notified successfully");
	return 0;
}

// Service initialization
int init_seal_check_service(void)
{
	LOG_INF("Seal check service initialized");
	return 0;
}