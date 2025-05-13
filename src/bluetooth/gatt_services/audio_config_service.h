#ifndef _AUDIO_CONFIG_SERVICE_H_
#define _AUDIO_CONFIG_SERVICE_H_

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

// Service UUID: Replace with your own generated UUID
#define BT_UUID_AUDIO_CONFIG_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define BT_UUID_AUDIO_CONFIG_SERVICE \
	BT_UUID_DECLARE_128(BT_UUID_AUDIO_CONFIG_SERVICE_VAL)

// Mode Characteristic UUID
#define BT_UUID_AUDIO_MODE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

#define BT_UUID_AUDIO_MODE \
	BT_UUID_DECLARE_128(BT_UUID_AUDIO_MODE_VAL)

int init_audio_config_service(void);

#endif /* _AUDIO_CONFIG_SERVICE_H_ */
