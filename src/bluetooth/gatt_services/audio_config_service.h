#ifndef _AUDIO_CONFIG_SERVICE_H_
#define _AUDIO_CONFIG_SERVICE_H_

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

// Service UUID
#define BT_UUID_AUDIO_CONFIG_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x1410df95, 0x5f68, 0x4ebb, 0xa7c7, 0x5e0fb9ae7557)

#define BT_UUID_AUDIO_CONFIG_SERVICE \
	BT_UUID_DECLARE_128(BT_UUID_AUDIO_CONFIG_SERVICE_VAL)

// Mode Characteristic UUID
#define BT_UUID_AUDIO_MODE_VAL \
	BT_UUID_128_ENCODE(0x1410df96, 0x5f68, 0x4ebb, 0xa7c7, 0x5e0fb9ae7557)

#define BT_UUID_AUDIO_MODE \
	BT_UUID_DECLARE_128(BT_UUID_AUDIO_MODE_VAL)

// Microphone Selection Characteristic UUID
#define BT_UUID_MIC_SELECT_VAL \
	BT_UUID_128_ENCODE(0x1410df97, 0x5f68, 0x4ebb, 0xa7c7, 0x5e0fb9ae7557)

#define BT_UUID_MIC_SELECT \
	BT_UUID_DECLARE_128(BT_UUID_MIC_SELECT_VAL)

// Audio Channel Characteristic UUID
#define BT_UUID_AUDIO_CHANNEL_VAL \
	BT_UUID_128_ENCODE(0x1410df98, 0x5f68, 0x4ebb, 0xa7c7, 0x5e0fb9ae7557)

#define BT_UUID_AUDIO_CHANNEL \
	BT_UUID_DECLARE_128(BT_UUID_AUDIO_CHANNEL_VAL)

int init_audio_config_service(void);

#endif /* _AUDIO_CONFIG_SERVICE_H_ */
