#ifndef _POWER_SAVING_SERVICE_H_
#define _POWER_SAVING_SERVICE_H_

#include <zephyr/bluetooth/uuid.h>

/** @brief Power saving service UUID. */
#define BT_UUID_POWER_SAVING_SERVICE_VAL \
	BT_UUID_128_ENCODE(0xd63fd1f0, 0x5f68, 0x4ebb, 0xa7c7, 0x5e0fb9ae7557)

/** @brief Current power saving mode characteristic UUID. */
#define BT_UUID_POWER_SAVING_MODE_VAL \
	BT_UUID_128_ENCODE(0xd63fd1f1, 0x5f68, 0x4ebb, 0xa7c7, 0x5e0fb9ae7557)

/** @brief Supported power saving modes characteristic UUID. */
#define BT_UUID_POWER_SAVING_SUPPORTED_MODES_VAL \
	BT_UUID_128_ENCODE(0xd63fd1f2, 0x5f68, 0x4ebb, 0xa7c7, 0x5e0fb9ae7557)

#define BT_UUID_POWER_SAVING_SERVICE \
	BT_UUID_DECLARE_128(BT_UUID_POWER_SAVING_SERVICE_VAL)

#define BT_UUID_POWER_SAVING_MODE \
	BT_UUID_DECLARE_128(BT_UUID_POWER_SAVING_MODE_VAL)

#define BT_UUID_POWER_SAVING_SUPPORTED_MODES \
	BT_UUID_DECLARE_128(BT_UUID_POWER_SAVING_SUPPORTED_MODES_VAL)

#endif /* _POWER_SAVING_SERVICE_H_ */
