#ifndef _SEAL_CHECK_SERVICE_H_
#define _SEAL_CHECK_SERVICE_H_

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include "openearable_common.h"

// Service UUID
#define BT_UUID_SEAL_CHECK_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x9abc, 0xdef123456789)

#define BT_UUID_SEAL_CHECK_SERVICE \
	BT_UUID_DECLARE_128(BT_UUID_SEAL_CHECK_SERVICE_VAL)

// Start Test Characteristic UUID (write 0xFF to start, reads back 0x00 when done)
#define BT_UUID_SEAL_CHECK_START_VAL \
	BT_UUID_128_ENCODE(0x12345679, 0x1234, 0x5678, 0x9abc, 0xdef123456789)

#define BT_UUID_SEAL_CHECK_START \
	BT_UUID_DECLARE_128(BT_UUID_SEAL_CHECK_START_VAL)

// Result Data Characteristic UUID (notify with seal_check_data)
#define BT_UUID_SEAL_CHECK_RESULT_VAL \
	BT_UUID_128_ENCODE(0x1234567A, 0x1234, 0x5678, 0x9abc, 0xdef123456789)

#define BT_UUID_SEAL_CHECK_RESULT \
	BT_UUID_DECLARE_128(BT_UUID_SEAL_CHECK_RESULT_VAL)

#ifdef __cplusplus
extern "C" {
#endif

// Function declarations
int init_seal_check_service(void);
int seal_check_notify_result(const struct seal_check_data *data);

#ifdef __cplusplus
}
#endif

#endif /* _SEAL_CHECK_SERVICE_H_ */