#ifndef DEVICE_ERROR_SERVICE_H
#define DEVICE_ERROR_SERVICE_H

#include <stdint.h>

#include <zephyr/bluetooth/uuid.h>

#define BT_UUID_DEVICE_ERROR_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x5f9c0001, 0x6f4a, 0x4c6b, 0x9f0d, 0x4f2f3b0a0001)
#define BT_UUID_DEVICE_ERROR_EVENT_VAL \
	BT_UUID_128_ENCODE(0x5f9c0002, 0x6f4a, 0x4c6b, 0x9f0d, 0x4f2f3b0a0001)

#define BT_UUID_DEVICE_ERROR_SERVICE BT_UUID_DECLARE_128(BT_UUID_DEVICE_ERROR_SERVICE_VAL)
#define BT_UUID_DEVICE_ERROR_EVENT BT_UUID_DECLARE_128(BT_UUID_DEVICE_ERROR_EVENT_VAL)

#define DEVICE_ERROR_PAYLOAD_VERSION 1
#define DEVICE_ERROR_MESSAGE_MAX_LENGTH 48

#define DEVICE_ERROR_SOURCE_SYSTEM 0xFF

#define DEVICE_ERROR_CODE_FIRMWARE_FATAL 0x0301
#define DEVICE_ERROR_CODE_FIRMWARE_LOG_ERROR 0x0302
#define DEVICE_ERROR_CODE_FIRMWARE_LOG_WARNING 0x0303
#define DEVICE_ERROR_CODE_FIRMWARE_LOG_INFO 0x0304
#define DEVICE_ERROR_CODE_FIRMWARE_LOG_DEBUG 0x0305

#ifdef __cplusplus
extern "C" {
#endif

enum device_error_level {
	DEVICE_ERROR_LEVEL_INFO = 0,
	DEVICE_ERROR_LEVEL_WARNING = 1,
	DEVICE_ERROR_LEVEL_ERROR = 2,
	DEVICE_ERROR_LEVEL_FATAL = 3,
};

typedef struct __attribute__((packed)) {
	uint8_t version;
	uint8_t level;
	uint16_t error_code;
	uint8_t source_id;
	uint32_t timestamp_ms;
	char message[DEVICE_ERROR_MESSAGE_MAX_LENGTH];
} device_error_data_t;

int init_device_error_service(void);
int send_device_error(enum device_error_level level, uint16_t error_code, uint8_t source_id,
		      const char *message);

void device_error_log_backend_enable(void);
void device_error_log_backend_disable(void);

#ifdef __cplusplus
}
#endif

#endif
