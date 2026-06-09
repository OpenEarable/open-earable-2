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

#define DEVICE_ERROR_SOURCE_IMU 0x00
#define DEVICE_ERROR_SOURCE_TEMP_BARO 0x01
#define DEVICE_ERROR_SOURCE_MICROPHONE 0x02
#define DEVICE_ERROR_SOURCE_PPG 0x04
#define DEVICE_ERROR_SOURCE_PULSE_OX 0x05
#define DEVICE_ERROR_SOURCE_OPTICAL_TEMP 0x06
#define DEVICE_ERROR_SOURCE_BONE_CONDUCTION 0x07
#define DEVICE_ERROR_SOURCE_SYSTEM 0xFF

#define DEVICE_ERROR_CODE_SD_REMOVED 0x0001
#define DEVICE_ERROR_CODE_SD_NOT_MOUNTED 0x0002
#define DEVICE_ERROR_CODE_SD_MOUNT_FAILED 0x0003
#define DEVICE_ERROR_CODE_SD_OPEN_FAILED 0x0004
#define DEVICE_ERROR_CODE_SD_WRITE_FAILED 0x0005
#define DEVICE_ERROR_CODE_SD_HEADER_WRITE_FAILED 0x0006
#define DEVICE_ERROR_CODE_SD_FLUSH_FAILED 0x0007
#define DEVICE_ERROR_CODE_SD_BUFFER_FULL 0x0008
#define DEVICE_ERROR_CODE_SD_CLOSE_FAILED 0x0009

#define DEVICE_ERROR_CODE_SENSOR_QUEUE_FULL 0x0101
#define DEVICE_ERROR_CODE_SENSOR_INIT_FAILED 0x0102
#define DEVICE_ERROR_CODE_SENSOR_READ_FAILED 0x0103
#define DEVICE_ERROR_CODE_SENSOR_INVALID_SAMPLE_RATE 0x0104
#define DEVICE_ERROR_CODE_SENSOR_NOT_FOUND 0x0105
#define DEVICE_ERROR_CODE_SENSOR_CONFIG_QUEUE_FULL 0x0106

#define DEVICE_ERROR_CODE_BLE_NOTIFY_FAILED 0x0201
#define DEVICE_ERROR_CODE_FIRMWARE_FATAL 0x0301

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
int send_device_errorf(enum device_error_level level, uint16_t error_code, uint8_t source_id,
		       const char *fmt, ...);

void device_error_log_inf(uint16_t error_code, uint8_t source_id, const char *fmt, ...);
void device_error_log_wrn(uint16_t error_code, uint8_t source_id, const char *fmt, ...);
void device_error_log_err(uint16_t error_code, uint8_t source_id, const char *fmt, ...);
void device_error_log_fatal(uint16_t error_code, uint8_t source_id, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
