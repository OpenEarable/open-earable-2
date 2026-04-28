#ifndef ERROR_DEFINITIONS_H
#define ERROR_DEFINITIONS_H

#include <stdint.h>

// Define error codes
#define IMU_STREAM_ERROR                    0xFF
#define ERROR_STREAM_START                  0x01
// ... more error codes later ...

// The structure to be sent over BLE
typedef struct __attribute__((packed)) {
    uint8_t error_code;
    uint8_t sensor_id;
    uint32_t timestamp; // e.g., k_uptime_get()
    char message[64];
} sensor_error_data_t;

#endif // ERROR_DEFINITIONS_H