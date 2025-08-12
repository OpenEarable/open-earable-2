#ifndef _SENSOR_SCHEME_H
#define _SENSOR_SCHEME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "SensorComponent.h"

#define BT_UUID_PARSE_INFO_SERVICE_VAL \
    BT_UUID_128_ENCODE(0xcaa25cb7, 0x7e1b, 0x44f2, 0xadc9, 0xe8c06c9ced43)

#define BT_UUID_PARSE_INFO_CHARAC_VAL \
    BT_UUID_128_ENCODE(0xcaa25cb9, 0x7e1b, 0x44f2, 0xadc9, 0xe8c06c9ced43)

#define BT_UUID_PARSE_INFO_REQUEST_CHARAC_VAL \
    BT_UUID_128_ENCODE(0xcaa25cba, 0x7e1b, 0x44f2, 0xadc9, 0xe8c06c9ced43)

#define BT_UUID_PARSE_INFO_RESPONSE_CHARAC_VAL \
    BT_UUID_128_ENCODE(0xcaa25cbb, 0x7e1b, 0x44f2, 0xadc9, 0xe8c06c9ced43)

#define BT_UUID_PARSE_INFO_SERVICE       BT_UUID_DECLARE_128(BT_UUID_PARSE_INFO_SERVICE_VAL)
#define BT_UUID_PARSE_INFO_CHARAC        BT_UUID_DECLARE_128(BT_UUID_PARSE_INFO_CHARAC_VAL)
#define BT_UUID_PARSE_INFO_REQUEST_CHARAC        BT_UUID_DECLARE_128(BT_UUID_PARSE_INFO_REQUEST_CHARAC_VAL)
#define BT_UUID_PARSE_INFO_RESPONSE_CHARAC        BT_UUID_DECLARE_128(BT_UUID_PARSE_INFO_RESPONSE_CHARAC_VAL)

enum SensorConfigOptionsMasks {
    DATA_STREAMING = 0x01,
    DATA_STORAGE = 0x02,
    FREQUENCIES_DEFINED = 0x10,
};

struct FrequencyOptions {
    uint8_t frequencyCount;
    uint8_t defaultFrequencyIndex;
    uint8_t maxBleFrequencyIndex;
    const float* frequencies;
};

struct SensorConfigOptions {
    uint8_t availableOptions;
    struct FrequencyOptions frequencyOptions;
};


struct SensorScheme {
    const char* name;
    uint8_t id;
    uint8_t groupCount;
    struct SensorComponentGroup* groups;
    struct SensorConfigOptions configOptions;
};

struct ParseInfoScheme {
    uint8_t sensorCount;
    uint8_t* sensorIds;
};

int initParseInfoService(struct ParseInfoScheme* scheme, struct SensorScheme* sensorSchemes);

struct SensorScheme* getSensorSchemeForId(uint8_t id);
struct ParseInfoScheme* getParseInfoScheme();

float getSampleRateForSensorId(uint8_t id, uint8_t frequencyIndex);
float getSampleRateForSensor(struct SensorScheme* sensorScheme, uint8_t frequencyIndex);

#ifdef __cplusplus
}
#endif

#endif // _SENSOR_SCHEME_H