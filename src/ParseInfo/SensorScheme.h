#ifndef _SENSOR_SCHEME_H
#define _SENSOR_SCHEME_H

#include <string>

#include "SensorComponent.h"

#define BT_UUID_PARSE_INFO_SERVICE_VAL \
    BT_UUID_128_ENCODE(0xcaa25cb7, 0x7e1b, 0x44f2, 0xadc9, 0xe8c06c9ced43)

#define BT_UUID_PARSE_INFO_CHARAC_VAL \
    BT_UUID_128_ENCODE(0xcaa25cb9, 0x7e1b, 0x44f2, 0xadc9, 0xe8c06c9ced43)

#define BT_UUID_PARSE_INFO_SERVICE       BT_UUID_DECLARE_128(BT_UUID_PARSE_INFO_SERVICE_VAL)
#define BT_UUID_PARSE_INFO_CHARAC        BT_UUID_DECLARE_128(BT_UUID_PARSE_INFO_CHARAC_VAL)

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
    FrequencyOptions frequencyOptions;
};


struct SensorScheme {
    std::string name;
    uint8_t id;
    uint8_t groupCount;
    SensorComponentGroup* groups;
    SensorConfigOptions configOptions;
};

struct ParseInfoScheme {
    uint8_t sensorCount;
    SensorScheme* sensors;
};

int initParseInfoService(ParseInfoScheme* scheme);

SensorScheme* getSensorSchemeForId(uint8_t id);

#endif