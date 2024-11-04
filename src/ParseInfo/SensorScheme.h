#ifndef _SENSOR_SCHEME_H
#define _SENSOR_SCHEME_H

#include <string>

#include "SensorComponent.h"

#define BT_UUID_PARSE_INFO_SERVICE_VAL \
    BT_UUID_128_ENCODE(0xcaa25cb7, 0x7e1b, 0x44f2, 0xadc9, 0xe8c06c9ced43)

#define BT_UUID_PARSE_INFO_CHARAC_VAL \
    BT_UUID_128_ENCODE(0xcaa25cb7, 0x7e1b, 0x44f2, 0xadc9, 0xe8c06c9ced43)

#define BT_UUID_PARSE_INFO_SERVICE       BT_UUID_DECLARE_128(BT_UUID_PARSE_INFO_SERVICE_VAL)
#define BT_UUID_PARSE_INFO_CHARAC        BT_UUID_DECLARE_128(BT_UUID_PARSE_INFO_CHARAC_VAL)

struct SensorScheme {
    std::string name;
    int id;
    size_t groupCount;
    SensorComponentGroup* groups;
};

struct ParseInfoScheme {
    size_t sensorCount;
    SensorScheme* sensors;
};

int initParseInfoService(ParseInfoScheme* scheme);

#endif