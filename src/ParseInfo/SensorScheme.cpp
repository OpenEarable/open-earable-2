#include "SensorScheme.h"

#include <string>
#include <cstring>

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <stdexcept>

LOG_MODULE_REGISTER(parse_info_service, LOG_LEVEL_DBG);

static char* parseInfoScheme;
static size_t parseInfoSchemeSize;

static ssize_t read_parse_info(struct bt_conn *conn,
                const struct bt_gatt_attr *attr,
                void *buf,
                uint16_t len,
                uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &parseInfoScheme, parseInfoSchemeSize);
}

BT_GATT_SERVICE_DEFINE(parseInfo_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_PARSE_INFO_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_PARSE_INFO_CHARAC,
                BT_GATT_CHRC_READ,
                BT_GATT_PERM_READ,
                read_parse_info, NULL, parseInfoScheme)
);

size_t getSensorSchemeSize(SensorScheme* scheme) {
    size_t size = 0;
    size += 1; // id
    size += scheme->name.size() + 1; // name and name length
    size += 1; // groupCount
    for (size_t i = 0; i < scheme->groupCount; i++) {
        size += getSensorComponentGroupSize(&scheme->groups[i]);
    }

    return size;
}

ssize_t serializeSensorScheme(SensorScheme* scheme, char* buffer, size_t bufferSize) {
    size_t size = getSensorSchemeSize(scheme);
    if (size > bufferSize) {
        return -1;
    }

    // id
    char* bufferStart = buffer;
    *buffer = scheme->id;
    buffer++;

    // name
    *buffer = scheme->name.size();
    buffer++;
    memcpy(buffer, scheme->name.c_str(), scheme->name.size());
    buffer += scheme->name.size();
    // *buffer = '\0';
    // buffer++;

    // groupCount
    *buffer = scheme->groupCount;
    buffer++;

    // groups
    for (size_t i = 0; i < scheme->groupCount; i++) {
        ssize_t groupSize = serializeSensorComponentGroup(&scheme->groups[i], buffer, bufferSize - (buffer - bufferStart));
        if (groupSize < 0) {
            return -1;
        }
        buffer += groupSize;
    }

    return buffer - bufferStart;
}

size_t getSchemeSize(ParseInfoScheme* scheme) {
    size_t size = 0;
    size += 1; // sensorCount
    for (size_t i = 0; i < scheme->sensorCount; i++) {
        size += getSensorSchemeSize(&scheme->sensors[i]);
    }

    return size;
}

ssize_t serializeScheme(ParseInfoScheme* scheme, char* buffer, size_t bufferSize) {
    size_t size = getSchemeSize(scheme);
    if (size > bufferSize) {
        return -1;
    }

    // sensorCount
    char* bufferStart = buffer;
    *buffer = scheme->sensorCount;
    buffer++;

    // sensors
    for (size_t i = 0; i < scheme->sensorCount; i++) {
        ssize_t sensorSize = serializeSensorScheme(&scheme->sensors[i], buffer, bufferSize - (buffer - bufferStart));
        if (sensorSize < 0) {
            return -1;
        }
        buffer += sensorSize;
    }

    return buffer - bufferStart;
}

int buildParseInfoScheme(ParseInfoScheme* scheme) {
    #define SCHEMES_NODE DT_PATH(sensor_schemes)

    #define ONE(node) 1
    int sensor_count = DT_FOREACH_CHILD_SEP(SCHEMES_NODE, ONE, (+));
    
    LOG_DBG("Found %d sensors", sensor_count);

    // SensorScheme schemes[sensor_count] = {
    //     DT_FOREACH_CHILD(SCHEMES_NODE, ONE),
    // };

    // #define BUILD_COMPONENT(componentNode) \
    //     SensorComponent component = SensorComponent(); \
    //     component.name = DT_PROP(componentNode, comp_name); \
    //     component.unit = DT_PROP(componentNode, unit); \
    //     component.parseType = DT_PROP(componentNode, parse_type) \
    //     return component;

    // #define BUILD_GROUP(groupNode) \
    //     SensorComponentGroup group = SensorComponentGroup(); \
    //     group.name = DT_PROP(groupNode, group_name); \
    //     group.componentCount = DT_FOREACH_CHILD_SEP(groupNode, ONE, (+)); \
    //     group.components = { \
    //         DT_FOREACH_CHILD(groupNode, BUILD_COMPONENT), \
    //     }; \
    //     return group;

    // #define BUILD_SENSOR(sensorNode) \
    //     SensorScheme sensor = SensorScheme(); \
    //     sensor.name = DT_PROP(sensorNode, sensor_name); \
    //     sensor.id = DT_PROP(sensorNode, id); \
    //     sensor.groupCount = DT_FOREACH_CHILD_SEP(sensorNode, ONE, (+)); \
    //     sensor.groups = { \
    //         DT_FOREACH_CHILD(sensorNode, BUILD_GROUP), \
    //     }; \
    //     return sensor;

    // scheme->sensorCount = DT_FOREACH_CHILD_SEP(SCHEMES_NODE, ONE, (+));
    // scheme->sensors = {
    //     DT_FOREACH_CHILD(SCHEMES_NODE, BUILD_SENSOR),
    // };

    return 0;
}

// int initParseInfoService() {
//     ParseInfoScheme scheme = ParseInfoScheme();
//     int ret = buildParseInfoScheme(&scheme);
//     if (ret < 0) {
//         LOG_ERR("Failed to build parse info scheme: %d", ret);
//         return ret;
//     }
//     parseInfoSchemeSize = getSchemeSize(&scheme);

//     parseInfoScheme = (char*)k_malloc(parseInfoSchemeSize);
//     if (parseInfoScheme == NULL) {
//         LOG_ERR("Failed to allocate memory for parse info scheme");
//         return -ENOMEM;
//     }
//     ssize_t schemeSize = serializeScheme(&scheme, parseInfoScheme, parseInfoSchemeSize);
//     if (schemeSize < 0) {
//         LOG_ERR("Failed to serialize parse info scheme");
//         return -1;
//     } else if ((size_t) schemeSize != parseInfoSchemeSize) {
//         LOG_ERR("Serialized parse info scheme size does not match calculated size");
//         return -1;
//     }

//     return 0;
// }

int initParseInfoService(ParseInfoScheme* scheme) {
    parseInfoSchemeSize = getSchemeSize(scheme);

    LOG_DBG("Parse info scheme size: %d", parseInfoSchemeSize);

    parseInfoScheme = (char*)k_malloc(parseInfoSchemeSize);
    if (parseInfoScheme == NULL) {
        LOG_ERR("Failed to allocate memory for parse info scheme");
        return -ENOMEM;
    }
    ssize_t schemeSize = serializeScheme(scheme, parseInfoScheme, parseInfoSchemeSize);
    LOG_DBG("Serialized scheme size: %d", schemeSize);
    if (schemeSize < 0) {
        LOG_ERR("Failed to serialize parse info scheme");
        return -1;
    } else if ((size_t) schemeSize != parseInfoSchemeSize) {
        LOG_ERR("Serialized parse info scheme size does not match calculated size");
        return -1;
    }

    return 0;
}