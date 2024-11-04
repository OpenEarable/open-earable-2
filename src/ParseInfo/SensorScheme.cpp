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
    return bt_gatt_attr_read(conn, attr, buf, len, offset, parseInfoScheme, parseInfoSchemeSize);
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

    // componentCount
    size_t componentCount = 0;
    for (size_t i = 0; i < scheme->groupCount; i++) {
        componentCount += scheme->groups[i].componentCount;
    }

    *buffer = componentCount;
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