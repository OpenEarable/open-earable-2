#include "SensorScheme.h"
#include "sensor_service.h"

#include <string>
#include <cstring>

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <stdexcept>
#include <unordered_map>

LOG_MODULE_REGISTER(parse_info_service, CONFIG_LOG_DEFAULT_LEVEL);

static char* parseInfoScheme;
static size_t parseInfoSchemeSize;

static char* sensorSchemeBuffer;
static size_t sensorSchemeBufferSize;

static ParseInfoScheme* parseInfoSchemeStruct;
static std::unordered_map<uint8_t, SensorScheme*> sensorSchemesMap;

static bool notify_enabled = false;

int initSensorSchemeForId(uint8_t id);

static void notify_client(struct bt_conn *conn);

static ssize_t read_parse_info(struct bt_conn *conn,
                const struct bt_gatt_attr *attr,
                void *buf,
                uint16_t len,
                uint16_t offset) {
    ssize_t ret = bt_gatt_attr_read(conn, attr, buf, len, offset, parseInfoScheme, parseInfoSchemeSize);

    return ret;
}

static ssize_t read_sensor_scheme(struct bt_conn *conn,
                const struct bt_gatt_attr *attr,
                void *buf,
                uint16_t len,
                uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, sensorSchemeBuffer, sensorSchemeBufferSize);
}

static ssize_t write_sensor_request(struct bt_conn *conn,
                const struct bt_gatt_attr *attr,
                const void *buf,
                uint16_t len,
                uint16_t offset,
                uint8_t flags) {
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t requestedSensorId = *(uint8_t*)buf;

    int ret = initSensorSchemeForId(requestedSensorId);
    if (ret < 0) {
        LOG_ERR("Failed to initialize sensor scheme for id %d", requestedSensorId);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    notify_client(conn);

    return len;
}


void scheme_ccc_cfg(const struct bt_gatt_attr *attr, uint16_t value) {
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}


BT_GATT_SERVICE_DEFINE(parseInfo_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_PARSE_INFO_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_PARSE_INFO_CHARAC,
                BT_GATT_CHRC_READ,
                BT_GATT_PERM_READ,
                read_parse_info, NULL, parseInfoScheme),
    BT_GATT_CHARACTERISTIC(BT_UUID_PARSE_INFO_REQUEST_CHARAC,
                BT_GATT_CHRC_WRITE,
                BT_GATT_PERM_WRITE,
                NULL, write_sensor_request, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_PARSE_INFO_RESPONSE_CHARAC,
                BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                BT_GATT_PERM_READ,
                read_sensor_scheme, NULL, sensorSchemeBuffer),
    BT_GATT_CCC(scheme_ccc_cfg,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void notify_client(struct bt_conn *conn) {
    if (notify_enabled) {
        int ret = bt_gatt_notify(
            conn,
            &parseInfo_service.attrs[5],
            sensorSchemeBuffer,
            sensorSchemeBufferSize
        );

        if (ret) {
            LOG_ERR("Failed to notify client, error code: %d", ret);
        }
    }
}

size_t getFrequencyOptionsSchemeSize(FrequencyOptions* options) {
    size_t size = 0;
    size += 3; // frequencyCount, defaultFrequencyIndex, maxBleFrequencyIndex
    size += options->frequencyCount * sizeof(float); // frequencies

    return size;
}

ssize_t serializeFrequencyOptions(FrequencyOptions* options, char* buffer, size_t bufferSize) {
    size_t size = getFrequencyOptionsSchemeSize(options);
    if (size > bufferSize) {
        return -1;
    }

    // frequencyCount
    char* bufferStart = buffer;
    *buffer = options->frequencyCount;
    buffer++;

    // defaultFrequencyIndex
    *buffer = options->defaultFrequencyIndex;
    buffer++;

    // maxBleFrequencyIndex
    *buffer = options->maxBleFrequencyIndex;
    buffer++;

    // frequencies
    memcpy(buffer, options->frequencies, options->frequencyCount * sizeof(float));
    buffer += options->frequencyCount * sizeof(float);

    return buffer - bufferStart;
}

size_t getSensorOptionsSchemeSize(SensorConfigOptions* options) {
    size_t size = 0;
    size += 1; // availableOptions
    if (options->availableOptions & FREQUENCIES_DEFINED) {
        size += getFrequencyOptionsSchemeSize(&options->frequencyOptions);
    }

    return size;
}

ssize_t serializeSensorOptionsScheme(SensorConfigOptions* options, char* buffer, size_t bufferSize) {
    size_t size = getSensorOptionsSchemeSize(options);
    if (size > bufferSize) {
        return -1;
    }

    // availableOptions
    char* bufferStart = buffer;
    *buffer = options->availableOptions;
    buffer++;

    // frequencyOptions
    if (options->availableOptions & FREQUENCIES_DEFINED) {
        ssize_t frequencySize = serializeFrequencyOptions(&options->frequencyOptions, buffer, bufferSize - (buffer - bufferStart));
        if (frequencySize < 0) {
            return -1;
        }
        buffer += frequencySize;
    }

    return buffer - bufferStart;
}

size_t getSensorSchemeSize(SensorScheme* scheme) {
    size_t size = 0;
    size += 1; // id
    size += strlen(scheme->name) + 1; // name and name length
    size += 1; // groupCount
    for (size_t i = 0; i < scheme->groupCount; i++) {
        size += getSensorComponentGroupSize(&scheme->groups[i]);
    }
    size += getSensorOptionsSchemeSize(&scheme->configOptions);

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
    *buffer = strlen(scheme->name);
    buffer++;
    memcpy(buffer, scheme->name, strlen(scheme->name));
    buffer += strlen(scheme->name);

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

    // configOptions
    ssize_t optionsSize = serializeSensorOptionsScheme(&scheme->configOptions, buffer, bufferSize - (buffer - bufferStart));
    if (optionsSize < 0) {
        return -1;
    }
    buffer += optionsSize;

    return buffer - bufferStart;
}

size_t getSchemeSize(ParseInfoScheme* scheme) {
    size_t size = 0;
    size += 1; // sensorCount
    size += scheme->sensorCount * sizeof(uint8_t);

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

    // sensorIds
    memcpy(buffer, scheme->sensorIds, scheme->sensorCount * sizeof(uint8_t));
    buffer += scheme->sensorCount * sizeof(uint8_t);

    return buffer - bufferStart;
}

int initParseInfoService(ParseInfoScheme* scheme, SensorScheme* sensorSchemes) {
    LOG_DBG("Initializing parse info service");

    parseInfoSchemeStruct = scheme;
    
    for (size_t i = 0; i < scheme->sensorCount; i++) {
        LOG_DBG("Storing sensor scheme for id %d", sensorSchemes[i].id);
        sensorSchemesMap[sensorSchemes[i].id] = &sensorSchemes[i];
    }

    LOG_DBG("Stored sensor schemes in map");

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

float getSampleRateForSensorId(uint8_t id, uint8_t frequencyIndex) {
    SensorScheme* scheme = getSensorSchemeForId(id);
    if (scheme == NULL) {
        return -1;
    }

    return getSampleRateForSensor(scheme, frequencyIndex);
}

float getSampleRateForSensor(struct SensorScheme* sensorScheme, uint8_t frequencyIndex) {
    if (!(sensorScheme->configOptions.availableOptions & FREQUENCIES_DEFINED)) {
        return -1;
    }

    if (frequencyIndex >= sensorScheme->configOptions.frequencyOptions.frequencyCount) {
        return -1;
    }

    return sensorScheme->configOptions.frequencyOptions.frequencies[frequencyIndex];
}

int initSensorSchemeForId(uint8_t id) {
    SensorScheme* scheme = getSensorSchemeForId(id);

    if (scheme == NULL) {
        LOG_ERR("No sensor scheme found for id %d", id);
        return -1;
    }

    if (sensorSchemeBuffer != NULL) k_free(sensorSchemeBuffer);
    
    sensorSchemeBuffer = NULL;

    sensorSchemeBufferSize = getSensorSchemeSize(scheme);
    sensorSchemeBuffer = (char*)k_malloc(sensorSchemeBufferSize);

    if (sensorSchemeBuffer == NULL) {
        LOG_ERR("Failed to allocate memory for sensor scheme");
        return -ENOMEM;
    }

    ssize_t schemeSize = serializeSensorScheme(scheme, sensorSchemeBuffer, sensorSchemeBufferSize);
    if (schemeSize < 0) {
        LOG_ERR("Failed to serialize sensor scheme");
        return -1;
    } else if ((size_t) schemeSize != sensorSchemeBufferSize) {
        LOG_ERR("Serialized sensor scheme size does not match calculated size");
        return -1;
    }

    return 0;
}

struct SensorScheme* getSensorSchemeForId(uint8_t id) {
    return sensorSchemesMap[id];
}

struct ParseInfoScheme* getParseInfoScheme() {
    return parseInfoSchemeStruct;
}