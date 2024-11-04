#ifndef _DEFAULT_SENSORS_H
#define _DEFAULT_SENSORS_H

#include "SensorScheme.h"

// ============= IMU =============

SensorComponent accComponents[] = {
    { .name = "X", .unit = "g", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Y", .unit = "g", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Z", .unit = "g", .parseType = PARSE_TYPE_FLOAT },
};

SensorComponent gyroComponents[] = {
    { .name = "X", .unit = "dps", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Y", .unit = "dps", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Z", .unit = "dps", .parseType = PARSE_TYPE_FLOAT },
};

SensorComponent magComponents[] = {
    { .name = "X", .unit = "uT", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Y", .unit = "uT", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Z", .unit = "uT", .parseType = PARSE_TYPE_FLOAT },
};

SensorComponentGroup imuGroups[] = {
    { .name = "ACC", .componentCount = 3, .components = accComponents },
    { .name = "GYRO", .componentCount = 3, .components = gyroComponents },
    { .name = "MAG", .componentCount = 3, .components = magComponents },
};

// ============= Sensors =============

SensorScheme sensors[] = {
    { .name = "IMU", .id = 0, .groupCount = 3, .groups = imuGroups },
};

ParseInfoScheme defaultSensors = {
    .sensorCount = 1,
    .sensors = sensors,
};

#endif // _DEFAULT_SENSORS_H