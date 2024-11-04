#ifndef _DEFAULT_SENSORS_H
#define _DEFAULT_SENSORS_H

#include "SensorScheme.h"

// Define components for each group
SensorComponent accComponents[] = {
    { .name = "ACC_X", .unit = "g", .parseType = PARSE_TYPE_FLOAT },
    { .name = "ACC_Y", .unit = "g", .parseType = PARSE_TYPE_FLOAT },
    { .name = "ACC_Z", .unit = "g", .parseType = PARSE_TYPE_FLOAT },
};

SensorComponent gyroComponents[] = {
    { .name = "GYRO_X", .unit = "dps", .parseType = PARSE_TYPE_FLOAT },
    { .name = "GYRO_Y", .unit = "dps", .parseType = PARSE_TYPE_FLOAT },
    { .name = "GYRO_Z", .unit = "dps", .parseType = PARSE_TYPE_FLOAT },
};

SensorComponent magComponents[] = {
    { .name = "MAG_X", .unit = "uT", .parseType = PARSE_TYPE_FLOAT },
    { .name = "MAG_Y", .unit = "uT", .parseType = PARSE_TYPE_FLOAT },
    { .name = "MAG_Z", .unit = "uT", .parseType = PARSE_TYPE_FLOAT },
};

// Define groups for the IMU sensor
SensorComponentGroup imuGroups[] = {
    { .name = "ACC", .componentCount = 3, .components = accComponents },
    { .name = "GYRO", .componentCount = 3, .components = gyroComponents },
    { .name = "MAG", .componentCount = 3, .components = magComponents },
};

// Define the sensor scheme
SensorScheme sensors[] = {
    { .name = "IMU", .id = 0, .groupCount = 3, .groups = imuGroups },
};

// Define the ParseInfoScheme instance
ParseInfoScheme defaultSensors = {
    .sensorCount = 1,
    .sensors = sensors,
};

#endif // _DEFAULT_SENSORS_H