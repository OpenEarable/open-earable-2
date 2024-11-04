#ifndef _SENSOR_COMPONENT_H
#define _SENSOR_COMPONENT_H

#include <string>

#include "ParseType.h"

struct SensorComponent {
    std::string name;
    std::string unit;
    ParseType parseType;
};

struct SensorComponentGroup {
    std::string name;
    size_t componentCount;
    SensorComponent* components;
};

size_t getSensorComponentGroupSize(SensorComponentGroup* group);
ssize_t serializeSensorComponentGroup(SensorComponentGroup* group, char* buffer, size_t bufferSize);

#endif