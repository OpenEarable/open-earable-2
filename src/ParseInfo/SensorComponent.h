#ifndef _SENSOR_COMPONENT_H
#define _SENSOR_COMPONENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ParseType.h"

#include <sys/types.h>

struct SensorComponent {
    const char* name;
    const char* unit;
    enum ParseType parseType;
};

struct SensorComponentGroup {
    const char* name;
    size_t componentCount;
    struct SensorComponent* components;
};

size_t getSensorComponentGroupSize(struct SensorComponentGroup* group);
ssize_t serializeSensorComponentGroup(struct SensorComponentGroup* group, char* buffer, size_t bufferSize);

#ifdef __cplusplus
}
#endif

#endif