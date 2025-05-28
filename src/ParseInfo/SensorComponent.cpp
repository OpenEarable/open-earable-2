#include "SensorComponent.h"
#include "ParseType.h"

#include <string>
#include <cstring>

size_t getSensorComponentGroupSize(struct SensorComponentGroup* group) {
    size_t size = 0;
    for (size_t i = 0; i < group->componentCount; i++) {
        size += 1;
        size += strlen(group->name) + 1;
        size += strlen(group->components[i].name) + 1;
        size += strlen(group->components[i].unit) + 1;
    }

    return size;
}

ssize_t serializeSensorComponentGroup(struct SensorComponentGroup* group, char* buffer, size_t bufferSize) {
    size_t size = getSensorComponentGroupSize(group);
    if (size > bufferSize) {
        return -1;
    }

    char* bufferStart = buffer;
    for (size_t i = 0; i < group->componentCount; i++) {
        *buffer = group->components[i].parseType;
        buffer++;

        *buffer = strlen(group->name);
        buffer++;
        memcpy(buffer, group->name, strlen(group->name));
        buffer += strlen(group->name);

        *buffer = strlen(group->components[i].name);
        buffer++;
        memcpy(buffer, group->components[i].name, strlen(group->components[i].name));
        buffer += strlen(group->components[i].name);

        *buffer = strlen(group->components[i].unit);
        buffer++;
        memcpy(buffer, group->components[i].unit, strlen(group->components[i].unit));
        buffer += strlen(group->components[i].unit);
    }

    return buffer - bufferStart;
}