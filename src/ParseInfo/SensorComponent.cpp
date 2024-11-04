#include "SensorComponent.h"
#include "ParseType.h"

#include <string>
#include <cstring>

size_t getSensorComponentGroupSize(SensorComponentGroup* group) {
    size_t size = 0;
    for (size_t i = 0; i < group->componentCount; i++) {
        size += 1;
        size += group->name.size() + 1;
        size += group->components[i].name.size() + 1;
        size += group->components[i].unit.size() + 1;
    }

    return size;
}

ssize_t serializeSensorComponentGroup(SensorComponentGroup* group, char* buffer, size_t bufferSize) {
    size_t size = getSensorComponentGroupSize(group);
    if (size > bufferSize) {
        return -1;
    }

    char* bufferStart = buffer;
    for (size_t i = 0; i < group->componentCount; i++) {
        *buffer = group->components[i].parseType;

        *buffer = group->name.size();
        buffer++;
        memcpy(buffer, group->name.c_str(), group->name.size());
        buffer += group->name.size();
        // *buffer = '\0';
        // buffer++;

        *buffer = group->components[i].name.size();
        buffer++;
        memcpy(buffer, group->components[i].name.c_str(), group->components[i].name.size());
        buffer += group->components[i].name.size();
        // *buffer = '\0';
        // buffer++;

        *buffer = group->components[i].unit.size();
        buffer++;
        memcpy(buffer, group->components[i].unit.c_str(), group->components[i].unit.size());
        buffer += group->components[i].unit.size();
        // *buffer = '\0';
        // buffer++;
    }

    return buffer - bufferStart;
}