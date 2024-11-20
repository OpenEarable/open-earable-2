#ifndef _DEFAULT_SENSORS_H
#define _DEFAULT_SENSORS_H

#include "SensorScheme.h"

#include "nrf5340_audio_common.h"

// ============= IMU =============

#define IMU_ACC_COUNT 3
SensorComponent accComponents[IMU_ACC_COUNT] = {
    { .name = "X", .unit = "g", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Y", .unit = "g", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Z", .unit = "g", .parseType = PARSE_TYPE_FLOAT },
};

#define IMU_GYRO_COUNT 3
SensorComponent gyroComponents[IMU_GYRO_COUNT] = {
    { .name = "X", .unit = "dps", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Y", .unit = "dps", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Z", .unit = "dps", .parseType = PARSE_TYPE_FLOAT },
};

#define IMU_MAG_COUNT 3
SensorComponent magComponents[IMU_MAG_COUNT] = {
    { .name = "X", .unit = "uT", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Y", .unit = "uT", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Z", .unit = "uT", .parseType = PARSE_TYPE_FLOAT },
};

#define IMU_GROUP_COUNT 3
SensorComponentGroup imuGroups[IMU_GROUP_COUNT] = {
    { .name = "ACC", .componentCount = IMU_ACC_COUNT, .components = accComponents },
    { .name = "GYRO", .componentCount = IMU_GYRO_COUNT, .components = gyroComponents },
    { .name = "MAG", .componentCount = IMU_MAG_COUNT, .components = magComponents },
};

// ============= BoneConductionIMU =============

#define BONE_CONDUCTION_ACC_COUNT 3
SensorComponent boneConductionIMUComponents[BONE_CONDUCTION_ACC_COUNT] = {
    { .name = "X", .unit = "g", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Y", .unit = "g", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Z", .unit = "g", .parseType = PARSE_TYPE_FLOAT },
};

#define BONE_CONDUCTION_IMU_GROUP_COUNT 1
SensorComponentGroup boneConductionIMUGroups[BONE_CONDUCTION_IMU_GROUP_COUNT] = {
    { .name = "ACC", .componentCount = BONE_CONDUCTION_ACC_COUNT, .components = boneConductionIMUComponents },
};

// ============= PPG =============

#define PPG_ADC_COUNT 4
SensorComponent ppgAdcComponents[PPG_ADC_COUNT] = {
    { .name = "RED", .unit = "ADC", .parseType = PARSE_TYPE_UINT32 },
    { .name = "IR", .unit = "ADC", .parseType = PARSE_TYPE_UINT32 },
    { .name = "GREEN", .unit = "ADC", .parseType = PARSE_TYPE_UINT32 },
    { .name = "AMBIENT", .unit = "ADC", .parseType = PARSE_TYPE_UINT32 },
};

#define PPG_GROUP_COUNT 1
SensorComponentGroup ppgGroups[PPG_GROUP_COUNT] = {
    { .name = "PPG", .componentCount = PPG_ADC_COUNT, .components = ppgAdcComponents },
};

// ============= OpticTemperature =============

#define OPTIC_TEMP_COUNT 1
SensorComponent opticTemperatureComponents[OPTIC_TEMP_COUNT] = {
    { .name = "Temperature", .unit = "°C", .parseType = PARSE_TYPE_FLOAT },
};

#define OPTIC_TEMP_GROUP_COUNT 1
SensorComponentGroup opticTemperatureGroups[OPTIC_TEMP_GROUP_COUNT] = {
    { .name = "TEMP", .componentCount = OPTIC_TEMP_COUNT, .components = opticTemperatureComponents },
};

// ============= Baro =============

#define BARO_TEMP_COUNT 1
SensorComponent baroTempComponents[BARO_TEMP_COUNT] = {
    { .name = "Temperature", .unit = "°C", .parseType = PARSE_TYPE_FLOAT },
};

#define BARO_PRESSURE_COUNT 1
SensorComponent baroPressureCompoents[BARO_PRESSURE_COUNT] = {
    { .name = "Pressure", .unit = "kPa", .parseType = PARSE_TYPE_FLOAT },
};

#define BARO_GROUP_COUNT 2
SensorComponentGroup baroGroups[BARO_GROUP_COUNT] = {
    { .name = "TEMP", .componentCount = BARO_TEMP_COUNT, .components = baroTempComponents },
    { .name = "PRESSURE", .componentCount = BARO_PRESSURE_COUNT, .components = baroPressureCompoents },
};

// ============= Sensors =============

#define SENSOR_COUNT 5
SensorScheme sensors[SENSOR_COUNT] = {
    { .name = "IMU", .id = ID_IMU, .groupCount = IMU_GROUP_COUNT, .groups = imuGroups },
    { .name = "PPG", .id = ID_PPG, .groupCount = PPG_GROUP_COUNT, .groups = ppgGroups },
    { .name = "OPTTEMP", .id = ID_OPTTEMP, .groupCount = OPTIC_TEMP_GROUP_COUNT, .groups = opticTemperatureGroups },
    { .name = "BARO", .id = ID_TEMP_BARO, .groupCount = BARO_GROUP_COUNT, .groups = baroGroups },
    { .name = "BONEIMU", .id = ID_BONE_CONDUCTION, .groupCount = BONE_CONDUCTION_IMU_GROUP_COUNT, .groups = boneConductionIMUGroups },
};

ParseInfoScheme defaultSensors = {
    .sensorCount = SENSOR_COUNT,
    .sensors = sensors,
};

#endif // _DEFAULT_SENSORS_H