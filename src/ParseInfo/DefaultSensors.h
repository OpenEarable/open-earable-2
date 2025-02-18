#ifndef _DEFAULT_SENSORS_H
#define _DEFAULT_SENSORS_H

#include "SensorScheme.h"

#include "zbus_common.h"
#include "openearable_common.h"

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
    { .name = "ACCELEROMETER", .componentCount = IMU_ACC_COUNT, .components = accComponents },
    { .name = "GYROSCOPE", .componentCount = IMU_GYRO_COUNT, .components = gyroComponents },
    { .name = "MAGNETOMETER", .componentCount = IMU_MAG_COUNT, .components = magComponents },
};

// ============= BoneConductionIMU =============

#define BONE_CONDUCTION_ACC_COUNT 3
SensorComponent boneConductionIMUComponents[BONE_CONDUCTION_ACC_COUNT] = {
    { .name = "X", .unit = "g", .parseType = PARSE_TYPE_INT16 },
    { .name = "Y", .unit = "g", .parseType = PARSE_TYPE_INT16 },
    { .name = "Z", .unit = "g", .parseType = PARSE_TYPE_INT16 },
};

#define BONE_CONDUCTION_IMU_GROUP_COUNT 1
SensorComponentGroup boneConductionIMUGroups[BONE_CONDUCTION_IMU_GROUP_COUNT] = {
    { .name = "ACCELEROMETER", .componentCount = BONE_CONDUCTION_ACC_COUNT, .components = boneConductionIMUComponents },
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
    { .name = "PHOTOPLETHYSMOGRAPHY", .componentCount = PPG_ADC_COUNT, .components = ppgAdcComponents },
};

// ============= OpticTemperature =============

#define OPTIC_TEMP_COUNT 1
SensorComponent opticTemperatureComponents[OPTIC_TEMP_COUNT] = {
    { .name = "Temperature", .unit = "°C", .parseType = PARSE_TYPE_FLOAT },
};

#define OPTIC_TEMP_GROUP_COUNT 1
SensorComponentGroup opticTemperatureGroups[OPTIC_TEMP_GROUP_COUNT] = {
    { .name = "OPTICAL_TEMPERATURE_SENSOR", .componentCount = OPTIC_TEMP_COUNT, .components = opticTemperatureComponents },
};

// ============= Baro =============

#define BARO_TEMP_COUNT 1
SensorComponent baroTempComponents[BARO_TEMP_COUNT] = {
    { .name = "Temperature", .unit = "°C", .parseType = PARSE_TYPE_FLOAT },
};

#define BARO_PRESSURE_COUNT 1
SensorComponent baroPressureComponents[BARO_PRESSURE_COUNT] = {
    { .name = "Pressure", .unit = "kPa", .parseType = PARSE_TYPE_FLOAT },
};

#define BARO_GROUP_COUNT 2
SensorComponentGroup baroGroups[BARO_GROUP_COUNT] = {
    { .name = "TEMPERATURE_SENSOR", .componentCount = BARO_TEMP_COUNT, .components = baroTempComponents },
    { .name = "BAROMETER", .componentCount = BARO_PRESSURE_COUNT, .components = baroPressureComponents },
};

// ============= Sensors =============

#define SENSOR_COUNT 5
SensorScheme sensors[SENSOR_COUNT] = {
    { .name = "BMX160", .id = ID_IMU, .groupCount = IMU_GROUP_COUNT, .groups = imuGroups },
    { .name = "ANALOG_DEVICES_MAXM86161EFD+", .id = ID_PPG, .groupCount = PPG_GROUP_COUNT, .groups = ppgGroups },
    { .name = "MELEXIS_MLX90632", .id = ID_OPTTEMP, .groupCount = OPTIC_TEMP_GROUP_COUNT, .groups = opticTemperatureGroups },
    { .name = "BOSCH_BMP388", .id = ID_TEMP_BARO, .groupCount = BARO_GROUP_COUNT, .groups = baroGroups },
    { .name = "BOSCH_BMA580", .id = ID_BONE_CONDUCTION, .groupCount = BONE_CONDUCTION_IMU_GROUP_COUNT, .groups = boneConductionIMUGroups },
};

ParseInfoScheme defaultSensors = {
    .sensorCount = SENSOR_COUNT,
    .sensors = sensors,
};

#endif // _DEFAULT_SENSORS_H