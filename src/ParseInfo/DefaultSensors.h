#ifndef _DEFAULT_SENSORS_H
#define _DEFAULT_SENSORS_H

#include "SensorScheme.h"

#include "zbus_common.h"
#include "openearable_common.h"

#include "../SensorManager/PPG.h"
#include "../SensorManager/IMU.h"
#include "../SensorManager/Baro.h"
#include "../SensorManager/Temp.h"
#include "../SensorManager/BoneConduction.h"
#include "../SensorManager/Microphone.h"


// ============= Microphones =============

#define MICRO_CHANNEL_COUNT 2
SensorComponent microComponenents[MICRO_CHANNEL_COUNT] = {
    { .name = "INNER", .unit = "ADC", .parseType = PARSE_TYPE_UINT16 },
    { .name = "Outer", .unit = "ADC", .parseType = PARSE_TYPE_UINT16 },
};

#define MICRO_GROUP_COUNT 1
SensorComponentGroup microGroups[MICRO_GROUP_COUNT] = {
    { .name = "MICROPHONE", .componentCount = MICRO_CHANNEL_COUNT, .components = microComponenents },
};

// ============= IMU =============

#define IMU_ACC_COUNT 3
SensorComponent accComponents[IMU_ACC_COUNT] = {
    { .name = "X", .unit = "m/s^2", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Y", .unit = "m/s^2", .parseType = PARSE_TYPE_FLOAT },
    { .name = "Z", .unit = "m/s^2", .parseType = PARSE_TYPE_FLOAT },
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

#define SENSOR_COUNT 6
SensorScheme defaultSensors[SENSOR_COUNT] = {
    {
        .name = "9-Axis IMU",
        .id = ID_IMU,
        .groupCount = IMU_GROUP_COUNT,
        .groups = imuGroups,
        .configOptions = {
            .availableOptions = DATA_STREAMING | DATA_STORAGE | FREQUENCIES_DEFINED,
            .frequencyOptions = {
                .frequencyCount = sizeof(IMU::sample_rates.reg_vals),
                .defaultFrequencyIndex = 1,
                .maxBleFrequencyIndex = 2,
                .frequencies = IMU::sample_rates.sample_rates,
            },
        },
    },
    {
        .name = "Microphones",
        .id = ID_MICRO,
        .groupCount = MICRO_GROUP_COUNT,
        .groups = microGroups,
        .configOptions = {
            .availableOptions = DATA_STORAGE | FREQUENCIES_DEFINED, // no streaming
            .frequencyOptions = {
                .frequencyCount = sizeof(Microphone::sample_rates.reg_vals),
                .defaultFrequencyIndex = 1,
                .maxBleFrequencyIndex = 1,
                .frequencies = Microphone::sample_rates.sample_rates,
            },
        },
    },
    {
        .name = "Pulse Oximeter",
        .id = ID_PPG,
        .groupCount = PPG_GROUP_COUNT,
        .groups = ppgGroups,
        .configOptions = {
            .availableOptions = DATA_STREAMING | DATA_STORAGE | FREQUENCIES_DEFINED,
            .frequencyOptions = {
                .frequencyCount = sizeof(PPG::sample_rates.reg_vals),
                .defaultFrequencyIndex = 2,
                .maxBleFrequencyIndex = 12,
                .frequencies = PPG::sample_rates.sample_rates,
            },
        },
    },
    {
        .name = "Skin Temperature Sensor",
        .id = ID_OPTTEMP,
        .groupCount = OPTIC_TEMP_GROUP_COUNT,
        .groups = opticTemperatureGroups,
        .configOptions = {
            .availableOptions = DATA_STREAMING | DATA_STORAGE | FREQUENCIES_DEFINED,
            .frequencyOptions = {
                .frequencyCount = sizeof(Temp::sample_rates.reg_vals),
                .defaultFrequencyIndex = 4,
                .maxBleFrequencyIndex = 7,
                .frequencies = Temp::sample_rates.sample_rates,
            },
        },
    },
    {
        .name = "Ear Canal Pressure Sensor",
        .id = ID_TEMP_BARO,
        .groupCount = BARO_GROUP_COUNT,
        .groups = baroGroups,
        .configOptions = {
            .availableOptions = DATA_STREAMING | DATA_STORAGE | FREQUENCIES_DEFINED,
            .frequencyOptions = {
                .frequencyCount = sizeof(Baro::sample_rates.reg_vals),
                .defaultFrequencyIndex = 12,
                .maxBleFrequencyIndex = 17,
                .frequencies = Baro::sample_rates.sample_rates,
            },
        },
    },
    {
        .name = "Bone Conduction Accelerometer",
        .id = ID_BONE_CONDUCTION,
        .groupCount = BONE_CONDUCTION_IMU_GROUP_COUNT,
        .groups = boneConductionIMUGroups,
        .configOptions = {
            .availableOptions = DATA_STREAMING | DATA_STORAGE | FREQUENCIES_DEFINED,
            .frequencyOptions = {
                .frequencyCount = sizeof(BoneConduction::sample_rates.reg_vals),
                .defaultFrequencyIndex = 2,
                .maxBleFrequencyIndex = 6,
                .frequencies = BoneConduction::sample_rates.sample_rates,
            },
        }, 
    },
};

ParseInfoScheme defaultSensorIds = {
    .sensorCount = SENSOR_COUNT,
    .sensorIds = (uint8_t[]){ ID_IMU, ID_PPG, ID_OPTTEMP, ID_TEMP_BARO, ID_BONE_CONDUCTION, ID_MICRO },
};

#endif // _DEFAULT_SENSORS_H
