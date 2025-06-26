//#pragma once

#ifndef SENSOR_SERVICE_H
#define SENSOR_SERVICE_H

#include <zephyr/bluetooth/gatt.h>
#include "openearable_common.h"
#include "zbus_common.h"

#define BT_UUID_SENSOR_VAL \
	BT_UUID_128_ENCODE(0x34c2e3bb, 0x34aa, 0x11eb, 0xadc1, 0x0242ac120002)

/** @brief Sensor Characteristic UUID. */
#define BT_UUID_SENSOR_CONFIG_VAL \
    BT_UUID_128_ENCODE(0x34c2e3be, 0x34aa, 0x11eb, 0xadc1, 0x0242ac120002)
#define BT_UUID_SENSOR_CONFIG_STATUS_VAL \
    BT_UUID_128_ENCODE(0x34c2e3bf, 0x34aa, 0x11eb, 0xadc1, 0x0242ac120002)
#define BT_UUID_SENSOR_RECORDING_NAME_VAL \
    BT_UUID_128_ENCODE(0x34c2e3c0, 0x34aa, 0x11eb, 0xadc1, 0x0242ac120002)

#define BT_UUID_SENSOR_DATA_VAL \
    BT_UUID_128_ENCODE(0x34c2e3bc, 0x34aa, 0x11eb, 0xadc1, 0x0242ac120002)

#define BT_UUID_SENSOR             BT_UUID_DECLARE_128(BT_UUID_SENSOR_VAL)
#define BT_UUID_SENSOR_CONFIG      BT_UUID_DECLARE_128(BT_UUID_SENSOR_CONFIG_VAL)
#define BT_UUID_SENSOR_CONFIG_STATUS BT_UUID_DECLARE_128(BT_UUID_SENSOR_CONFIG_STATUS_VAL)
#define BT_UUID_SENSOR_RECORDING_NAME BT_UUID_DECLARE_128(BT_UUID_SENSOR_RECORDING_NAME_VAL)
#define BT_UUID_SENSOR_DATA        BT_UUID_DECLARE_128(BT_UUID_SENSOR_DATA_VAL)

#ifdef __cplusplus
extern "C" {
#endif

int init_sensor_service();
const char *get_sensor_recording_name();
//int send_sensor_data(); //struct sensor_data * data);

int set_sensor_config_status(struct sensor_config config);

void temp_disable_notifies(bool disable);

#ifdef __cplusplus
}
#endif

#endif