//#pragma once

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <zephyr/bluetooth/gatt.h>
#include "openearable_common.h"
#include "zbus_common.h"

#ifdef __cplusplus
extern "C" {
#endif

int init_sd_logger();
//int send_sensor_data(); //struct sensor_data * data);

#ifdef __cplusplus
}
#endif

#endif