//#pragma once

#ifndef BATTERY_SERVICE_H
#define BATTERY_SERVICE_H

#include <zephyr/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

float get_battery_level();

#ifdef __cplusplus
}
#endif


#endif