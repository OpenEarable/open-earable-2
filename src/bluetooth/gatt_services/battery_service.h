//#pragma once

#ifndef BATTERY_SERVICE_H
#define BATTERY_SERVICE_H

#include <zephyr/bluetooth/gatt.h>
#include "nrf5340_audio_common.h"

#ifdef __cplusplus
extern "C" {
#endif

int init_battery_service();

float get_battery_level();

int bt_send_battery_level(struct battery_data * data);

#ifdef __cplusplus
}
#endif


#endif