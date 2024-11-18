#ifndef _SENSOR_MANAGER_H
#define _SENSOR_MANAGER_H

#include "nrf5340_audio_common.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#ifdef __cplusplus
extern "C" {
#endif

void start_sensor_manager();

void stop_sensor_manager();

void config_sensor(struct sensor_config * config);

#ifdef __cplusplus
}
#endif

#endif