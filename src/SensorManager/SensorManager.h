#ifndef _SENSOR_MANAGER_H
#define _SENSOR_MANAGER_H

#include "openearable_common.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#ifdef __cplusplus
extern "C" {
#endif

enum sensor_manager_state {
    INIT,
    RUNNING,
    SUSPENDED,
};

extern struct k_work_q sensor_work_q;

enum sensor_manager_state get_state();

void init_sensor_manager();

void start_sensor_manager();

void stop_sensor_manager();

void config_sensor(struct sensor_config * config);

#ifdef __cplusplus
}
#endif

#endif