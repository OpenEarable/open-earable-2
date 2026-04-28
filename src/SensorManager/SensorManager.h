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

/**
 * @brief Get the number of currently active (running) sensors.
 *
 * Sensor Manager internally keeps track of the number of sensors it has
 * started and stopped.
 *
 * @return Number of sensors currently running.
 */
int sensor_manager_get_active_sensor_count(void);

#ifdef __cplusplus
}
#endif

#endif