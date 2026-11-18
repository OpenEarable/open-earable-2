#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Get time since boot in microseconds.
 * @return Time since boot in microseconds.
 */
uint64_t get_time_since_boot_us(void);

/**
 * @brief Get the current synchronized time in microseconds (since 1. January 1970).
 * @return Current synchronized time in microseconds.
 */
uint64_t get_current_time_us(void);

/**
 * @brief Check whether the wall-clock time has been synchronized.
 * @return True after the Time Offset characteristic has been written.
 */
bool time_sync_is_synced(void);

/**
 * @brief Initialize the time synchronization module.
 *
 * @return 0 on success, negative error code on failure.
 */
int init_time_sync();

#ifdef __cplusplus
}
#endif
