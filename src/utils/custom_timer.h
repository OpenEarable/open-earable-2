#ifndef CUSTOM_TIMER_H
#define CUSTOM_TIMER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes and starts the custom timer.
 *
 * Configures the hardware timer, sets up the overflow interrupt, and starts
 * the timer for microsecond tracking.
 */
void custom_timer_start(void);

/**
 * @brief Returns the current time in microseconds.
 *
 * This function captures the current timer value and combines it with
 * an overflow counter to provide a 64-bit microsecond count.
 *
 * @return uint64_t The current time in microseconds.
 */
uint64_t custom_micros(void);

/**
 * @brief Returns the current time in milliseconds.
 *
 * This function converts the microsecond count provided by custom_micros()
 * into milliseconds.
 *
 * @return uint64_t The current time in milliseconds.
 */
uint64_t custom_millis(void);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_TIMER_H */
