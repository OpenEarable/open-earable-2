#ifndef OPEN_EARABLE_LED_SERVICE_H
#define OPEN_EARABLE_LED_SERVICE_H

#include "LED.h"
#include <zephyr/bluetooth/gatt.h>
#include "../drivers/LED_Controller/KTD2026.h"

#include "zephyr/led_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Initialize the LED controller used by the GATT service. */
int init_led_service();

#ifdef __cplusplus
}
#endif

#endif //OPEN_EARABLE_LED_SERVICE_H
