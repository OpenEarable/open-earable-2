#ifndef OPEN_EARABLE_LED_SERVICE_H
#define OPEN_EARABLE_LED_SERVICE_H

#include "LED.h"
#include <zephyr/bluetooth/gatt.h>
#include "../drivers/LED_Controller/KTD2026.h"

#define BT_UUID_LED_VAL \
	BT_UUID_128_ENCODE(0x81040a2e, 0x4819, 0x11ee, 0xbe56, 0x0242ac120002)

/** @brief LED Characteristic UUID. */
#define BT_UUID_LED_RGB_VAL \
    BT_UUID_128_ENCODE(0x81040e7a, 0x4819, 0x11ee, 0xbe56, 0x0242ac120002)

#define BT_UUID_LED_STATE_VAL \
    BT_UUID_128_ENCODE(0x81040e7b, 0x4819, 0x11ee, 0xbe56, 0x0242ac120002)

#define BT_UUID_LED           BT_UUID_DECLARE_128(BT_UUID_LED_VAL)
#define BT_UUID_LED_RGB       BT_UUID_DECLARE_128(BT_UUID_LED_RGB_VAL)
#define BT_UUID_LED_STATE       BT_UUID_DECLARE_128(BT_UUID_LED_STATE_VAL)

#ifdef __cplusplus
extern "C" {
#endif

int init_led_service();

#ifdef __cplusplus
}
#endif

#endif //OPEN_EARABLE_LED_SERVICE_H
