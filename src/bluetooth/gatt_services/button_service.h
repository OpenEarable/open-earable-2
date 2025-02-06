//#pragma once

#ifndef BUTTON_SERVICE_H
#define BUTTON_SERVICE_H

#include <zephyr/bluetooth/gatt.h>
#include "openearable_common.h"
#include "zbus_common.h"

#define BT_UUID_BUTTON_VAL \
	BT_UUID_128_ENCODE(0x29c10bdc, 0x4773, 0x11ee, 0xbe56, 0x0242ac120002)

/** @brief LED Characteristic UUID. */
#define BT_UUID_BUTTON_STATE_VAL \
    BT_UUID_128_ENCODE(0x29c10f38, 0x4773, 0x11ee, 0xbe56, 0x0242ac120002)

#define BT_UUID_BUTTON             BT_UUID_DECLARE_128(BT_UUID_BUTTON_VAL)
#define BT_UUID_BUTTON_STATE       BT_UUID_DECLARE_128(BT_UUID_BUTTON_STATE_VAL)

#ifdef __cplusplus
extern "C" {
#endif

int init_button_service();
int bt_send_button_state(enum button_action _button_state);

#ifdef __cplusplus
}
#endif

#endif