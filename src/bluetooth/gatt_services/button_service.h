//#pragma once

#ifndef BUTTON_SERVICE_H
#define BUTTON_SERVICE_H

#include <zephyr/bluetooth/gatt.h>
#include "openearable_common.h"
#include "zbus_common.h"

#include "zephyr/button_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Start forwarding button events to the GATT service. */
int init_button_service();
/** @brief Store the latest button action and notify subscribed peers. */
int bt_send_button_state(enum button_action _button_state);

#ifdef __cplusplus
}
#endif

#endif