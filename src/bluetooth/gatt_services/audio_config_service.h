#ifndef _AUDIO_CONFIG_SERVICE_H_
#define _AUDIO_CONFIG_SERVICE_H_

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "zephyr/audio_configuration_ble.h"

/** @brief Initialize the codec to normal audio mode. */
int init_audio_config_service(void);

#endif /* _AUDIO_CONFIG_SERVICE_H_ */
