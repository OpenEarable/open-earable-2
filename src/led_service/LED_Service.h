#ifndef OPEN_EARABLE_LED_SERVICE_H
#define OPEN_EARABLE_LED_SERVICE_H

#include "LED.h"
#include <zephyr/bluetooth/gatt.h>

#define BT_UUID_LBS_VAL \
	BT_UUID_128_ENCODE(0x81040a2e, 0x4819, 0x11ee, 0xbe56, 0x0242ac120002)

/** @brief LED Characteristic UUID. */
#define BT_UUID_LBS_LED_VAL \
    BT_UUID_128_ENCODE(0x81040a7a, 0x4819, 0x11ee, 0xbe56, 0x0242ac120002)

#define BT_UUID_LBS           BT_UUID_DECLARE_128(BT_UUID_LBS_VAL)
#define BT_UUID_LBS_LED       BT_UUID_DECLARE_128(BT_UUID_LBS_LED_VAL)

class LED_Service {
public:
    void begin();

    //void static receiveState(BLEDevice central, BLECharacteristic characteristic);
/*private:
    BLEService * _ledService{};
    BLECharacteristic * _ledSetStateC{};*/
};

extern LED_Service led_service;

#endif //OPEN_EARABLE_LED_SERVICE_H
