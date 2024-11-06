#include "StateIndicator.h"
#include "../drivers/LED_Controller/KTD2026.h"

#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>

#include "nrf5340_audio_common.h"

ZBUS_CHAN_DECLARE(bt_mgmt_chan);

static void connect_evt_handler(const struct zbus_channel *chan)
{
	const struct bt_mgmt_msg *msg;

	msg = (bt_mgmt_msg *) zbus_chan_const_msg(chan);

	switch (msg->event) {
	case BT_MGMT_CONNECTED:
		state_indicator.set_state(CONNECTED);
		break;

	case BT_MGMT_DISCONNECTED:
		state_indicator.set_state(PAIRED);
		break;
	}
}

ZBUS_LISTENER_DEFINE(bt_mgmt_evt_listen3, connect_evt_handler); //static

void StateIndicator::init(enum earable_state state) {
    int ret = zbus_chan_add_obs(&bt_mgmt_chan, &bt_mgmt_evt_listen3, ZBUS_ADD_OBS_TIMEOUT_MS);
    set_state(state);
	/*if (ret) {
		LOG_ERR("Failed to add bt_mgmt listener");
	}*/
}

void StateIndicator::set_state(enum earable_state state) {

    RGBColor color = {0,0,0};
    switch (state) {
    case UNPAIRED:
        color[0] = 0;    // Rot
        color[1] = 0;    // Grün
        color[2] = 16;  // Blau

        led_controller.blink(color, 100, 500);
        break;
    case PAIRED:
        color[0] = 0;    // Rot
        color[1] = 0;    // Grün
        color[2] = 16;  // Blau 

        led_controller.blink(color, 100, 2000);
        break;
    case CONNECTED:
        color[0] = 0;    // Rot
        color[1] = 16;  // Grün
        color[2] = 0;    // Blau

        led_controller.blink(color, 100, 2000);
        break;
    }
}

StateIndicator state_indicator;