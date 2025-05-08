#include "StateIndicator.h"
#include "../drivers/LED_Controller/KTD2026.h"

#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>

#include "openearable_common.h"
#include "zbus_common.h"

#include "channel_assignment.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(state_indicator, CONFIG_LOG_DEFAULT_LEVEL);

ZBUS_CHAN_DECLARE(bt_mgmt_chan);
ZBUS_CHAN_DECLARE(battery_chan);

static void connect_evt_handler(const struct zbus_channel *chan)
{
	const struct bt_mgmt_msg *msg;

	msg = (bt_mgmt_msg *) zbus_chan_const_msg(chan);

	switch (msg->event) {
	case BT_MGMT_CONNECTED:
		state_indicator.set_pairing_state(CONNECTED);
		break;

	case BT_MGMT_DISCONNECTED:
		state_indicator.set_pairing_state(PAIRED);
		break;
	}
}

ZBUS_LISTENER_DEFINE(bt_mgmt_evt_listen3, connect_evt_handler); //static

static void power_evt_handler(const struct zbus_channel *chan)
{
	const struct battery_data *msg;

	msg = (battery_data *) zbus_chan_const_msg(chan);

    // LOG_INF("Power Handler %i", (msg->charging_state >> 5) & 0x3);

    // int state = (msg->charging_state >> 5) & 0x3;

    state_indicator.set_charging_state(msg->charging_state);

	/*switch ((msg->charging_state >> 5) & 0x3) {
    case 2:
        LOG_INF("charging state: discharge");
        state_indicator.set_charging_state(DISCHARGING);
        break;
    case 1:
        LOG_INF("charging state: charging");
        state_indicator.set_charging_state(CHARGING);
        break;
    case 3:
        LOG_INF("charging state: done");
        state_indicator.set_charging_state(FULLY_CHARGED);
        break;
    //case 3:
        //LOG_WRN("charging state: fault");
        //state_indicator.set_state(FAULT);

        //uint16_t ts_fault = battery_controller.read_ts_fault();
        //LOG_WRN("TS_ENABLED: %i, TS FAULT: %i", ts_fault >> 7, (ts_fault >> 5) & 0x3);

        //battery_controller.setup();
        
        //break;
    }*/
}

ZBUS_LISTENER_DEFINE(power_evt_listen, power_evt_handler); //static

void StateIndicator::init(struct earable_state state) {
    int ret;

    led_controller.begin();

    ret = zbus_chan_add_obs(&bt_mgmt_chan, &bt_mgmt_evt_listen3, ZBUS_ADD_OBS_TIMEOUT_MS);
    if (ret && ret != -EALREADY) {
		LOG_ERR("Failed to add bt_mgmt listener");
	}

    ret = zbus_chan_add_obs(&battery_chan, &power_evt_listen, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret && ret != -EALREADY) {
		LOG_ERR("Failed to add battery listener");
	}

    set_state(state);
}

void StateIndicator::set_custom_color(RGBColor color) {
    memcpy(&this->color, color, sizeof(RGBColor));
    if (_state.led_mode == CUSTOM) led_controller.setColor(color);
}

void StateIndicator::set_indication_mode(enum led_mode state) {
    _state.led_mode = state;
    set_state(_state);
}

void StateIndicator::set_charging_state(enum charging_state state) {
    _state.charging_state = state;
    set_state(_state);
}

void StateIndicator::set_pairing_state(enum pairing_state state) {
    _state.pairing_state = state;
    set_state(_state);
}

void StateIndicator::set_state(struct earable_state state) {
    _state = state;

    // do not update the state if set to custom color
    if (_state.led_mode == CUSTOM) {
        led_controller.setColor(color);
        return;
    }

    RGBColor color = {0,0,0};
    switch (_state.charging_state) {
    case POWER_CONNECTED:
        color[0] = 24; // Rot
        color[1] = 8;  // Grün
        color[2] = 0;  // Blau

        led_controller.setColor(color);
        break;
    case CHARGING:
        color[0] = 24; // Rot
        color[1] = 8;  // Grün
        color[2] = 0;  // Blau

        led_controller.pulse(color, 1000, 1000, 512, 2000);
        break;
    case PRECHARGING:
        color[0] = 32; // Rot
        color[1] = 0;  // Grün
        color[2] = 0;  // Blau

        led_controller.pulse(color, 1000, 1000, 512, 2000);
        break;
    case FULLY_CHARGED:
        color[0] = 0;   // Rot
        color[1] = 32;  // Grün
        color[2] = 0;   // Blau

        led_controller.setColor(color);
        break;
    case FAULT:
        color[0] = 32; // Rot
        color[1] = 0;  // Grün
        color[2] = 0;  // Blau

        led_controller.setColor(color);
        break;
    default:
        switch (_state.pairing_state) {
        case SET_PAIRING:
            audio_channel channel;
            channel_assignment_get(&channel);
            if (channel == AUDIO_CH_L) {
                color[0] = 0;   // Rot
                color[1] = 0;   // Grün
                color[2] = 16;  // Blau
            } else  if (channel == AUDIO_CH_R) {
                color[0] = 16;   // Rot
                color[1] = 0;   // Grün
                color[2] = 0;  // Blau
            }

            led_controller.blink(color, 100, 200);
            break;
        case BONDING:
            color[0] = 0;   // Rot
            color[1] = 0;   // Grün
            color[2] = 16;  // Blau

            led_controller.blink(color, 100, 500);
            break;
        case PAIRED:
            color[0] = 0;   // Rot
            color[1] = 0;   // Grün
            color[2] = 16;  // Blau 

            led_controller.blink(color, 100, 2000);
            break;
        case CONNECTED:
            color[0] = 0;   // Rot
            color[1] = 16;  // Grün
            color[2] = 0;   // Blau

            led_controller.blink(color, 100, 2000);
            break;
        }
    }
}

StateIndicator state_indicator;