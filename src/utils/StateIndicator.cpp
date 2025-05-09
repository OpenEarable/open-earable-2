#include "StateIndicator.h"
#include "../drivers/LED_Controller/KTD2026.h"

#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>

#include "openearable_common.h"
#include "zbus_common.h"

#include "channel_assignment.h"


#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>

#include "bootutil/boot_hooks.h"
#include "bootutil/mcuboot_status.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(state_indicator, CONFIG_LOG_DEFAULT_LEVEL);

ZBUS_CHAN_DECLARE(bt_mgmt_chan);
ZBUS_CHAN_DECLARE(battery_chan);

struct mgmt_callback mcu_mgr_cb;

enum mgmt_cb_return chuck_write_indication(uint32_t event, enum mgmt_cb_return prev_status,
                                int32_t *rc, uint16_t *group, bool *abort_more,
                                void *data, size_t data_size)
{
    if (event == MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK) {
        /* This is the event we registered for */
		led_controller.setColor(LED_ORANGE);
        k_msleep(10);
        led_controller.setColor(LED_OFF);
    }
    /*else if (event == MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK_WRITE_COMPLETE) {
        led_controller.setColor(LED_OFF);
    }
    else if (event == MGMT_EVT_OP_OS_MGMT_RESET) {
		LOG_INF("RESET received");
	}*/

	//LOG_INF("my_function called with event: %d", event);

    /* Return OK status code to continue with acceptance to underlying handler */
    return MGMT_CB_OK;
}

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

    state_indicator.set_charging_state(msg->charging_state);
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

	mcu_mgr_cb.callback = chuck_write_indication;
    mcu_mgr_cb.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK; //MGMT_EVT_OP_IMG_MGMT_ALL
    mgmt_callback_register(&mcu_mgr_cb);

    set_state(state);
}

void StateIndicator::set_custom_color(const RGBColor &color) {
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

    switch (_state.charging_state) {
    case POWER_CONNECTED:
        led_controller.setColor(LED_ORANGE);
        break;
    case CHARGING:
        led_controller.pulse(LED_ORANGE, 1000, 1000, 512, 2000);
        break;
    case PRECHARGING:
        led_controller.pulse(LED_RED, 1000, 1000, 512, 2000);
        break;
    case TRICKLE_CHARGING:
        led_controller.pulse(LED_GREEN, 1000, 1000, 512, 2000);
        break;
    case FULLY_CHARGED:
        led_controller.setColor(LED_GREEN);
        break;
    case FAULT:
        led_controller.setColor(LED_RED);
        break;
    case BATTERY_CRITICAL:
        led_controller.blink(LED_RED, 100, 2000);
        break;
    case BATTERY_LOW:
        led_controller.blink(LED_ORANGE, 100, 2000);
        break;
    default:
        switch (_state.pairing_state) {
        case SET_PAIRING:
            audio_channel channel;
            channel_assignment_get(&channel);
            if (channel == AUDIO_CH_L) {
                led_controller.blink(LED_BLUE, 100, 200);
            } else  if (channel == AUDIO_CH_R) {
                led_controller.blink(LED_RED, 100, 200);
            }
            break;
        case BONDING:
            led_controller.blink(LED_BLUE, 100, 500);
            break;
        case PAIRED:
            led_controller.blink(LED_BLUE, 100, 2000);
            break;
        case CONNECTED:
            led_controller.blink(LED_GREEN, 100, 2000);
            break;
        }
    }
}

StateIndicator state_indicator;