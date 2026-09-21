#include "Button.h"

#include <zephyr/sys/util.h>

#include "button_manager.h"
#include "../Battery/PowerManager.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(button);

//K_WORK_DELAYABLE_DEFINE(Button::button_work, Button::button_work_handler);

struct gpio_callback Button::button_cb_data;

void Button::button_isr(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	Button * button;

	if (pins & BIT(BUTTON_EARABLE)) {
		//earable_btn._read_state();
		button = &earable_btn;
	}

	/*if (pins & BIT(BUTTON_VOLUME_UP)) {
		volume_up_btn._read_state();
	}

	if (pins & BIT(BUTTON_VOLUME_DOWN)) {
		volume_down_btn._read_state();
	}*/

	button->_temp_buttonState = static_cast<button_action>(gpio_pin_get_dt(&(button->button)));

	if (button->_buttonState == button->_temp_buttonState) {
		k_work_cancel_delayable(&(button->button_work));
	} else {
		k_work_reschedule(&(button->button_work), BUTTON_DEBOUNCE);
	}
}

Button::Button(gpio_dt_spec spec) : button(spec) {
    k_work_init_delayable(&button_work, button_work_handler);
}

void Button::begin() {
    int ret;

    if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("Error: button device %s is not ready\n",
		       button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_BOTH);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_isr, button_cb_data.pin_mask | BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	//LOG_INF("Set up button at %s pin %d", button.port->name, button.pin);

	// initial state
	bool reading = gpio_pin_get_dt(&button);

    if (reading) _buttonState = BUTTON_PRESS;
}

void Button::end() {
    gpio_init_callback(&button_cb_data, button_isr, button_cb_data.pin_mask & (~BIT(button.pin)));
	//gpio_remove_callback();
}

void Button::button_work_handler(struct k_work * work) {
	Button *btn = CONTAINER_OF(work, Button, button_work);

	btn->update_state();
}

int Button::update_state() {
	struct button_msg msg;
	int ret;

	_buttonState = _temp_buttonState;

	msg.button_pin = button.pin;
	msg.button_action = _buttonState;

	ret = k_msgq_put(&button_queue, &msg, K_NO_WAIT);
	if (ret) {
		LOG_WRN("Btn msg queue full");
	}

	LOG_INF("Button state: %i", _buttonState);

	return ret;
}

button_action Button::getState() const {
    return _buttonState;
}

Button earable_btn(GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios));
// Button volume_up_btn(GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0}));
// Button volume_down_btn(GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw2), gpios, {0}));
// Button four_btn(GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw3), gpios, {0}));