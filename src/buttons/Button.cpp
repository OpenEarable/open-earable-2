#include "Button.h"

#include "button_manager.h"

struct gpio_callback Button::button_cb_data;

const struct gpio_dt_spec Button::buttons[] = {
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0}), // BUTTON_VOLUME_DOWN
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0}), // BUTTON_VOLUME_UP
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw2), gpios, {0}), // BUTTON_PLAY_PAUSE
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw3), gpios, {0}), // BUTTON_4
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw4), gpios, {0}), // BUTTON_5
};

void Button::button_isr(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	if (pins & BIT(BUTTON_PLAY_PAUSE)) {
		earable_btn._read_state();
	}

	if (pins & BIT(BUTTON_VOLUME_UP)) {
		volume_up_btn._read_state();
	}

	if (pins & BIT(BUTTON_VOLUME_DOWN)) {
		volume_down_btn._read_state();
	}
}

Button::Button(gpio_dt_spec spec, bool inverted) : _inverted(inverted), button(spec) {
    
}

void Button::begin() {
    int ret;

    if (!gpio_is_ready_dt(&button)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_BOTH);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_isr, button_cb_data.pin_mask | BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	// initial state
	bool reading = gpio_pin_get_dt(&button);

    //if (_inverted) reading = !reading;
    reading = _inverted ^ reading;

    if (reading) _buttonState = BUTTON_PRESS;

	//printk("mask:%i\n", button_cb_data.pin_mask);
}

void Button::end() {
    gpio_init_callback(&button_cb_data, button_isr, button_cb_data.pin_mask & (~BIT(button.pin)));
	//gpio_remove_callback();
}

void Button::inverted(bool _inverted) {
    this->_inverted = _inverted;
}

void Button::_read_state() {
	int ret;

    bool reading = gpio_pin_get_dt(&button);

    //if (_inverted) reading = !reading;
    reading = _inverted ^ reading;

    //CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER
    unsigned long now = k_cyc_to_ms_floor32(k_cycle_get_32());

    if (!reading) {
        if (_buttonState != BUTTON_RELEASED) {
            //button_service.write_state(RELEASED);
            //printk("Button released at %" PRIu32 "\n", now);

			_buttonState = BUTTON_RELEASED;
			_lastDebounceTime = now;

			ret = update_state();
        }
        _buttonState = BUTTON_RELEASED;
        _lastDebounceTime = now;
		
        return;
    }

    if (now - _lastDebounceTime < _debounceDelay) return;
    _buttonState = BUTTON_PRESS;
    //button_service.write_state(_buttonState);
    //printk("Button pressed at %" PRIu32 "\n", now);

	ret = update_state();
}

int Button::update_state() {
	struct button_msg msg;
	int ret;

	msg.button_pin = button.pin;
	msg.button_action = _buttonState;

	ret = k_msgq_put(&button_queue, &msg, K_NO_WAIT);
	if (ret == -EAGAIN) {
		//LOG_WRN("Btn msg queue full");
		printk("Btn msg queue full");
	}

	return ret;
}

button_action Button::getState() const {
    return _buttonState;
}

void Button::setDebounceTime(unsigned long debounceTime) {
    _debounceDelay = debounceTime;
}

/*Button earable_btn(DT_GPIO_PIN(DT_ALIAS(sw0), gpios));
Button volume_up_btn(DT_GPIO_PIN(DT_ALIAS(sw1), gpios));
Button volume_down_btn(DT_GPIO_PIN(DT_ALIAS(sw2), gpios));
Button four_btn(DT_GPIO_PIN(DT_ALIAS(sw3), gpios));*/
//Button five_btn(BUTTON_5);

Button earable_btn(GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0}));
Button volume_up_btn(GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0}));
Button volume_down_btn(GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw2), gpios, {0}));
Button four_btn(GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw3), gpios, {0}));