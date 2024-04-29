#include "LoadSwitch.h"

LoadSwitch::LoadSwitch(const gpio_dt_spec _pin) : ctrl_pin(_pin) {}

int LoadSwitch::begin() {
    int ret;

    ret = device_is_ready(ctrl_pin.port);
    if (!ret) {
        printk("Pins not ready.\n");
        return -1;
    }

    ret = gpio_pin_configure_dt(&ctrl_pin, GPIO_OUTPUT_INACTIVE); //GPIO_OUTPUT_INACTIVE
    if (ret != 0) {
        printk("Failed to setup Load Switch.\n");
        return ret;
    }

    return 0;
}

bool LoadSwitch::is_on() {
    int power_on = gpio_pin_get_dt(&ctrl_pin);

    return power_on;
}

void LoadSwitch::set(bool on) {
    if (on) gpio_pin_set_dt(&ctrl_pin, 1);
    else gpio_pin_set_dt(&ctrl_pin, 0);
}