#include "PowerSwitch.h"

PowerSwitch power_switch;

int PowerSwitch::begin() {
    int ret;

    ret = gpio_is_ready_dt(&power_switch_pin); //bool
    if (!ret) {
            printk("Power switch not ready.\n");
            return -1;
    }

    ret = gpio_pin_configure_dt(&power_switch_pin, GPIO_INPUT | GPIO_PULL_DOWN);
	if (ret != 0) {
        printk("Failed to set P0.10 as input.\n");
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&power_switch_pin, GPIO_INT_EDGE_TO_INACTIVE);
    if (ret != 0) {
        printk("Failed to setup interrupt on power switch.\n");
        return ret;
    }

    return 0;
}

int PowerSwitch::set_wakeup_int() {
    int ret;

    ret = gpio_is_ready_dt(&power_switch_pin); //bool
    if (!ret) {
            printk("Power switch not ready.\n");
            return -1;
    }

    ret = gpio_pin_interrupt_configure_dt(&power_switch_pin, GPIO_INT_LEVEL_ACTIVE);
    if (ret != 0) {
        printk("Failed to setup interrupt on power switch.\n");
        return ret;
    }

    return 0;
}

bool PowerSwitch::is_on() {
    int power_on = gpio_pin_get_dt(&power_switch_pin);

    return power_on;
}

int PowerSwitch::set_power_off_callback(gpio_callback_handler_t handler) {
    gpio_init_callback(&power_switch_cb_data, handler, power_switch_cb_data.pin_mask | BIT(power_switch_pin.pin));
    return gpio_add_callback(power_switch_pin.port, &power_switch_cb_data);
}