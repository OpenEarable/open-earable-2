#ifndef _POWER_SWITCH_H
#define _POWER_SWITCH_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

class PowerSwitch {
public:
    int begin();

    int set_wakeup_int();
    bool is_on();

    int set_power_off_callback(gpio_callback_handler_t handler);
private:
    /*const struct gpio_dt_spec power_switch_pin = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
        .pin = 12, //11
        .dt_flags = GPIO_ACTIVE_HIGH
    };*/

    //const struct gpio_dt_spec power_switch_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(power_switch), gpios);
    const struct gpio_dt_spec power_switch_pin = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(power_switch), gpios, {0});
    const struct gpio_dt_spec button_pin = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(button0), gpios, {0});

    gpio_callback power_switch_cb_data;

    //static void power_switch_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
};

extern PowerSwitch power_switch;

#endif