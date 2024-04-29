#ifndef _LOAD_SWITCH_H
#define _LOAD_SWITCH_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

class LoadSwitch {
public:
    LoadSwitch(const gpio_dt_spec _pin);

    int begin();
    void set(bool on);
    bool is_on();
private:
    const gpio_dt_spec ctrl_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(load_switch), gpios);
};

#endif