#ifndef _LOAD_SWITCH_H
#define _LOAD_SWITCH_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

class LoadSwitch {
public:
    LoadSwitch(const gpio_dt_spec _pin);

    int begin() const;
    void set(bool on) const;
    bool is_on() const;
private:
    const gpio_dt_spec ctrl_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(load_switch), gpios);
};

#endif