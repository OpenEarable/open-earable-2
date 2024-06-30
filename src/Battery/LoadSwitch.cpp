#include "LoadSwitch.h"
#include "LoadSwitchPM.h"
#include "PowerManager.h"

LoadSwitch::LoadSwitch(const gpio_dt_spec _pin) : ctrl_pin(_pin) {}

int LoadSwitch::begin() const {
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

bool LoadSwitch::is_on() const {
    int power_on = gpio_pin_get_dt(&ctrl_pin);

    return power_on;
}

void LoadSwitch::set(bool on) const {
    if (on) gpio_pin_set_dt(&ctrl_pin, 1);
    else gpio_pin_set_dt(&ctrl_pin, 0);
}

/*
int dev_pm_control(const struct device *dev,
                           enum pm_device_action action)
{
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        power_manager.v1_8_switch.set(false);
        //printk("switch off\n");
        break;
    case PM_DEVICE_ACTION_RESUME:
        power_manager.v1_8_switch.set(true);
        //printk("switch on\n");
        k_msleep(10);
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

int dev_pm_init(const struct device *dev) {
    power_manager.v1_8_switch.begin();
    return 0;
}*/