//#include "PowerManager.h"

#include "openearable_common.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(battery_pub, CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL);

struct load_switch_data {
    struct gpio_dt_spec ctrl_pin;
    bool default_on;
};

/*static int b_init(const struct device *dev)
{
    ARG_UNUSED(dev);

    struct load_switch_data *data_1_8 = dev->data;

    if(data_1_8->default_on) {
        int ret = pm_device_action_run(ls_1_8, PM_DEVICE_ACTION_SUSPEND);
        if (ret < 0) {
            printk("Failed to suspend device: %d", ret);
        }
    }

    return 0;
}

SYS_INIT(b_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);*/