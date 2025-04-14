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

void battery_chan_update() {
    int ret;

	while (1) {
		k_msgq_get(&battery_queue, &pm_msg, K_FOREVER);

		ret = zbus_chan_pub(&battery_chan, &pm_msg, K_FOREVER); //K_NO_WAIT
		if (ret) {
			LOG_ERR("Failed to publish battery msg, ret: %d", ret);
		}
	}
}

K_THREAD_DEFINE(battery_publish, 1024, battery_chan_update, NULL, NULL, //CONFIG_BUTTON_PUBLISH_STACK_SIZE
		NULL, K_PRIO_PREEMPT(CONFIG_BUTTON_PUBLISH_THREAD_PRIO), 0, 0); //CONFIG_BUTTON_PUBLISH_THREAD_PRIO


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