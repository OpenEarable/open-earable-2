//#include "LoadSwitch.h"
//#include "PowerManager.h"

#include "nrf5340_audio_common.h"

#include <zephyr/drivers/gpio.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include "LoadSwitch.h"

struct load_switch_data {
    struct gpio_dt_spec ctrl_pin;
};

int generic_pm_control(const struct device *dev, enum pm_device_action action)
{
    struct load_switch_data *data = dev->data;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        /* suspend the device */
        gpio_pin_set_dt(&data->ctrl_pin, 0);
        break;
    case PM_DEVICE_ACTION_RESUME:
        /* resume the device */
        gpio_pin_set_dt(&data->ctrl_pin, 1);
        k_msleep(1); //LS: t_on = 250µs, LDO: 500µs
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

int init_pm_device(const struct device *dev)
{
    struct load_switch_data *data = dev->data;
    int ret;

    ret = device_is_ready(data->ctrl_pin.port);
    if (!ret) {
        printk("Pins not ready.\n");
        return -1;
    }

    ret = gpio_pin_configure_dt(&data->ctrl_pin, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        printk("Failed to setup Load Switch.\n");
        return ret;
    }

    return 0;
}

/*#define PM_DEVICE_DEFINE(node_id, pm_data)                           \
    static struct load_switch_data pm_data = {                         \
        .ctrl_pin = GPIO_DT_SPEC_GET(node_id, gpios),                \
    };                                                               \
    PM_DEVICE_DT_DEFINE(node_id, generic_pm_control);                \
    DEVICE_DT_DEFINE(node_id, init_pm_device, PM_DEVICE_DT_GET(node_id), \
                     &pm_data, NULL, POST_KERNEL, 80, NULL);

PM_DEVICE_DEFINE(DT_NODELABEL(load_switch), load_switch1_data);*/
//PM_DEVICE_DEFINE(DT_NODELABEL(load_switch2), load_switch2_data);

static struct load_switch_data load_switch_1_8 = {
    .ctrl_pin = GPIO_DT_SPEC_GET(load_switch_1_8_id, gpios),
};

static struct load_switch_data load_switch_3_3 = {
    .ctrl_pin = GPIO_DT_SPEC_GET(load_switch_3_3_id, lsctrl_gpios),
};

static struct load_switch_data load_switch_sd_d = {
    .ctrl_pin = GPIO_DT_SPEC_GET(load_switch_sd_id, gpios),
};

PM_DEVICE_DT_DEFINE(load_switch_sd_id, generic_pm_control);
DEVICE_DT_DEFINE(load_switch_sd_id, init_pm_device, PM_DEVICE_DT_GET(load_switch_sd_id),
                    &load_switch_sd_d, NULL, POST_KERNEL, 80, NULL);

PM_DEVICE_DT_DEFINE(load_switch_1_8_id, generic_pm_control);
DEVICE_DT_DEFINE(load_switch_1_8_id, init_pm_device, PM_DEVICE_DT_GET(load_switch_1_8_id),
                    &load_switch_1_8, NULL, POST_KERNEL, 80, NULL);

PM_DEVICE_DT_DEFINE(load_switch_3_3_id, generic_pm_control);
DEVICE_DT_DEFINE(load_switch_3_3_id, init_pm_device, PM_DEVICE_DT_GET(load_switch_3_3_id),
                    &load_switch_3_3, NULL, POST_KERNEL, 80, NULL);