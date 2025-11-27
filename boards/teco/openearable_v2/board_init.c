#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>  // ✅ Correct Power Management API
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(board_init, LOG_LEVEL_DBG);

//#include "nrf5340_audio_common.h"

#include <zephyr/drivers/gpio.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#define load_switch_sd_id DT_NODELABEL(load_switch_sd)
#define load_switch_1_8_id DT_NODELABEL(load_switch)
#define load_switch_3_3_id DT_CHILD(DT_NODELABEL(bq25120a), load_switch)
//#define load_switch_3_3_id DT_NODELABEL(lsctrl)

const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
const struct device *const ls_1_8 = DEVICE_DT_GET(load_switch_1_8_id);
const struct device *const ls_3_3 = DEVICE_DT_GET(load_switch_3_3_id);
const struct device *const ls_sd = DEVICE_DT_GET(load_switch_sd_id);

struct load_switch_data {
    struct gpio_dt_spec ctrl_pin;
    int delay_us;
    bool default_on;
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
        k_usleep(data->delay_us); //LS: t_on = 250µs, LDO: 500µs
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

    ret = gpio_pin_configure_dt(&data->ctrl_pin, data->default_on ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        printk("Failed to setup Load Switch.\n");
        return ret;
    }

    return 0;
}

static struct load_switch_data load_switch_1_8 = {
    .ctrl_pin = GPIO_DT_SPEC_GET(load_switch_1_8_id, enable_gpios),
    .default_on = DT_NODE_HAS_PROP(load_switch_1_8_id, default_on),
    .delay_us = DT_PROP(load_switch_1_8_id, power_delay_us)
};

static struct load_switch_data load_switch_3_3 = {
    .ctrl_pin = GPIO_DT_SPEC_GET(load_switch_3_3_id, enable_gpios),
    .default_on = DT_NODE_HAS_PROP(load_switch_3_3_id, default_on),
    .delay_us = DT_PROP(load_switch_3_3_id, power_delay_us),
};

static struct load_switch_data load_switch_sd_d = {
    .ctrl_pin = GPIO_DT_SPEC_GET(load_switch_sd_id, enable_gpios),
    .default_on = DT_NODE_HAS_PROP(load_switch_sd_id, default_on),
    .delay_us = DT_PROP(load_switch_sd_id, power_delay_us),
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
