#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>  // ✅ Correct Power Management API
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mcu_hook, LOG_LEVEL_DBG);

//#include "bootutil/boot_hooks.h"
//#include "bootutil/mcuboot_status.h"

//#include "nrf5340_audio_common.h"

#include <zephyr/drivers/gpio.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

struct load_switch_data {
    struct gpio_dt_spec ctrl_pin;
    bool default_on;
};

int init_load_switch()
{
    int ret;
    static const struct gpio_dt_spec load_switch_pin = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
        .pin = 11,
        .dt_flags = GPIO_ACTIVE_HIGH
    };

    static const struct gpio_dt_spec ls_3_3_pin = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
        .pin = 14,
        .dt_flags = GPIO_ACTIVE_HIGH
    };

    ret = device_is_ready(load_switch_pin.port);
    if (!ret) {
        printk("Pins not ready.\n");
        return -1;
    }

    ret = device_is_ready(ls_3_3_pin.port);
    if (!ret) {
        printk("Pins not ready.\n");
        return -1;
    }

    printk("setup pins\n");

    ret = gpio_pin_configure_dt(&load_switch_pin, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        printk("Failed to setup Load Switch.\n");
        return ret;
    }

    ret = gpio_pin_configure_dt(&ls_3_3_pin, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        printk("Failed to setup Load Switch.\n");
        return ret;
    }

    k_msleep(10);

    gpio_pin_set_dt(&load_switch_pin, 1);

    k_msleep(1);

    /*ret = gpio_pin_configure_dt(&load_switch_pin, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        printk("Failed to setup Load Switch.\n");
        return ret;
    }*/

    return 0;
}

SYS_INIT(init_load_switch, POST_KERNEL, 80);