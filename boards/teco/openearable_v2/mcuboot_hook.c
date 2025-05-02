#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/gpio.h>

int init_load_switch()
{
    int ret;
    static const struct gpio_dt_spec load_switch_pin = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
        .pin = 11,
        .dt_flags = GPIO_ACTIVE_HIGH
    };

    ret = device_is_ready(load_switch_pin.port);
    if (!ret) {
        printk("Pins not ready.\n");
        return -1;
    }

    //LOG_DBG("Turn on loadswitch for flash.");

    ret = gpio_pin_configure_dt(&load_switch_pin, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        printk("Failed to setup Load Switch.\n");
        return ret;
    }

    return 0;
}

SYS_INIT(init_load_switch, PRE_KERNEL_2, 80);

/*
int wait() {
    k_msleep(1);

    return 0;
}

SYS_INIT(wait, POST_KERNEL, 80);*/