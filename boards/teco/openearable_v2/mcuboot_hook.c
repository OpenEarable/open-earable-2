#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

//#include "bootutil/bootutil.h"
#include "bootutil/boot_hooks.h"
#include "bootutil/mcuboot_status.h"

#include <zephyr/drivers/gpio.h>

#define LED_NODE DT_ALIAS(mcuboot_led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static struct k_timer blink_timer;

static void blink_timer_handler(struct k_timer *timer) {
    static bool led_state = false;
    gpio_pin_set_dt(&led, led_state);
    led_state = !led_state;
}

void mcuboot_status_change(mcuboot_status_type_t status)
{
    static bool led_initialized = false;

    // Initialisiere LED beim ersten Aufruf
    if (!led_initialized) {
        if (device_is_ready(led.port)) {
            gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
            led_initialized = true;
        }
    }

    switch (status) {
        case MCUBOOT_STATUS_STARTUP:
            //state_indicator.set_state(STARTUP);
            break;
        case MCUBOOT_STATUS_UPGRADING:
            k_timer_init(&blink_timer, blink_timer_handler, NULL);
            k_timer_start(&blink_timer, K_MSEC(250), K_MSEC(250));
            break;
        case MCUBOOT_STATUS_BOOTABLE_IMAGE_FOUND:
            k_timer_stop(&blink_timer);
            gpio_pin_set_dt(&led, 0);
            break;
        case MCUBOOT_STATUS_NO_BOOTABLE_IMAGE_FOUND:
            k_timer_stop(&blink_timer);
            gpio_pin_set_dt(&led, 1);
            break;
        case MCUBOOT_STATUS_BOOT_FAILED:
            //state_indicator.set_state(BOOT_FAILED);
            break;
        case MCUBOOT_STATUS_USB_DFU_WAITING:
            //state_indicator.set_state(USB_DFU_WAITING);
            break;
        case MCUBOOT_STATUS_USB_DFU_ENTERED:
            //state_indicator.set_state(USB_DFU_ENTERED);
            break;
        case MCUBOOT_STATUS_USB_DFU_TIMED_OUT:
            //state_indicator.set_state(USB_DFU_TIMED_OUT);
            break;
        case MCUBOOT_STATUS_SERIAL_DFU_ENTERED:
            //state_indicator.set_state(SERIAL_DFU_ENTERED);
            break;
    }
}

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

    ret = gpio_pin_configure_dt(&load_switch_pin, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        printk("Failed to setup Load Switch.\n");
        return ret;
    }

    ret = gpio_pin_configure_dt(&ls_3_3_pin, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        printk("Failed to setup 3.3V.\n");
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