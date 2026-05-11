#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

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

    // Initialize LED on first call
    if (!led_initialized) {
        if (device_is_ready(led.port)) {
            gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
            led_initialized = true;
        }
        k_timer_init(&blink_timer, blink_timer_handler, NULL);
    }

    /*
     * The printk() calls below are kept (commented out) for future debugging.
     *
     * NOTE ON MCUBOOT CONSOLE LOGGING (RTT & USB CDC ACM):
     * MCUboot executes extremely fast. By default, you will NOT see printk logs
     * in RTT or USB UART, because of a timing race condition with the host PC:
     *
     * 1. RTT: MCUboot and the App have separate RTT control blocks in RAM. The
     *    J-Link Viewer searches RAM for the block, but MCUboot usually jumps to
     *    the App before J-Link can find and connect to MCUboot's RTT block.
     * 2. USB UART (CDC ACM): Enumerable USB takes ~1-2 seconds. MCUboot finishes
     *    executing long before the PC creates /dev/ttyACM0.
     *
     * HOW TO VIEW THESE PRINTS:
     * - RTT / USB CDC ACM: You must configure the backend in sysbuild/mcuboot/prj.conf,
     *   AND you must add a blocking delay (e.g., k_msleep(3000);) right here at
     *   the STARTUP case to give your PC/Viewer time to connect before MCUboot exits.
     * - Hardware UART: The most reliable method. Route CONFIG_UART_CONSOLE=y to
     *   physical TTL pins. It is stateless and does not require artificial delays.
     */
    // printk("mcuboot_status_change: %d\n", status);

    switch (status) {
        case MCUBOOT_STATUS_STARTUP:
            // printk("  STARTUP\n");
            k_timer_start(&blink_timer, K_MSEC(50), K_MSEC(50));
            break;
        case MCUBOOT_STATUS_UPGRADING:
            // printk("  UPGRADING\n");
            k_timer_start(&blink_timer, K_MSEC(250), K_MSEC(250));
            break;
        case MCUBOOT_STATUS_BOOTABLE_IMAGE_FOUND:
            // printk("  BOOTABLE_IMAGE_FOUND\n");
            k_timer_stop(&blink_timer);
            gpio_pin_set_dt(&led, 0);
            break;
        case MCUBOOT_STATUS_NO_BOOTABLE_IMAGE_FOUND:
            // printk("  NO_BOOTABLE_IMAGE_FOUND\n");
            k_timer_stop(&blink_timer);
            gpio_pin_set_dt(&led, 1);
            break;
        case MCUBOOT_STATUS_BOOT_FAILED:
            // printk("  BOOT_FAILED\n");
            k_timer_stop(&blink_timer);
            gpio_pin_set_dt(&led, 1);
            break;
        case MCUBOOT_STATUS_USB_DFU_WAITING:
            // printk("  USB_DFU_WAITING\n");
            break;
        case MCUBOOT_STATUS_USB_DFU_ENTERED:
            // printk("  USB_DFU_ENTERED\n");
            break;
        case MCUBOOT_STATUS_USB_DFU_TIMED_OUT:
            // printk("  USB_DFU_TIMED_OUT\n");
            break;
        case MCUBOOT_STATUS_SERIAL_DFU_ENTERED:
            // printk("  SERIAL_DFU_ENTERED\n");
            break;
    }
}

int init_load_switch()
{
    int ret;

    // V_LS 1.8v is required for flash.
    static const struct gpio_dt_spec load_switch_pin = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
        .pin = 11,
        .dt_flags = GPIO_ACTIVE_HIGH
    };

    // LS 3.3v is required for the Error LED.
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
        printk("Failed to setup 1.8V load switch.\n");
        return ret;
    }

    ret = device_is_ready(ls_3_3_pin.port);
    if (!ret) {
        printk("Pins not ready.\n");
        return -1;
    }

    ret = gpio_pin_configure_dt(&ls_3_3_pin, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) {
        printk("Failed to setup 3.3V load switch.\n");
        return ret;
    }

    return 0;
}

SYS_INIT(init_load_switch, PRE_KERNEL_2, 80);