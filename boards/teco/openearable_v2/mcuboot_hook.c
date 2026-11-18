#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

//#include "bootutil/bootutil.h"
#include "bootutil/boot_hooks.h"
#include "bootutil/mcuboot_status.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <hal/nrf_power.h>
#include "boot_charger_status.h"

#define LED_NODE DT_ALIAS(mcuboot_led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static struct k_timer blink_timer;
static struct k_work_delayable charger_watchdog_work;
static const struct device *const charger_i2c = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(bq25120a)));
/* MCUboot does not include the application's custom devicetree bindings. */
static const struct gpio_dt_spec ls_3_3_pin = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)), .pin = 14, .dt_flags = GPIO_ACTIVE_HIGH
};
static const struct gpio_dt_spec charger_cd_pin = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)), .pin = 17, .dt_flags = GPIO_ACTIVE_HIGH
};

#define CHARGER_ADDR DT_REG_ADDR(DT_NODELABEL(bq25120a))
#define CHARGER_LDO_CTRL 0x07
#define CHARGER_LDO_MASK 0x7c
#define CHARGER_LDO_3V3 (25U << 2)

/* Reading 0x07 also consumes the adjacent button flags on this charger.
 * Receive both bytes, and retain the wake reason for the application. */
static int read_charger_ldo(uint8_t *status)
{
    uint8_t control;
    uint8_t registers[2];
    int ret = i2c_reg_read_byte(charger_i2c, CHARGER_ADDR, 0x00, &control);
    if (ret) return ret;
    k_busy_wait(100);
    ret = i2c_burst_read(charger_i2c, CHARGER_ADDR, CHARGER_LDO_CTRL,
                         registers, sizeof(registers));
    if (ret) return ret;
    boot_charger_save(&NRF_POWER->GPREGRET[1],
                      (registers[1] & 0x03) | ((control & 0x10) >> 2));
    *status = registers[0];
    return 0;
}

static void charger_watchdog_handler(struct k_work *work)
{
    uint8_t status;
    ARG_UNUSED(work);
    /* Any transaction feeds the charger watchdog during long image swaps. */
    int ret = read_charger_ldo(&status);
    if (ret || (status & CHARGER_LDO_MASK) != CHARGER_LDO_3V3) {
        gpio_pin_set_dt(&ls_3_3_pin, 0);
        if (!ret && (status & 0x80)) {
            k_busy_wait(100);
            i2c_reg_write_byte(charger_i2c, CHARGER_ADDR, CHARGER_LDO_CTRL, status & ~0x80);
        }
        printk("3.3 V regulator lost configuration.\n");
        return;
    }
    k_work_schedule(&charger_watchdog_work, K_SECONDS(10));
}

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

    ret = gpio_pin_configure_dt(&ls_3_3_pin, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        printk("Failed to setup 3.3V.\n");
        return ret;
    }

    /* Inhibit charging on USB and wake I2C when running from the battery. */
    ret = gpio_pin_configure_dt(&charger_cd_pin, GPIO_OUTPUT_ACTIVE);
    if (ret != 0) return ret;

    return 0;
}

SYS_INIT(init_load_switch, PRE_KERNEL_2, 80);

/* The external flash needs 3.3 V during MCUboot. Program the LDO after I2C
 * initialization but before SPI NOR initialization (priority 80). */
static int init_3v3_regulator(void)
{
    uint8_t status;
    int ret;

    /* Never replay a snapshot left by an interrupted previous boot. */
    (void)boot_charger_take(&NRF_POWER->GPREGRET[1]);
    if (!device_is_ready(charger_i2c)) return -ENODEV;
    k_busy_wait(1000); /* CD needs up to 1 ms to wake the I2C interface. */
    ret = read_charger_ldo(&status);
    if (ret) return ret;
    k_busy_wait(100);
    if (status & 0x80) {
        ret = i2c_reg_write_byte(charger_i2c, CHARGER_ADDR, CHARGER_LDO_CTRL, status & ~0x80);
        if (ret) return ret;
        k_busy_wait(100);
        ret = read_charger_ldo(&status);
        if (ret || (status & 0x80)) return -EIO;
        k_busy_wait(100);
    }
    if ((status & CHARGER_LDO_MASK) != CHARGER_LDO_3V3) {
        ret = i2c_reg_write_byte(charger_i2c, CHARGER_ADDR, CHARGER_LDO_CTRL,
                                 (status & 0x01) | CHARGER_LDO_3V3);
        if (ret) return ret;
        k_busy_wait(100);
        ret = read_charger_ldo(&status);
        if (ret || (status & (0x80 | CHARGER_LDO_MASK)) != CHARGER_LDO_3V3) return -EIO;
    }
    ret = gpio_pin_set_dt(&ls_3_3_pin, 1);
    if (ret) return ret;
    k_busy_wait(600);
    k_work_init_delayable(&charger_watchdog_work, charger_watchdog_handler);
    k_work_schedule(&charger_watchdog_work, K_SECONDS(10));
    return 0;
}

SYS_INIT(init_3v3_regulator, POST_KERNEL, 60);

/*
int wait() {
    k_msleep(1);

    return 0;
}

SYS_INIT(wait, POST_KERNEL, 80);*/
