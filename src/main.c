/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

#define I2C_DEV_LABEL "I2C_1" // Change this to your I2C device label
#define START_ADDRESS 0x40    // Start address for scanning (0x03 is the first valid address)
#define END_ADDRESS   0x45    // End address for scanning (0x77 is the last valid address)

void i2c_scan(const struct device *i2c_dev) {
    printk("Scanning I2C bus...\n");

    for (uint8_t i = START_ADDRESS; i <= END_ADDRESS; i++) {
        struct i2c_msg msgs[1];
        uint8_t dst;

        msgs[0].buf = &dst;
        msgs[0].len = 0U; // Zero-length read
        msgs[0].flags = I2C_MSG_READ | I2C_MSG_STOP;

        if (i2c_transfer(i2c_dev, &msgs[0], 1, i) == 0) {
            printk("I2C device detected at address 0x%02X\n", i);
        }
    }

    printk("I2C scan complete\n");
}

int main(void) {
    const struct device *i2c_dev;

    i2c_dev = device_get_binding(I2C_DEV_LABEL);
    if (!i2c_dev) {
        printk("I2C: Device driver not found.\n");
        return;
    }

    i2c_scan(i2c_dev);
}