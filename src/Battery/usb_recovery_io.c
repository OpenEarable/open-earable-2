/* SPDX-License-Identifier: Apache-2.0 */
#include "usb_recovery.h"
#include "usb_recovery_io.h"

#include <errno.h>
#include <hal/nrf_reset.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/usb/usb_device.h>

LOG_MODULE_REGISTER(usb_recovery, LOG_LEVEL_INF);

BUILD_ASSERT(CONFIG_I2C_NRFX_TRANSFER_TIMEOUT > 0,
	     "USB recovery requires bounded I2C transfers");

static const struct gpio_dt_spec pg = GPIO_DT_SPEC_GET(DT_NODELABEL(bq25120a), pg_gpios);
static const struct gpio_dt_spec cd = GPIO_DT_SPEC_GET(DT_NODELABEL(bq25120a), cd_gpios);
static const struct device *const bus = DEVICE_DT_GET(DT_NODELABEL(i2c1));

int usb_recovery_io_init(void)
{
	if (!gpio_is_ready_dt(&pg) || !gpio_is_ready_dt(&cd)) {
		return -ENODEV;
	}
	return gpio_pin_configure_dt(&pg, GPIO_INPUT);
}

int usb_recovery_io_power_present(void)
{
	return gpio_pin_get_dt(&pg);
}

int usb_recovery_io_inhibit_charge(void)
{
	/* BQ25120A CD=HIGH disables charging with valid VIN, but leaves the USB
	 * power path supplying SYS. Do not toggle CD or reset the charger here. */
	int ret = gpio_pin_configure_dt(&cd, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		return ret;
	}
	ret = gpio_pin_get_dt(&cd);
	return ret < 0 ? ret : (ret == 1 ? 0 : -EIO);
}

static int read_gauge(uint8_t reg, uint16_t *value)
{
	uint8_t bytes[2];
	int ret = i2c_burst_read(bus, DT_REG_ADDR(DT_NODELABEL(bq27220)), reg,
			       bytes, sizeof(bytes));
	k_usleep(1000);
	if (!ret) {
		*value = sys_get_le16(bytes);
	}
	return ret;
}

int usb_recovery_io_read_snapshot(struct usb_recovery_snapshot *s)
{
	if (!device_is_ready(bus)) {
		return -ENODEV;
	}
	/* Only main accesses this bus before PowerManager::begin(). None of the
	 * normal battery callbacks or gauge configuration loops has started. */
	/* A freshly powered gauge needs time to initialize. Keep CD high while
	 * waiting, with a bounded retry count and bounded individual transfers. */
	int ret;
	for (int attempt = 0; attempt < 10; ++attempt) {
		ret = read_gauge(0x3a, &s->operation_status);
		if (ret || (s->operation_status & (1U << 5))) {
			break;
		}
		k_sleep(K_MSEC(100));
	}
	if (ret) {
		return ret;
	}
	ret = read_gauge(0x08, &s->voltage_mv);
	if (!ret) {
		ret = read_gauge(0x06, &s->temperature_dk);
	}
	if (!ret) {
		ret = read_gauge(0x0a, &s->battery_flags);
	}
	/* Do not read charger status: its read-to-clear reset flag belongs to the
	 * normal button/boot logic. CD alone controls inhibition in this mode. */
	LOG_INF("Battery %u mV, temperature %u dK, flags 0x%04x, op 0x%04x, read %d",
		s->voltage_mv, s->temperature_dk, s->battery_flags, s->operation_status, ret);
	return ret;
}

static void recovery_power_off(void)
{
	(void)usb_disable();
	(void)pm_device_action_run(DEVICE_DT_GET(DT_NODELABEL(load_switch_sd)),
				   PM_DEVICE_ACTION_SUSPEND);
	(void)pm_device_action_run(DEVICE_DT_GET(DT_CHILD(DT_NODELABEL(bq25120a), load_switch)),
				   PM_DEVICE_ACTION_SUSPEND);
	(void)pm_device_action_run(DEVICE_DT_GET(DT_NODELABEL(load_switch)),
				   PM_DEVICE_ACTION_SUSPEND);
	(void)pm_device_action_run(DEVICE_DT_GET(DT_CHOSEN(zephyr_console)),
				   PM_DEVICE_ACTION_SUSPEND);
	/* CD remains HIGH, including if USB returns during shutdown. Do not use
	 * PowerManager::power_down(): its normal services were never initialized. */
	int ret = gpio_pin_interrupt_configure_dt(&pg, GPIO_INT_LEVEL_ACTIVE);
	if (ret || usb_recovery_io_power_present() != 0) {
		sys_reboot(SYS_REBOOT_COLD);
	}
	sys_poweroff();
}

void usb_recovery_run(void)
{
	LOG_WRN("USB recovery: charging disabled; audio, Bluetooth and sensors not started");
	nrf_reset_network_force_off(NRF_RESET, true);
	int ret = usb_enable(NULL);
	if (ret) {
		LOG_ERR("USB recovery enumeration failed: %d", ret);
	}
	for (;;) {
		ret = usb_recovery_poll();
		if (ret <= 0) {
			if (ret < 0) {
				LOG_ERR("USB recovery power check failed: %d", ret);
			}
			recovery_power_off();
		}
		k_sleep(K_MSEC(100));
	}
}
