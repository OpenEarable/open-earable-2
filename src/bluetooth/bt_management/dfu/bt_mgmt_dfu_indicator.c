#include "bt_mgmt_dfu_indicator.h"

#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_mgmt_dfu_indicator, CONFIG_BT_MGMT_DFU_LOG_LEVEL);

#define DFU_CHUNK_LED_PULSE_TIME K_MSEC(30)

static const struct gpio_dt_spec dfu_activity_led =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(led_error), gpios, {0});
static bool dfu_activity_led_ready;

static void dfu_activity_led_release_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(dfu_activity_led_release_work, dfu_activity_led_release_work_handler);

static void dfu_activity_led_release_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!dfu_activity_led_ready) {
		return;
	}

	(void)gpio_pin_set_dt(&dfu_activity_led, 0);
}

void bt_mgmt_dfu_indicator_update_chunk_led(void)
{
	if (!dfu_activity_led_ready) {
		return;
	}

	(void)gpio_pin_set_dt(&dfu_activity_led, 1);
	(void)k_work_reschedule(&dfu_activity_led_release_work, DFU_CHUNK_LED_PULSE_TIME);
}

int bt_mgmt_dfu_indicator_init(void)
{
	int ret;

	dfu_activity_led_ready = device_is_ready(dfu_activity_led.port);
	if (!dfu_activity_led_ready) {
		LOG_WRN("DFU activity LED not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&dfu_activity_led, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_WRN("Failed to configure DFU activity LED: %d", ret);
		dfu_activity_led_ready = false;
		return ret;
	}

	return 0;
}

static int bt_mgmt_dfu_indicator_sys_init(void)
{
	int ret = bt_mgmt_dfu_indicator_init();

	if (ret && ret != -ENODEV) {
		return ret;
	}

	return 0;
}

SYS_INIT(bt_mgmt_dfu_indicator_sys_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
