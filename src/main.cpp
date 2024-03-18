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

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/settings/settings.h>

#include <zephyr/zbus/zbus.h>

#include "macros_common.h"
#include "nrf5340_audio_common.h"
#include "fw_info_app.h"
#include "led.h"
//#include "button_handler.h"
//#include "button_assignments.h"
#include "nrfx_clock.h"
#include "sd_card.h"
#include "bt_mgmt.h"
//#include "board_version.h"
#include "channel_assignment.h"
#include "streamctrl.h"
#include "sd_card_playback.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

#include "SensorManager/SensorManager.h"
//#include "led_service/LED.h"
#include "led_service/LED_Service.h"
#include "buttons/Button.h"

#include "device_info.h"
#include "battery_service.h"
#include "button_service.h"
#include "sensor_service.h"

#include "nrf.h"

#include "Battery/PowerManager.h"

// BQ27220 fuel_gauge(&Wire);
// BQ25120a charge_controller(&Wire);

//static struct board_version board_rev;

static int hfclock_config_and_start(void)
{
	int ret;

	/* Use this to turn on 128 MHz clock for cpu_app */
	ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

	ret -= NRFX_ERROR_BASE_NUM;
	if (ret) {
		return ret;
	}

	nrfx_clock_hfclk_start();
	while (!nrfx_clock_hfclk_is_running()) {
	}

	return 0;
}

static int leds_set(void)
{
	int ret;

	ret = led_off(0);
	ret = led_off(1);
	ret = led_off(2);

	/* Blink LED 3 to indicate that APP core is running */
	ret = led_blink(3);//LED_APP_3_GREEN);
	if (ret) {
		return ret;
	}

	/*

#if (CONFIG_AUDIO_DEV == HEADSET)
	enum audio_channel channel;

	channel_assignment_get(&channel);

	if (channel == AUDIO_CH_L) {
		ret = led_on(LED_APP_RGB, LED_COLOR_BLUE);
	} else {
		ret = led_on(LED_APP_RGB, LED_COLOR_MAGENTA);
	}

	if (ret) {
		return ret;
	}
#elif (CONFIG_AUDIO_DEV == GATEWAY)
	ret = led_on(LED_APP_RGB, LED_COLOR_GREEN);
	if (ret) {
		return ret;
	}
#endif /* (CONFIG_AUDIO_DEV == HEADSET) */

	return 0;
}

static int channel_assign_check(void)
{
#if (CONFIG_AUDIO_DEV == HEADSET) && CONFIG_AUDIO_HEADSET_CHANNEL_RUNTIME
	//int ret;
	bool pressed;

	//ret = button_pressed(BUTTON_VOLUME_DOWN, &pressed);
	pressed = volume_down_btn.getState() == BUTTON_PRESS;
	printk("%i\n", volume_down_btn.getState());
	/*if (ret) {
		return ret;
	}*/

	if (pressed) {
		channel_assignment_set(AUDIO_CH_L);
		return 0;
	}

	pressed = volume_up_btn.getState() == BUTTON_PRESS;

	//ret = button_pressed(BUTTON_VOLUME_UP, &pressed);
	/*if (ret) {
		return ret;
	}*/

	if (pressed) {
		channel_assignment_set(AUDIO_CH_R);
		return 0;
	}
#endif

	return 0;
}

int main(void) {
	int ret;

	LOG_DBG("nRF5340 APP core started");

	ret = hfclock_config_and_start();
	ERR_CHK(ret);

	const struct gpio_dt_spec button_pin = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
        .pin = 15, //9
        .dt_flags = GPIO_ACTIVE_LOW
	};

	if (!device_is_ready(button_pin.port)) printk("P0.10 not ready.\n");
	ret = gpio_pin_configure_dt(&button_pin, GPIO_INPUT | GPIO_PULL_UP);
	if (ret != 0) printk("Failed to set P0.10 as input.\n");

	ret = power_manager.begin();
	ERR_CHK(ret);

	ret = led_init();
	ERR_CHK(ret);

	const struct gpio_dt_spec g_p0_21_gpio = {
         .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
         .pin = 21,
         .dt_flags = GPIO_ACTIVE_HIGH
 	};

	// external hardware codec
	gpio_pin_configure_dt(&g_p0_21_gpio, GPIO_OUTPUT_LOW);
	//gpio_pin_set_dt(&g_p0_21_gpio, 0 or 1);
	//nrf_gpio_cfg_output(4);

	//led_service.begin();
	//earable_led.init();

	earable_btn.begin();
	volume_down_btn.begin();
	volume_up_btn.begin();
	four_btn.begin();
	//five_btn.begin();

	//ret = button_handler_init();
	//ERR_CHK(ret);

	channel_assignment_init();

	ret = channel_assign_check();
	ERR_CHK(ret);

	ret = fw_info_app_print();
	ERR_CHK(ret);

	/*ret = board_version_valid_check();
	ERR_CHK(ret);

	ret = board_version_get(&board_rev);
	ERR_CHK(ret);

	//info_init();

	if (board_rev.mask & BOARD_VERSION_VALID_MSK_SD_CARD) {
		ret = sd_card_init();
		if (ret != -ENODEV) {
			ERR_CHK(ret);
		}
	}*/

	//k_usleep(100);

	sensor_config imu_config = {ID_IMU, 60, 0};
	sensor_config baro_config = {ID_TEMP_BARO, 20, 0};

	SensorManager::manager.start();
	SensorManager::manager.config(&baro_config);
	SensorManager::manager.config(&imu_config);

	//fuel_gauge.begin();
	//battery_controller.begin();

	ret = bt_mgmt_init();
	ERR_CHK(ret);

	ret = init_button_service();
	ERR_CHK(ret);

	ret = init_sensor_service();
	ERR_CHK(ret);

	ret = leds_set();
	ERR_CHK(ret);

    ret = streamctrl_start();
	ERR_CHK(ret);

	while(true) {
		k_msleep(1000);
		float voltage = fuel_gauge.voltage();
        printk("Voltage: %.3fV\n", voltage);

		float soc = fuel_gauge.state_of_charge();
        printk("soc: %.3f%%\n", soc); //\x0a = %
        
        float current = fuel_gauge.current();
        printk("current: %.3fmA\n", current);
        
        current = fuel_gauge.average_current();
        printk("average current: %.3fmA\n", current);
	}
}