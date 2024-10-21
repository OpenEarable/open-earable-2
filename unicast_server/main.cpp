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

#include "../src/modules/sd_card.h"

#include <zephyr/settings/settings.h>

#include <zephyr/zbus/zbus.h>

#include "macros_common.h"
#include "nrf5340_audio_common.h"
#include "streamctrl.h"

#include "../src/Battery/PowerManager.h"
#include "../src/SensorManager/SensorManager.h"
#include "../src/buttons/Button.h"
#include "../src/utils/StateIndicator.h"

#include "device_info.h"
#include "battery_service.h"
#include "button_service.h"
#include "sensor_service.h"
#include "led_service.h"

//#include "unicast_server.h"

// #include "nrf.h"

#include "streamctrl.h"

#include <zephyr/pm/device.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device_runtime.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

/*#define BUFFER_SIZE (16 * 1024)

size_t size = BUFFER_SIZE;
char buf[BUFFER_SIZE];*/

int bonded_device_count = 0;

void count_bonds(const bt_bond_info *info, void *user_data) {
	bonded_device_count++;
}

int main(void) {
	int ret;

	LOG_DBG("nRF5340 APP core started");

	ret = power_manager.begin();
	ERR_CHK(ret);

	streamctrl_start();

	//LOG_INF("Bonded devices: %i", bonded_device_count);

	/*LoadSwitch ls_sd_dev = LoadSwitch(GPIO_DT_SPEC_GET(DT_NODELABEL(load_switch_sd), gpios));
	LoadSwitch ls_1_8_dev = LoadSwitch(GPIO_DT_SPEC_GET(DT_NODELABEL(load_switch), gpios));

	//ls_sd_dev.begin();
	//ls_1_8_dev.begin();

	ret = pm_device_runtime_get(ls_1_8);
	ret = pm_device_runtime_get(ls_sd);

	//ls_sd_dev.set(true);
	//ls_1_8_dev.set(true);

	//k_msleep(10);

	// LOG_INF("SD power on: %i", ls_sd_dev.is_on());
	// LOG_INF("1.8V power on: %i", ls_1_8_dev.is_on());

	// LOG_INF("SD initing .........................................");

	//const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(sdhc0));

	const struct gpio_dt_spec state_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(sd_state), gpios);

	ret = device_is_ready(state_pin.port); //bool
	if (!ret) {
		LOG_INF("SD state pins not ready.\n");
		return -1;
	}

	ret = gpio_pin_configure_dt(&state_pin, GPIO_INPUT);
	if (ret != 0) {
		LOG_INF("Failed to set state pin as input.\n");
		return ret;
	}

	for (int i = 0; i < 4; i++) {
		LOG_INF("sd state: %i\n", gpio_pin_get_dt(&state_pin));
	}

	ret = sd_card_init();
	for (int i = 0; i < 10; i++) {
		LOG_INF("%i\n", ret);
	}*/
	
	/*if (ret != -ENODEV) {
		LOG_INF("SD successful: %i", ret);

		//sd_card_list_files(&path, buffer, &size);
		//printk("%s", buf);

		struct fs_file_t f_entry;

		ret = sd_card_open("play_file_2.wav", &f_entry);
		int start = k_cyc_to_us_floor32(k_cycle_get_32());
		ret = sd_card_read(buf, &size, &f_entry);

		int end = k_cyc_to_us_floor32(k_cycle_get_32());

		sd_card_close(&f_entry);

		LOG_INF("File closed.");

		double duration = end - start;

		double throuput = (double) (BUFFER_SIZE) / duration;

		LOG_INF("Duration: %i", end - start);
		LOG_INF("Troughput: %.5f MB/s", throuput);
	} else {
		LOG_WRN("SD failed\n");
	}*/

	led_service.begin();

	bt_foreach_bond(BT_ID_DEFAULT, count_bonds, NULL);

	if (bonded_device_count > 0) {
		state_indicator.init(PAIRED);
	} else {
		state_indicator.init(UNPAIRED);
	}

	/*if (board_rev.mask & BOARD_VERSION_VALID_MSK_SD_CARD) {
		ret = sd_card_init();
		if (ret != -ENODEV) {
			ERR_CHK(ret);
		}
	}*/

	start_sensor_manager();

	//sensor_config imu = {ID_IMU, 80, 0};
	/*sensor_config imu = {ID_PPG, 400, 0};
	sensor_config temp = {ID_OPTTEMP, 10, 0};

	config_sensor(&temp);*/

	ret = init_battery_service();
	ERR_CHK(ret);

	ret = init_button_service();
	ERR_CHK(ret);

	ret = init_sensor_service();
	ERR_CHK(ret);

	// error test
	//long *a = nullptr;
	//*a = 10;

	return 0;
}