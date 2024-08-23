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

#include "device_info.h"
#include "battery_service.h"
#include "button_service.h"
#include "sensor_service.h"
#include "led_service.h"

//#include "unicast_server.h"

#include "nrf.h"

#include "streamctrl.h"

#include <zephyr/pm/device.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device_runtime.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

#define BUFFER_SIZE (16 * 1024)

size_t size = BUFFER_SIZE;
char buf[BUFFER_SIZE];

int main(void) {
	int ret;

	LOG_DBG("nRF5340 APP core started");

	ret = power_manager.begin();
	ERR_CHK(ret);

	streamctrl_start();

	ret = pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(load_switch)));
	ret = pm_device_runtime_get(DEVICE_DT_GET(DT_NODELABEL(load_switch_sd)));

	k_msleep(50);

	LOG_INF("SD initing .........................................");

	ret = sd_card_init();
	LOG_INF("SD initialization: %i", ret);
	if (ret != -ENODEV) {
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
	}

	led_service.begin();

	earable_btn.begin();

	/*if (board_rev.mask & BOARD_VERSION_VALID_MSK_SD_CARD) {
		ret = sd_card_init();
		if (ret != -ENODEV) {
			ERR_CHK(ret);
		}
	}*/

	start_sensor_manager();

	//sensor_config imu = {ID_IMU, 80, 0};
	sensor_config imu = {ID_PPG, 400, 0};

	//config_sensor(&imu);

	ret = init_battery_service();
	ERR_CHK(ret);

	ret = init_button_service();
	ERR_CHK(ret);

	ret = init_sensor_service();
	ERR_CHK(ret);

	// error test
	//long *a = nullptr;
	//*a = 10;
}