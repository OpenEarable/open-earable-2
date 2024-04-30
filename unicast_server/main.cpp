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
#include "streamctrl.h"


#include "../src/SensorManager/SensorManager.h"
#include "../src/led_service/LED_Service.h"
#include "../src/buttons/Button.h"

#include "device_info.h"
#include "battery_service.h"
#include "button_service.h"
#include "sensor_service.h"

#include "unicast_server.h"

#include "nrf.h"

#include "../src/Battery/PowerManager.h"
#include "../src/SensorManager/MAX30102/MAX30102.h"

#include "streamctrl.h"


#include <zephyr/pm/device.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device_runtime.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

static MAX30105 ppg;

int main(void) {
	int ret;

	/*const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    pm_device_action_run(cons, PM_DEVICE_ACTION_TURN_ON);

	const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    pm_device_action_run(i2c, PM_DEVICE_ACTION_TURN_ON);*/

	LOG_DBG("nRF5340 APP core started");

	//ret = hfclock_config_and_start();
	//ERR_CHK(ret);

	/*const struct gpio_dt_spec button_pin = {
        .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
        .pin = 15, //9
        .dt_flags = GPIO_ACTIVE_LOW
	};

	if (!device_is_ready(button_pin.port)) printk("P0.10 not ready.\n");
	ret = gpio_pin_configure_dt(&button_pin, GPIO_INPUT | GPIO_PULL_UP);
	if (ret != 0) printk("Failed to set P0.10 as input.\n");*/

	ret = power_manager.begin();
	ERR_CHK(ret);

	//k_msleep(10);

	//battery_controller.setup();

	power_manager.set_3_3(true);
	power_manager.set_1_8(true);

	streamctrl_start();

	//ret = led_init();
	//ERR_CHK(ret);

	/*const struct gpio_dt_spec g_p0_21_gpio = {
         .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
         .pin = 21,
         .dt_flags = GPIO_ACTIVE_HIGH
 	};

	// external hardware codec
	gpio_pin_configure_dt(&g_p0_21_gpio, GPIO_OUTPUT_LOW);*/
	//gpio_pin_set_dt(&g_p0_21_gpio, 0 or 1);
	//nrf_gpio_cfg_output(4);

	//led_service.begin();
	//earable_led.init();

	/*earable_btn.begin();
	volume_down_btn.begin();
	volume_up_btn.begin();
	four_btn.begin();*/
	//five_btn.begin();

	//ret = button_handler_init();
	//ERR_CHK(ret);

	/*channel_assignment_init();

	ret = channel_assign_check();
	ERR_CHK(ret);*/

	/*ret = fw_info_app_print();
	ERR_CHK(ret);*/

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

	sensor_config imu_config = {ID_IMU, 120, 0};
	sensor_config baro_config = {ID_TEMP_BARO, 20, 0};
	sensor_config ppg_config = {ID_PPG, 400, 0};

	SensorManager::manager.start();
	//SensorManager::manager.config(&baro_config);
	SensorManager::manager.config(&imu_config);
	//SensorManager::manager.config(&ppg_config);

	//fuel_gauge.begin();
	//battery_controller.begin();

	//ret = bt_mgmt_init();
	//ERR_CHK(ret);

	ret = init_battery_service();
	ERR_CHK(ret);

	//ret = init_button_service();
	//ERR_CHK(ret);

	ret = init_sensor_service();
	ERR_CHK(ret);

	/*ret = leds_set();
	ERR_CHK(ret);*/

    /*ret = streamctrl_start();
	ERR_CHK(ret);*/

	/*if (!ppg.begin(Wire1)) {
			LOG_ERR("Could not find a valid MAX30102 sensor, check wiring!\n");
	} else {
			LOG_INF("Success full PPG.\n");
	}*/

	//ppg.setup();
	/*ppg.setup(0x64, 1, 2, 400, 215, 8192); //0x1F

	while(true) {
		if(ppg.check() > 0)
		{
			while(ppg.available()) {
				printk(" %i, %i\n", ppg.getFIFORed(), ppg.getFIFOIR());
				ppg.nextSample();
			}
		}
		else {
			k_msleep(1000/400/2);
		}

		//ppg.safeCheck(250);
				
		//printk(" %i, %i\n", ppg.getRed(), 0);
		//oximter.processPPG(ppg.getRed(), 0); //ppg.getIR()
	}*/

	/*while(true) {
		float voltage = fuel_gauge.voltage();
        printk("Voltage: %.3fV\n", voltage);

		float soc = fuel_gauge.state_of_charge();
        printk("soc: %.3f%%\n", soc);
        
        float current = fuel_gauge.current();
        printk("current: %.3fmA\n", current);
        
        current = fuel_gauge.average_current();
        printk("average current: %.3fmA\n", current);

		battery_controller.exit_high_impedance();

		uint16_t ts_fault = battery_controller.read_ts_fault();
        //printk("charging flags: 0x%x\n", status_2);
        printk("TS_ENABLED: %i, TS: %i\n",
        ts_fault >> 7, (ts_fault >> 5) & 0x3);

        uint16_t status_2 = battery_controller.read_charging_state();
        //printk("charging flags: 0x%x\n", status_2);
        printk("state: %i\n", status_2>>6);

        //k_usleep(66);
        float battery_voltage = battery_controller.read_battery_voltage_control();
        printk("voltage: %.3fV\n", battery_voltage);

        //k_usleep(66);
        struct chrg_state charge_ctrl = battery_controller.read_charging_control();
        printk("charger enabled: %i, %.3fmA\n", charge_ctrl.enabled, charge_ctrl.mAh);

        //k_usleep(66);
        //uint16_t charge_ctrl = read_charging_control();
        chrg_state term_ctrl = battery_controller.read_termination_control();
        printk("term enabled: %i, %.3fmA\n", term_ctrl.enabled, term_ctrl.mAh);
        //printk("state: %i\n", status_2>>6);

		float ldo_voltage = battery_controller.read_ldo_voltage();
        printk("LDO voltage: %.3fV\n", ldo_voltage);

        //k_usleep(66);
        ilim_uvlo uvlo_ilim = battery_controller.read_uvlo_ilim();
        printk("uvlo: %.3fV\n", uvlo_ilim.uvlo_v);
        printk("ilim: %.3fmA\n", uvlo_ilim.lim_mA);

        printk("power good: %i\n", battery_controller.power_connected());

        battery_controller.enter_high_impedance();

        printk("-------------------\n");     

		k_msleep(1000);
	}*/
}