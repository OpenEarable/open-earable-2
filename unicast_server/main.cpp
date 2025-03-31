/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

//#include "../src/modules/sd_card.h"

#include <zephyr/settings/settings.h>

#include "macros_common.h"
#include "openearable_common.h"
#include "streamctrl.h"

#include "../src/Battery/PowerManager.h"
#include "../src/SensorManager/SensorManager.h"
#include "../src/utils/StateIndicator.h"

#include "device_info.h"
#include "battery_service.h"
#include "button_service.h"
#include "sensor_service.h"
#include "led_service.h"

#include "SensorScheme.h"
#include "DefaultSensors.h"
#include "../src/SD_Card/SDLogger/SDLogger.h"

#include "uicr.h"

#include "streamctrl.h"

#include "bt_mgmt.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);
//BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
//	     "Console device is not ACM CDC UART device");

void print_bat_info() {

        bat_status status = fuel_gauge.battery_status();
        //printk("flags: 0x%x\n", status);
        LOG_INF("battery peresent: %i", status.BATTPRES);
        LOG_INF("FC / FD: %i / %i", status.FC, status.FD);

        float voltage = fuel_gauge.voltage();
        LOG_INF("Voltage: %.3fV", voltage);

        float temp = fuel_gauge.temperature();
        LOG_INF("temp: %.3f°C", temp);

        float cap = fuel_gauge.capacity();
        LOG_INF("full charge cap: %.3fmAh", cap);

        float minutes = fuel_gauge.time_to_full();
        LOG_INF("full charge time: %ih %02dmin", (int) minutes / 60, (int)minutes % 60);

        minutes = fuel_gauge.time_to_empty();
        LOG_INF("time to empty: %ih %02dmin", (int) minutes / 60, (int)minutes % 60);
        
        float soc = fuel_gauge.state_of_charge();
        LOG_INF("soc: %.3f%", soc);
        
        float current = fuel_gauge.current();
        LOG_INF("current: %.3fmA", current);
        
        current = fuel_gauge.average_current();
        LOG_INF("average current: %.3fmA", current);
        
        float dCAP = fuel_gauge.design_cap();
        LOG_INF("design CAP: %.1fmAh", dCAP);
        
        float rm = fuel_gauge.remaining_cap();
        LOG_INF("remaining CAP: %.1fmAh", rm);
        
        float cc = fuel_gauge.charge_current();
        LOG_INF("charge current: %.1fmA", cc);
        
        float stb = fuel_gauge.standby_current();
        LOG_INF("standby current: %.1fmA", stb);
        
        int n = fuel_gauge.cycle_count();
        LOG_INF("cycle count: %i", n);
        
        op_state state = fuel_gauge.operation_state();
        LOG_INF("security level: %i", state.SEC);
        // printk("init complete: %i\n", state.INITCOMP);
        //printk("calibration: %i\n", state.CALD);
        LOG_INF("edv2: %i", state.EDV2);
        LOG_INF("fcc update on dsg: %i", state.VDQ);
        LOG_INF("cfg update: %i", state.CFG_UPDATE);

        gauge_status g_status = fuel_gauge.gauging_state();

        /*uint16_t gauge_state = 0;
        k_usleep(66);
        //write_command(CMD_GAUGING_STATUS);
        uint16_t command = CMD_GAUGING_STATUS;
        writeReg(address, 0x3E, (uint8_t *) &command, sizeof(command));
        k_usleep(66);
        readReg(address, 0x40, (uint8_t *) &gauge_state, sizeof(gauge_state));
        k_usleep(66);
        //printk("gauge: %x\n", gauge_state);
        printk("edv2: %i, edv1: %i, edv0: %i\n",
        (gauge_state & (1 << 6)) > 0, (bool)(gauge_state & (1 << 5)), (bool)(gauge_state & (1 << 13)));*/

        /*if (state.EDV2) {
                dk_set_led_on(DK_LED1);

                if (g_status.edv1) dk_set_led_off(DK_LED2);
                else dk_set_led_on(DK_LED2);
        } else {
                dk_set_led_off(DK_LED1);

                if (status.FC) {
                        dk_set_led_on(DK_LED2);
                } else {
                        dk_set_led_off(DK_LED2);
                }
        }*/

        //////////////////////////////////

        LOG_INF("-------------------");

        battery_controller.exit_high_impedance();

        //uint8_t dis_ship = 0;
        //writeReg(address_bq25120a, bq25120a_regs[BQ25120A_REG_CTRL], (uint8_t *) &dis_ship, sizeof(dis_ship));

        uint16_t status_2 = battery_controller.read_charging_state();
        //printk("charging flags: 0x%x\n", status_2);
        LOG_INF("state: %i", status_2>>6);

        /*k_usleep(66);
        uint16_t fault = read_fault();
        //printk("charging flags: 0x%x\n", status_2);
        printk("U_ov: %i, U_uv: %i, BAT_uvlo: %i, BAT_OCP: %i, RESET_FAULT: %i, TIMER_FAULT: %i, VINDPM_STAT: %i, CD_STAT: %i, SYS_EN_STAT: %i\n",
        fault >> 7, (fault >> 6) & 0x1, (fault >> 5) & 0x1, (fault >> 4) & 0x1,
        (status_2 >> 4) & 0x1, (status_2 >> 3) & 0x1, (status_2 >> 2) & 0x1, (status_2 >> 1) & 0x1, status_2 & 0x1);

        k_usleep(66);
        uint16_t ts_fault = read_ts_fault();
        //printk("charging flags: 0x%x\n", status_2);
        printk("TS_ENABLED: %i, TS: %i\n",
        ts_fault >> 7, (ts_fault >> 5) & 0x3);*/

        /*ts_fault &= ~(1 << 7);

        writeReg(address_bq25120a, bq25120a_regs[BQ25120A_REG_TS_FAULT], (uint8_t *) &ts_fault, sizeof(ts_fault));*/

        //k_usleep(66);
        float batter_voltage = battery_controller.read_battery_voltage_control();
        LOG_INF("voltage: %.3fV", batter_voltage);

        //k_usleep(66);
        struct chrg_state charge_ctrl = battery_controller.read_charging_control();
        LOG_INF("charger enabled: %i, %.3fmA", charge_ctrl.enabled, charge_ctrl.mAh);

        //k_usleep(66);
        //uint16_t charge_ctrl = read_charging_control();
        chrg_state term_ctrl = battery_controller.read_termination_control();
        LOG_INF("term enabled: %i, %.3fmA", term_ctrl.enabled, term_ctrl.mAh);
        //printk("state: %i\n", status_2>>6);

        //k_usleep(66);
        ilim_uvlo uvlo_ilim = battery_controller.read_uvlo_ilim();
        LOG_INF("uvlo: %.3fV", uvlo_ilim.uvlo_v);
        LOG_INF("ilim: %.3fmA", uvlo_ilim.lim_mA);

        LOG_INF("power good: %i", battery_controller.power_connected());

        battery_controller.enter_high_impedance();

        LOG_INF("-------------------");     
}

int main(void) {
	int ret;

	LOG_DBG("nRF5340 APP core started");

	ret = power_manager.begin();
	ERR_CHK(ret);

	streamctrl_start();

        uint32_t sirk = uicr_sirk_get();

	if (sirk == 0xFFFFFFFFU) {
                state_indicator.set_pairing_state(SET_PAIRING);
        } else if (bonded_device_count > 0 && !oe_boot_state.timer_reset) {
                state_indicator.set_pairing_state(PAIRED);
	} else {
                state_indicator.set_pairing_state(BONDING);
	}

	init_sensor_manager();

	//sensor_config imu = {ID_IMU, 80, 0};
	//sensor_config imu = {ID_PPG, 400, 0};
	//sensor_config temp = {ID_OPTTEMP, 10, 0};
        // sensor_config temp = {ID_BONE_CONDUCTION, 100, 0};

	//config_sensor(&temp);

	//sensor_config ppg = {ID_PPG, 400, 0};
	//config_sensor(&ppg);

        ret = init_led_service();
	ERR_CHK(ret);

	ret = init_battery_service();
	ERR_CHK(ret);

	ret = init_button_service();
	ERR_CHK(ret);

	ret = init_sensor_service();
	ERR_CHK(ret);

	ret = initParseInfoService(&defaultSensorIds, defaultSensors);
	ERR_CHK(ret);

	// error test
	//long *a = nullptr;
	//*a = 10;

	return 0;
}