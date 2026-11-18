/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

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
#include "audio_response_service.h"

#include "SensorScheme.h"
#include "DefaultSensors.h"

#include "time_sync.h"

#include "../src/SD_Card/SDLogger/SDLogger.h"
#include "SDMassStorage.h"

#include "uicr.h"

#include "streamctrl.h"

#include "bt_mgmt.h"

#include "bt_mgmt_conn_interval.h"
#include "conn_interval/conn_intvl_linear.h"

//#include "sd_card.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);
//BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
//	     "Console device is not ACM CDC UART device");

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include <zephyr/storage/disk_access.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
extern "C" {
#include <sample_usbd.h>
}

USBD_DEFINE_MSC_LUN(sd, "SD", "OpenEarable", "SD Card", "1.00");
#else
#include <zephyr/usb/usb_device.h>
#endif


int main(void) {
	int ret;

	LOG_DBG("nRF5340 APP core started");

	ret = power_manager.begin();
	ERR_CHK(ret);

	uint8_t standalone = uicr_standalone_get();

	LOG_INF("Standalone mode: %i", standalone);

	/*sdcard_manager.init();

	sdcard_manager.mount();*/

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	/*
	 * Must run after PowerManager has enabled the SD and SPI level-shifter
	 * rails, since probing the disk requires the card to be powered.
	 */
	ret = sd_mass_storage_init();
	if (ret) {
		LOG_ERR("Failed to initialize SD mass-storage monitoring: %d", ret);
	}

	struct usbd_context *usbd = sample_usbd_init_device(sd_mass_storage_usb_msg_cb);
	if (usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
	} else {
		sd_mass_storage_set_usb_context(usbd);
		ret = usbd_enable(usbd);
		if (ret) {
			LOG_ERR("Failed to enable USB: %d", ret);
		}
	}
#elif defined(CONFIG_USB_DEVICE_STACK)
	{
		ret = usb_enable(NULL);
		if (ret) {
			LOG_ERR("Failed to enable USB");
			return 0;
		}
	}
#endif

	streamctrl_start();

	uint32_t sirk = uicr_sirk_get();
	uint8_t bonded_device_count = 0;
	bt_mgmt_num_bonds_get(&bonded_device_count);
	if (sirk == 0xFFFFFFFFU) {
		state_indicator.set_pairing_state(SET_PAIRING);
	} else if (bonded_device_count > 0 && !oe_boot_state.timer_reset) {
		state_indicator.set_pairing_state(PAIRED);
	} else {
		state_indicator.set_pairing_state(BONDING);
	}

	init_sensor_manager();

    ret = init_led_service();
	ERR_CHK(ret);

	ret = init_battery_service();
	ERR_CHK(ret);

	ret = init_button_service();
	ERR_CHK(ret);

	ret = initParseInfoService(&defaultSensorIds, defaultSensors);
	ERR_CHK(ret);

	ret = init_sensor_service();
	ERR_CHK(ret);

	ret = init_audio_response_service();
	ERR_CHK(ret);

	/* Step by one ISO interval (in 1.25 ms units): an ACL interval that is
	 * not a multiple of it meets a CIS event at a fixed rate, which with
	 * 10 ms frames cost every third frame at 15 ms and every fifth at 25 ms.
	 */
	bt_mgmt_conn_interval_init(new ConnIntvlLinear(
	    CONFIG_AUDIO_FRAME_DURATION_US / 1250,
	    CONFIG_BLE_ACL_CONN_INTERVAL,
	    CONFIG_BLE_ACL_CONN_INTERVAL_SLOW
	));

	ret = init_time_sync();
	ERR_CHK(ret);

	// error test
	//long *a = nullptr;
	//*a = 10;

	return 0;
}
