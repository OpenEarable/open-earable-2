#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include <zephyr/settings/settings.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
extern "C" {
#include <sample_usbd.h>
}
#else
#include <zephyr/usb/usb_device.h>
#endif

#include "macros_common.h"
#include "openearable_common.h"
#include "streamctrl.h"
#include "uicr.h"
#include "bt_mgmt.h"
#include "bt_mgmt_conn_interval.h"
#include "conn_interval/conn_intvl_linear.h"
#include "time_sync.h"

#include "device_info.h"
#include "battery_service.h"
#include "button_service.h"
#include "sensor_service.h"
#include "led_service.h"

#include "SensorScheme.h"
#include "DefaultSensors.h"

#include "Battery/PowerManager.h"
#include "SensorManager/SensorManager.h"
#include "SD_Card/SD_Card_Manager/SD_Card_Manager.h"
#include "utils/StateIndicator.h"

LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
/* Register SD card as a USB Mass Storage LUN */
USBD_DEFINE_MSC_LUN(sd, "SD", "OpenEarable", "SD Card", "1.00");

/* Global USB device context, used by SDCardManager to disable/enable USB
 * when the firmware needs exclusive SD card access for logging. */
struct usbd_context *g_usbd;
#endif

int main(void) {
	int ret;

	LOG_DBG("nRF5340 APP core started");

	ret = power_manager.begin();
	ERR_CHK(ret);

	uint8_t standalone = uicr_standalone_get();
	LOG_INF("Standalone mode: %i", standalone);

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
	{
		/* Bring SD rails up (if a card is present) before disk probe and USB
		 * enable so MSC sees the card on first host poll. */
		sdcard_manager.init();
		ret = disk_access_init("SD");
		LOG_INF("disk_access_init(SD)=%d", ret);

		g_usbd = sample_usbd_init_device(NULL);
		if (g_usbd == NULL) {
			LOG_ERR("Failed to initialize USB device");
		} else {
			ret = usbd_enable(g_usbd);
			if (ret) {
				LOG_ERR("Failed to enable USB: %d", ret);
			}
		}
	}
#elif defined(CONFIG_USB_DEVICE_STACK)
	ret = usb_enable(NULL);
	if (ret) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}
#endif

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

	bt_mgmt_conn_interval_init(new ConnIntvlLinear(
	    4,                // linear increase step (8ms units)
	    CONFIG_BLE_ACL_CONN_INTERVAL,
	    CONFIG_BLE_ACL_CONN_INTERVAL_SLOW
	));

	ret = init_time_sync();
	ERR_CHK(ret);

	return 0;
}
