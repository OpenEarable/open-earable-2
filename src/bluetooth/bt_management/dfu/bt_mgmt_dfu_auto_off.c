#include "AutoOffManager.h"
#include "bt_mgmt_dfu_auto_off.h"

#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_mgmt_dfu_auto_off, CONFIG_BT_MGMT_DFU_LOG_LEVEL);

#ifdef CONFIG_MCUMGR_MGMT_NOTIFICATION_HOOKS

/*
 * Avoid auto-off during DFU (Device Firmware Update). Since we cannot identify
 * a clear end of every firmware upload reliably, use a timed latch of the
 * auto-off veto, refreshed for every upload chunk.
 */
static const char dfu_auto_off_token[] = "DFU";
#define DFU_CHUNK_RECEIVED_LATCH_TIME K_MINUTES(5)

static void dfu_auto_off_release_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(dfu_auto_off_release_work, dfu_auto_off_release_work_handler);

static void dfu_auto_off_release_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	auto_off_allow(dfu_auto_off_token);
}

void bt_mgmt_dfu_auto_off_hold(void)
{
	auto_off_prohibit(dfu_auto_off_token);
	(void)k_work_reschedule(&dfu_auto_off_release_work, DFU_CHUNK_RECEIVED_LATCH_TIME);
}

int bt_mgmt_dfu_auto_off_init(void)
{
	int ret;

	ret = auto_off_register_participant(dfu_auto_off_token, POWER_SAVING_LEVEL_AGGRESSIVE);
	if (ret && ret != -EALREADY) {
		LOG_WRN("Failed to register DFU with auto-off: %d", ret);
		return ret;
	}

	auto_off_allow(dfu_auto_off_token);

	return 0;
}

SYS_INIT(bt_mgmt_dfu_auto_off_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif
