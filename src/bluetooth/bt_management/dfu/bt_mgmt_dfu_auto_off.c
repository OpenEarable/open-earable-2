#include "AutoOffManager.h"

#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>

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

static struct mgmt_callback dfu_auto_off_mgmt_cb;

static void dfu_auto_off_release_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(dfu_auto_off_release_work, dfu_auto_off_release_work_handler);

static void dfu_auto_off_release_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	auto_off_allow(dfu_auto_off_token);
}

static void dfu_auto_off_hold(void)
{
	auto_off_prohibit(dfu_auto_off_token);
	(void)k_work_reschedule(&dfu_auto_off_release_work, DFU_CHUNK_RECEIVED_LATCH_TIME);
}

static enum mgmt_cb_return dfu_auto_off_mgmt_callback(uint32_t event,
						      enum mgmt_cb_return prev_status,
						      int32_t *rc, uint16_t *group,
						      bool *abort_more, void *data,
						      size_t data_size)
{
	ARG_UNUSED(prev_status);
	ARG_UNUSED(rc);
	ARG_UNUSED(group);
	ARG_UNUSED(abort_more);
	ARG_UNUSED(data);
	ARG_UNUSED(data_size);

	if (event == MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK) {
		dfu_auto_off_hold();
	}

	return MGMT_CB_OK;
}

static int bt_mgmt_dfu_auto_off_init(void)
{
	int ret;

	ret = auto_off_register_participant(dfu_auto_off_token, POWER_SAVING_LEVEL_AGGRESSIVE);
	if (ret && ret != -EALREADY) {
		LOG_WRN("Failed to register DFU with auto-off: %d", ret);
		return ret;
	}

	auto_off_allow(dfu_auto_off_token);

	dfu_auto_off_mgmt_cb.callback = dfu_auto_off_mgmt_callback;
	dfu_auto_off_mgmt_cb.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK;
	mgmt_callback_register(&dfu_auto_off_mgmt_cb);

	return 0;
}

SYS_INIT(bt_mgmt_dfu_auto_off_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif
