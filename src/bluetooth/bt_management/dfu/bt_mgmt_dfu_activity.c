#include "bt_mgmt_dfu_activity.h"

#include <zephyr/init.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>

#include "bt_mgmt_dfu_auto_off.h"
#include "bt_mgmt_dfu_indicator.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_mgmt_dfu_activity, CONFIG_BT_MGMT_DFU_LOG_LEVEL);

#ifdef CONFIG_MCUMGR_MGMT_NOTIFICATION_HOOKS

static struct mgmt_callback dfu_activity_mgmt_cb;

void bt_mgmt_dfu_activity_chunk_received(void)
{
	bt_mgmt_dfu_auto_off_hold();
	bt_mgmt_dfu_indicator_update_chunk_led();
}

static enum mgmt_cb_return dfu_activity_mgmt_callback(uint32_t event,
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
		bt_mgmt_dfu_activity_chunk_received();
	}

	return MGMT_CB_OK;
}

#endif

int bt_mgmt_dfu_activity_init(void)
{
#ifdef CONFIG_MCUMGR_MGMT_NOTIFICATION_HOOKS
	dfu_activity_mgmt_cb.callback = dfu_activity_mgmt_callback;
	dfu_activity_mgmt_cb.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK;
	mgmt_callback_register(&dfu_activity_mgmt_cb);
#endif

	return 0;
}

SYS_INIT(bt_mgmt_dfu_activity_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
