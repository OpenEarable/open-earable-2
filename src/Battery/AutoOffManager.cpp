#include "AutoOffManager.h"

#include <errno.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/zbus/zbus.h>

#include "PowerManager.h"
#include "zbus_common.h"

#include <zephyr/logging/log.h>
// TODO: Reduce amounts of log messages
LOG_MODULE_REGISTER(auto_off, LOG_LEVEL_DBG);

// TODO: Make timeout controllable via app configuration.
#define AUTO_OFF_IDLE_TIMEOUT K_MINUTES(1)

static atomic_t audio_is_streaming;

static void le_audio_evt_handler(const struct zbus_channel *chan);

ZBUS_CHAN_DECLARE(le_audio_chan);
ZBUS_LISTENER_DEFINE(le_audio_evt_listen_pm, le_audio_evt_handler);

K_WORK_DELAYABLE_DEFINE(AutoOffManager::auto_off_work, AutoOffManager::auto_off_work_handler);

int AutoOffManager::init()
{
	int ret;

	ret = zbus_chan_add_obs(&le_audio_chan, &le_audio_evt_listen_pm, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret == -EALREADY) {
		LOG_INF("LE audio auto-off listener already registered");
		return ret;
	}
	if (ret) {
		LOG_WRN("Failed to add LE audio auto-off listener: %d", ret);
		return ret;
	}

	LOG_INF("LE audio auto-off listener registered");
	schedule();

	return 0;
}

void AutoOffManager::schedule()
{
	(void)k_work_reschedule(&auto_off_work, AUTO_OFF_IDLE_TIMEOUT);
	LOG_INF("Auto-off timer armed: %d min", k_ticks_to_sec_near32(AUTO_OFF_IDLE_TIMEOUT.ticks)/60);
}

void AutoOffManager::cancel()
{
	(void)k_work_cancel_delayable(&auto_off_work);
}

void AutoOffManager::auto_off_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (atomic_get(&audio_is_streaming) != 0) {
		LOG_DBG("Auto-off skipped because audio is streaming");
		return;
	}

	LOG_INF("Auto-off timeout reached");
	power_manager.power_down();
}

static void le_audio_evt_handler(const struct zbus_channel *chan)
{
	const struct le_audio_msg *msg;

	msg = (const struct le_audio_msg *)zbus_chan_const_msg(chan);
	LOG_INF("LE audio evt received in auto-off manager: %d", msg->event);

	switch (msg->event) {
	case LE_AUDIO_EVT_STREAMING:
		atomic_set(&audio_is_streaming, 1);
		auto_off_manager.cancel();
		LOG_INF("Auto-off canceled (audio streaming)");
		break;
	case LE_AUDIO_EVT_NOT_STREAMING:
		/*
		 * LE_AUDIO_EVT_NOT_STREAMING is not fired when an audio stream is paused,
		 * only when it is fully stopped.
		 */
		atomic_set(&audio_is_streaming, 0);
		auto_off_manager.schedule();
		break;
	default:
		LOG_INF("LE audio evt ignored for auto-off: %d", msg->event);
		break;
	}
}

AutoOffManager auto_off_manager;
