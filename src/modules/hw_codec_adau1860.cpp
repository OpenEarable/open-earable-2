/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "hw_codec.h"

#include <zephyr/kernel.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/settings/settings.h>

#include "macros_common.h"
#include "zbus_common.h"
#include "openearable_common.h"
#include "../drivers/ADAU1860.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hw_codec, CONFIG_MODULE_HW_CODEC_LOG_LEVEL);

#define VOLUME_ADJUST_STEP_DB 3

ZBUS_SUBSCRIBER_DEFINE(volume_evt_sub, CONFIG_VOLUME_MSG_SUB_QUEUE_SIZE);

static uint32_t prev_volume_reg_val = OUT_VOLUME_DEFAULT;
static bool muted = false;

static k_tid_t volume_msg_sub_thread_id;
static struct k_thread volume_msg_sub_thread_data;

K_THREAD_STACK_DEFINE(volume_msg_sub_thread_stack, CONFIG_VOLUME_MSG_SUB_STACK_SIZE);

static enum audio_mode audio_mode;

static int settings_set_cb(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
    if (strcmp(name, "mode") == 0 && len == sizeof(audio_mode)) {
        int rc = read_cb(cb_arg, &audio_mode, sizeof(audio_mode));
        /*if (rc >= 0) {
            hw_codec_set_audio_mode(audio_mode);
        }*/
		if (rc >= 0) return 0;
		else return rc;
    }
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(audio, "audio", NULL, settings_set_cb, NULL, NULL);

int hw_codec_set_audio_mode(enum audio_mode mode) {
    int ret;

	audio_mode = mode;

	settings_save_one("audio/mode", &mode, sizeof(mode));

	ret = dac.fdsp_bank_select((uint8_t) mode);
	// TODO: make writing to bank work
	k_msleep(200);
	ret = hw_codec_volume_adjust(0);
	ret = dac.mute(muted);
	if (ret) {
		LOG_ERR("Failed to set audio mode, ret: %d", ret);
		return ret;
	}
	return ret;
}

enum audio_mode hw_codec_get_audio_mode() {
	return audio_mode;
}

/**
 * @brief	Convert the zbus volume to the actual volume setting for the HW codec.
 *
 * @note	The range for zbus volume is from 0 to 255 and the
 *		range for HW codec volume is from 0 to 128 + offset.
 */
static uint16_t zbus_vol_conversion(uint8_t volume)
{
	//return (((uint16_t)volume + 1) / 2) + 0x40;
#ifdef CONFIG_FDSP
	return volume;
#else
	return (((int)volume + 1) * (MAX_VOLUME_REG_VAL - MIN_VOLUME_REG_VAL) / 255) + MIN_VOLUME_REG_VAL;
#endif
}

/**
 * @brief	Handle volume events from zbus.
 */
static void volume_msg_sub_thread(void)
{
	int ret;

	const struct zbus_channel *chan;

	while (1) {
		ret = zbus_sub_wait(&volume_evt_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		struct volume_msg msg;

		ret = zbus_chan_read(chan, &msg, ZBUS_READ_TIMEOUT_MS);
		if (ret) {
			LOG_ERR("Failed to read from zbus: %d", ret);
		}

		uint8_t event = msg.event;
		uint8_t volume = msg.volume;

		LOG_DBG("Received event = %d, volume = %d", event, volume);

		switch (event) {
		case VOLUME_UP:
			LOG_DBG("Volume up received");
			ret = hw_codec_volume_increase();
			if (ret) {
				LOG_ERR("Failed to increase volume, ret: %d", ret);
			}
			break;
		case VOLUME_DOWN:
			LOG_DBG("Volume down received");
			ret = hw_codec_volume_decrease();
			if (ret) {
				LOG_ERR("Failed to decrease volume, ret: %d", ret);
			}
			break;
		case VOLUME_SET:
			LOG_DBG("Volume set received");
			ret = hw_codec_volume_set(zbus_vol_conversion(volume));
			if (ret) {
				LOG_ERR("Failed to set the volume to %d, ret: %d", volume, ret);
			}
			break;
		case VOLUME_MUTE:
			LOG_DBG("Volume mute received");
			ret = hw_codec_volume_mute();
			if (ret) {
				LOG_ERR("Failed to mute volume, ret: %d", ret);
			}
			break;
		case VOLUME_UNMUTE:
			LOG_DBG("Volume unmute received");
			ret = hw_codec_volume_unmute();
			if (ret) {
				LOG_ERR("Failed to unmute volume, ret: %d", ret);
			}
			break;
		default:
			LOG_WRN("Unexpected/unhandled volume event: %d", event);
			break;
		}

		STACK_USAGE_PRINT("volume_msg_thread", &volume_msg_sub_thread_data);
	}
}

int hw_codec_volume_set(uint8_t set_val)
{
	int ret;
	uint32_t volume_reg_val;

	volume_reg_val = set_val;
	if (volume_reg_val == 0) {
		LOG_WRN("Volume at MIN (-64dB)");
#ifdef CONFIG_FDSP
	} else if (volume_reg_val >= 255) {
		LOG_WRN("Volume at MAX (0dB)");
	}
#else
	} else if (volume_reg_val >= MAX_VOLUME_REG_VAL) {
		LOG_WRN("Volume at MAX (0dB)");
		volume_reg_val = MAX_VOLUME_REG_VAL;
	}
#endif

	ret = dac.set_volume(volume_reg_val);

	if (ret) {
		return ret;
	}

	prev_volume_reg_val = volume_reg_val;

	/* This is rounded down to nearest integer */
	LOG_DBG("Volume: %" PRId32 " dB", (volume_reg_val / 2) - MAX_VOLUME_DB);

	return 0;
}

int hw_codec_volume_adjust(int8_t adjustment_db)
{
	int ret;
	int32_t new_volume_reg_val;

	LOG_DBG("Adj dB in: %d", adjustment_db);

	if (adjustment_db == 0) {
		new_volume_reg_val = prev_volume_reg_val;
	} else {
		uint32_t volume_reg_val;

		volume_reg_val = dac.get_volume();

		/* The adjustment is in dB, 1 bit equals 0.375 dB,
		 * so multiply by 8/3 to get increments of 1 dB
		 */
		new_volume_reg_val = volume_reg_val + (adjustment_db * 8 / 3);
		if (new_volume_reg_val <= 0) {
			LOG_WRN("Volume at MIN (-64dB)");
			new_volume_reg_val = 0;
		} else if (new_volume_reg_val >= MAX_VOLUME_REG_VAL) {
			LOG_WRN("Volume at MAX (0dB)");
			new_volume_reg_val = MAX_VOLUME_REG_VAL;
		}
	}

	ret = hw_codec_volume_set(new_volume_reg_val);
	if (ret) {
		return ret;
	}

	return 0;
}

int hw_codec_volume_decrease(void)
{
	int ret;

	ret = hw_codec_volume_adjust(-VOLUME_ADJUST_STEP_DB);
	if (ret) {
		return ret;
	}

	return 0;
}

int hw_codec_volume_increase(void)
{
	int ret;

	ret = hw_codec_volume_adjust(VOLUME_ADJUST_STEP_DB);
	if (ret) {
		return ret;
	}

	return 0;
}

int hw_codec_volume_mute(void)
{
	int ret;

	ret = dac.mute(true);

	muted = true;
	
	return ret;
}

int hw_codec_volume_unmute(void)
{
	int ret;

	ret = dac.mute(false);

	muted = false;

	return ret;
}

int hw_codec_default_conf_enable(void)
{
	int ret;

	ret = hw_codec_volume_adjust(0);
	if (ret) {
		return ret;
	}

	//ret = dac.setup();
	if (!muted) {
		ret = dac.mute(false);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

int hw_codec_stop_audio(void)
{
	int ret;

	ret = dac.mute(true);
	if (ret) {
		return ret;
	}

	return 0;
}

int hw_codec_soft_reset(void)
{
	int ret;

	ret = dac.soft_reset();
	if (ret) {
		return ret;
	}

	return 0;
}

int hw_codec_init(void)
{
	int ret;

	ret = dac.begin();
	if (ret) {
		return ret;
	}

	hw_codec_set_audio_mode(audio_mode);

	/* Run a soft reset on start to make sure all registers are default values */
	/*ret = dac.soft_reset();
	if (ret) {
		return ret;
	}*/

	volume_msg_sub_thread_id = k_thread_create(
		&volume_msg_sub_thread_data, volume_msg_sub_thread_stack,
		CONFIG_VOLUME_MSG_SUB_STACK_SIZE, (k_thread_entry_t)volume_msg_sub_thread, NULL,
		NULL, NULL, K_PRIO_PREEMPT(CONFIG_VOLUME_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(volume_msg_sub_thread_id, "VOLUME_MSG_SUB");
	ERR_CHK(ret);

	return 0;
}