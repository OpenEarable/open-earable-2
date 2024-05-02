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

#include "macros_common.h"
#include "nrf5340_audio_common.h"
/*#include "cs47l63.h"
#include "cs47l63_spec.h"
#include "cs47l63_reg_conf.h"
#include "cs47l63_comm.h"*/

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hw_codec, CONFIG_MODULE_HW_CODEC_LOG_LEVEL);

#define VOLUME_ADJUST_STEP_DB 3
#define BASE_10		      10

ZBUS_SUBSCRIBER_DEFINE(volume_evt_sub, CONFIG_VOLUME_MSG_SUB_QUEUE_SIZE);

/*static uint32_t prev_volume_reg_val = OUT_VOLUME_DEFAULT;

static cs47l63_t cs47l63_driver;*/

static k_tid_t volume_msg_sub_thread_id;
static struct k_thread volume_msg_sub_thread_data;

K_THREAD_STACK_DEFINE(volume_msg_sub_thread_stack, CONFIG_VOLUME_MSG_SUB_STACK_SIZE);

/**
 * @brief	Convert the zbus volume to the actual volume setting for the HW codec.
 *
 * @note	The range for zbus volume is from 0 to 255 and the
 *		range for HW codec volume is from 0 to 128.
 */
static uint16_t zbus_vol_conversion(uint8_t volume)
{
	return (((uint16_t)volume + 1) / 2);
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

/**
 * @brief Write to multiple registers in CS47L63.
 */
static int cs47l63_comm_reg_conf_write(const uint32_t config[][2], uint32_t num_of_regs)
{
	int ret;
	uint32_t reg;
	uint32_t value;

	/*for (int i = 0; i < num_of_regs; i++) {
		reg = config[i][0];
		value = config[i][1];

		if (reg == SPI_BUSY_WAIT) {
			LOG_DBG("Busy waiting instead of writing to CS47L63");
			// Wait for us defined in value
			k_busy_wait(value);
		} else {
			ret = cs47l63_write_reg(&cs47l63_driver, reg, value);
			if (ret) {
				return ret;
			}
			return 0;
		}
	}*/

	return 0;
}

int hw_codec_volume_set(uint8_t set_val)
{
	return 0;
}

int hw_codec_volume_adjust(int8_t adjustment_db)
{
	return 0;
}

int hw_codec_volume_decrease(void)
{
	return 0;
}

int hw_codec_volume_increase(void)
{
	return 0;
}

int hw_codec_volume_mute(void)
{
	return 0;
}

int hw_codec_volume_unmute(void)
{
	return 0;
}

int hw_codec_default_conf_enable(void)
{
	return 0;
}

int hw_codec_soft_reset(void)
{
	return 0;
}

int hw_codec_init(void)
{
	return 0;
}

static int cmd_input(const struct shell *shell, size_t argc, char **argv)
{
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(hw_codec_cmd,
			       SHELL_COND_CMD(CONFIG_SHELL, input, NULL,
					      " Select input\n\t0: LINE_IN\n\t\t1: PDM_MIC",
					      cmd_input),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(hw_codec, &hw_codec_cmd, "Change settings on HW codec", NULL);
