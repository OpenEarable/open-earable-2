/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <nrfx_clock.h>

#include "led.h"
//#include "button_handler.h"
//#include "button_manager.h"
#include "button_assignments.h"
#include "sd_card.h"
#include "board_version.h"
#include "channel_assignment.h"

#include "sd_card_playback.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nrf5340_audio_dk, CONFIG_MODULE_NRF5340_AUDIO_DK_LOG_LEVEL);

int openearable_init(void)
{
	int ret;

	/*ret = led_init();
	if (ret) {
		LOG_ERR("Failed to initialize LED module");
		return ret;
	}*/

	/*ret = button_handler_init();
	if (ret) {
		LOG_ERR("Failed to initialize button handler");
		return ret;
	}*/

	/*ret = channel_assign_check();
	if (ret) {
		LOG_ERR("Failed get channel assignment");
		return ret;
	}*/

	/*
	if (board_rev.mask & BOARD_VERSION_VALID_MSK_SD_CARD) {
		ret = sd_card_init();
		if (ret != -ENODEV && ret != 0) {
			LOG_ERR("Failed to initialize SD card");
			return ret;
		}
	}

	if (IS_ENABLED(CONFIG_SD_CARD_PLAYBACK)) {
		ret = sd_card_playback_init();
		if (ret) {
			LOG_ERR("Failed to initialize SD card playback");
			return ret;
		}
	}*/

	/* Use this to turn on 128 MHz clock for cpu_app */
	ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);
	ret -= NRFX_ERROR_BASE_NUM;
	if (ret) {
		return ret;
	}

	return 0;
}
